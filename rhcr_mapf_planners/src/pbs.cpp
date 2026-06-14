#include "rhcr_mapf_planners/pbs.h"

#include <ros/ros.h>
#include <unordered_map>
#include <set>
#include <utility>
#include <cmath>
#include <limits>
#include <algorithm>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_state/robot_state.h>

namespace rhcr_mapf_planners
{

PBS::PBS(const moveit::core::RobotModelConstPtr& robot_model,
         const planning_scene::PlanningScenePtr& planning_scene,
         double delta_time,
         double horizon,
         std::size_t max_rrt_nodes)
  : robot_model_(robot_model)
  , planning_scene_(planning_scene)
  , delta_time_(delta_time)
  , horizon_(horizon)
  , max_rrt_nodes_(max_rrt_nodes)
{
  work_state_.reset(new moveit::core::RobotState(robot_model_));
  work_state_->setToDefaultValues();
}

// === Public entry points ===
bool PBS::resolveConflicts(std::vector<AgentInfo>& agents)
{
  return resolveConflicts(agents, 0.0, std::numeric_limits<double>::infinity());
}

bool PBS::resolveConflicts(std::vector<AgentInfo>& agents, double w_start, double w_end)
{
  PBSNode root;
  root.agents = agents;
  root.constraints.clear();

  // Worst case the priority order is a total order, so allow up to O(N^2) branches.
  std::size_t max_depth = agents.size() * agents.size() + agents.size() + 2;

  bool ok = solveNode(root, 0, max_depth, w_start, w_end);
  if (ok)
    agents = root.agents;
  else
    ROS_WARN("PBS::resolveConflicts: no conflict-free solution in window [%.2f, %.2f].",
             w_start, w_end);
  return ok;
}

// === Core recursive solver ===
bool PBS::solveNode(PBSNode& node, std::size_t depth, std::size_t max_depth,
                    double w_start, double w_end)
{
  if (depth > max_depth)
  {
    ROS_WARN("PBS::solveNode: exceeded max depth %zu.", max_depth);
    return false;
  }

  Conflict c = findEarliestConflict(node, w_start, w_end);
  if (!c.valid)
    return true;  // window is conflict-free

  ROS_INFO("PBS: conflict at t=%.3f between %s and %s (depth=%zu).",
           c.t, node.agents[c.a].name.c_str(), node.agents[c.b].name.c_str(), depth);

  // Branch 1: a precedes b (b yields). Skip if it would contradict an existing b precedes a.
  if (!isBefore(node, c.b, c.a))
  {
    PBSNode child = node;
    child.constraints.push_back(PriorityConstraint{c.a, c.b});
    if (replanAndMakeConsistent(child, c.b, c.t, w_start, w_end) &&
        solveNode(child, depth + 1, max_depth, w_start, w_end))
    {
      node = child;
      return true;
    }
  }

  // Branch 2: b precedes a (a yields).
  if (!isBefore(node, c.a, c.b))
  {
    PBSNode child = node;
    child.constraints.push_back(PriorityConstraint{c.b, c.a});
    if (replanAndMakeConsistent(child, c.a, c.t, w_start, w_end) &&
        solveNode(child, depth + 1, max_depth, w_start, w_end))
    {
      node = child;
      return true;
    }
  }

  return false;
}

// === Conflict detection ===
void PBS::buildLinkToAgent(const PBSNode& node,
                           std::unordered_map<std::string, std::size_t>& m) const
{
  m.clear();
  for (std::size_t i = 0; i < node.agents.size(); ++i)
  {
    const auto* jmg = robot_model_->getJointModelGroup(node.agents[i].move_group);
    if (!jmg)
      continue;
    for (const auto& link : jmg->getLinkModelNames())
      m[link] = i;
  }
}

// Place ALL agents at their interpolated positions at time t and collect every
// colliding (agent, agent) pair (ordered low<high). World/self contacts are ignored.
void PBS::collidingPairsAtTime(const PBSNode& node, double t,
                               const std::unordered_map<std::string, std::size_t>& m,
                               std::set<std::pair<std::size_t, std::size_t>>& pairs) const
{
  moveit::core::RobotState& rs = *work_state_;
  rs.setToDefaultValues();

  for (std::size_t ai = 0; ai < node.agents.size(); ++ai)
  {
    const auto* jmg = robot_model_->getJointModelGroup(node.agents[ai].move_group);
    if (!jmg)
      continue;
    std::vector<double> q = getAgentPositionsAtTime(node.agents[ai], t);
    if (q.size() == jmg->getVariableCount())
      rs.setJointGroupPositions(jmg, q);
  }
  rs.update();

  collision_detection::CollisionRequest creq;
  collision_detection::CollisionResult  cres;
  creq.contacts = true;
  creq.max_contacts = 1000;
  creq.max_contacts_per_pair = 1;

  planning_scene_->checkCollision(creq, cres, rs);
  if (!cres.collision)
    return;

  for (const auto& kv : cres.contacts)
  {
    auto it1 = m.find(kv.first.first);
    auto it2 = m.find(kv.first.second);
    if (it1 == m.end() || it2 == m.end())
      continue;                 // contact with the static world: not an inter-agent conflict
    if (it1->second == it2->second)
      continue;                 // self-collision within one agent: ignored in PBS
    std::size_t lo = std::min(it1->second, it2->second);
    std::size_t hi = std::max(it1->second, it2->second);
    pairs.insert(std::make_pair(lo, hi));
  }
}

PBS::Conflict PBS::findEarliestConflict(const PBSNode& node, double w_start, double w_end) const
{
  Conflict result;
  result.valid = false;
  result.a = result.b = 0;
  result.t = 0.0;

  if (node.agents.empty())
    return result;

  std::unordered_map<std::string, std::size_t> link_to_agent;
  buildLinkToAgent(node, link_to_agent);

  double max_time = getMaxTrajectoryTime(node);
  if (max_time <= 0.0)
    return result;

  double t_lo = std::max(0.0, w_start);
  int step_lo = static_cast<int>(std::ceil(t_lo / delta_time_ - 1e-9));
  int step_hi = static_cast<int>(std::floor(max_time / delta_time_));

  std::set<std::pair<std::size_t, std::size_t>> pairs;
  for (int step = step_lo; step <= step_hi; ++step)
  {
    double t = step * delta_time_;
    if (t >= w_end)
      break;

    pairs.clear();
    collidingPairsAtTime(node, t, link_to_agent, pairs);
    if (!pairs.empty())
    {
      result.valid = true;
      result.a = pairs.begin()->first;
      result.b = pairs.begin()->second;
      result.t = t;
      return result;  // earliest, since we scan time ascending
    }
  }
  return result;
}

bool PBS::firstConflictTime(const PBSNode& node, std::size_t a, std::size_t b,
                            double w_start, double w_end, double& t_out) const
{
  std::unordered_map<std::string, std::size_t> link_to_agent;
  buildLinkToAgent(node, link_to_agent);

  double max_time = getMaxTrajectoryTime(node);
  if (max_time <= 0.0)
    return false;

  std::pair<std::size_t, std::size_t> want(std::min(a, b), std::max(a, b));

  double t_lo = std::max(0.0, w_start);
  int step_lo = static_cast<int>(std::ceil(t_lo / delta_time_ - 1e-9));
  int step_hi = static_cast<int>(std::floor(max_time / delta_time_));

  std::set<std::pair<std::size_t, std::size_t>> pairs;
  for (int step = step_lo; step <= step_hi; ++step)
  {
    double t = step * delta_time_;
    if (t >= w_end)
      break;
    pairs.clear();
    collidingPairsAtTime(node, t, link_to_agent, pairs);
    if (pairs.count(want))
    {
      t_out = t;
      return true;
    }
  }
  return false;
}

// === Priority utilities ===
bool PBS::isBefore(const PBSNode& node, std::size_t before, std::size_t after) const
{
  std::vector<bool> visited(node.agents.size(), false);
  std::vector<std::size_t> stack;
  stack.push_back(before);

  while (!stack.empty())
  {
    std::size_t cur = stack.back();
    stack.pop_back();
    if (cur == after)
      return true;
    if (visited[cur])
      continue;
    visited[cur] = true;
    for (const auto& c : node.constraints)
      if (c.before == cur && !visited[c.after])
        stack.push_back(c.after);
  }
  return false;
}

std::vector<std::size_t> PBS::getHigherPriorityAgents(const PBSNode& node,
                                                      std::size_t agent_idx) const
{
  std::vector<std::size_t> result;
  for (std::size_t i = 0; i < node.agents.size(); ++i)
  {
    if (i == agent_idx)
      continue;
    if (isBefore(node, i, agent_idx))  // i precedes agent_idx => i is higher priority
      result.push_back(i);
  }
  return result;
}

// === Replanning ===
bool PBS::replanAndMakeConsistent(PBSNode& node, std::size_t yield_agent,
                                  double t_seed, double w_start, double w_end)
{
  std::set<std::size_t> work;
  std::unordered_map<std::size_t, double> seed;
  work.insert(yield_agent);
  seed[yield_agent] = t_seed;

  std::size_t budget = node.agents.size() * 5 + 5;
  std::size_t iters = 0;

  while (!work.empty())
  {
    if (++iters > budget)
      return false;  // could not reach consistency within budget

    // Topological pick: an agent with no higher-priority agent still pending,
    // so each agent is replanned only after the agents it must avoid are final.
    std::size_t a = *work.begin();
    for (std::size_t cand : work)
    {
      bool blocked = false;
      for (std::size_t o : work)
        if (o != cand && isBefore(node, o, cand)) { blocked = true; break; }
      if (!blocked) { a = cand; break; }
    }
    work.erase(a);

    double ts = seed.count(a) ? seed[a] : w_start;
    if (!replanAgentSuffix(node, a, ts, w_start, w_end))
      return false;

    // After replanning a, propagate to any ordered partner that now conflicts.
    for (std::size_t b = 0; b < node.agents.size(); ++b)
    {
      if (b == a)
        continue;
      double tc = 0.0;
      if (!firstConflictTime(node, a, b, w_start, w_end, tc))
        continue;

      if (isBefore(node, a, b))        // a higher than b => b must yield
      {
        work.insert(b);
        seed[b] = seed.count(b) ? std::min(seed[b], tc) : tc;
      }
      else if (isBefore(node, b, a))   // b higher than a => a must yield again
      {
        work.insert(a);
        seed[a] = seed.count(a) ? std::min(seed[a], tc) : tc;
      }
      // unordered pair: leave it for solveNode to branch on
    }
  }
  return true;
}

bool PBS::replanAgentSuffix(PBSNode& node, std::size_t agent_idx,
                            double t_seed, double w_start, double w_end)
{
  AgentInfo& agent = node.agents[agent_idx];
  std::vector<std::size_t> higher = getHigherPriorityAgents(node, agent_idx);

  // 1) Freeze the committed prefix and find a *valid* state to restart from.
  double t_freeze = findValidFreezeStart(node, agent_idx, higher, t_seed, w_start);
  if (t_freeze < 0.0)
  {
    ROS_WARN("PBS::replanAgentSuffix: no valid freeze state for '%s'.", agent.name.c_str());
    return false;
  }

  // 2) Determine which goal the agent is heading to, and re-plan the *entire*
  //    remaining goal sequence (no truncation).
  int g0 = nextGoalIndexAtTime(agent, t_freeze, 0.05);
  if (g0 < 0 || agent.goals.empty())
  {
    ROS_WARN("PBS::replanAgentSuffix: no remaining goal for '%s'.", agent.name.c_str());
    return false;
  }

  std::vector<double> vmax = jointVelocityLimits(agent.move_group);
  std::vector<double> q_cur = getAgentPositionsAtTime(agent, t_freeze);
  double t_cur = t_freeze;
  std::vector<AgentInfo::TrajectoryPoint> suffix;
  bool first = true;

  for (int g = g0; g < static_cast<int>(agent.goals.size()); ++g)
  {
    if (agent.goals[g].type != AgentInfo::Goal::JOINT)
      continue;  // pose goals must be converted to joint goals before PBS

    TimeAwareRRT rrt(robot_model_, planning_scene_, node.agents, agent_idx,
                     delta_time_, horizon_, max_rrt_nodes_);
    if (!vmax.empty())
      rrt.setMaxJointVelocities(vmax);
    rrt.setWaitBias(0.2);

    std::vector<AgentInfo::TrajectoryPoint> seg;
    if (!rrt.plan(t_cur, q_cur, agent.goals[g].joint_values, higher, seg))
    {
      ROS_WARN("PBS::replanAgentSuffix: RRT failed for '%s' toward goal %d.",
               agent.name.c_str(), g);
      return false;
    }

    // Skip the duplicated junction point between consecutive segments.
    for (std::size_t k = (first ? 0 : 1); k < seg.size(); ++k)
      suffix.push_back(seg[k]);
    first = false;
    t_cur = seg.back().time;
    q_cur = seg.back().positions;
  }

  if (suffix.empty())
    return false;

  // 3) Splice: committed prefix (< t_freeze) + freshly planned suffix.
  std::vector<AgentInfo::TrajectoryPoint> merged;
  merged.reserve(agent.trajectory.size() + suffix.size());
  for (const auto& pt : agent.trajectory)
  {
    if (pt.time < t_freeze - 1e-6)
      merged.push_back(pt);
    else
      break;
  }
  for (auto& pt : suffix)
    merged.push_back(pt);
  agent.trajectory.swap(merged);

  ROS_INFO("PBS::replanAgentSuffix: '%s' replanned from t=%.2f through goals [%d..%d] (%zu pts).",
           agent.name.c_str(), t_freeze, g0,
           static_cast<int>(agent.goals.size()) - 1, agent.trajectory.size());
  return true;
}

double PBS::findValidFreezeStart(const PBSNode& node, std::size_t agent_idx,
                                 const std::vector<std::size_t>& higher,
                                 double t_seed, double w_start) const
{
  double t = std::floor(t_seed / delta_time_ + 1e-9) * delta_time_;
  double t_min = std::max(0.0, w_start);

  for (; t >= t_min - 1e-9; t -= delta_time_)
  {
    double tq = std::max(t, 0.0);
    std::vector<double> q = getAgentPositionsAtTime(node.agents[agent_idx], tq);
    if (stateValidVsHigher(node, agent_idx, q, tq, higher))
      return tq;
  }
  return -1.0;
}

int PBS::nextGoalIndexAtTime(const AgentInfo& agent, double t, double goal_tol) const
{
  if (agent.goals.empty())
    return -1;

  // A goal g is "reached by time t" if some prefix waypoint (time <= t) is within
  // goal_tol of it. Goals are sequential, so stop at the first unreached goal.
  int last_reached = -1;
  for (int g = 0; g < static_cast<int>(agent.goals.size()); ++g)
  {
    const auto& goal = agent.goals[g];
    if (goal.type != AgentInfo::Goal::JOINT)
      break;  // cannot reason about a non-joint goal here

    bool reached = false;
    for (const auto& pt : agent.trajectory)
    {
      if (pt.time > t + 1e-6)
        break;
      if (pt.positions.size() != goal.joint_values.size())
        continue;
      double d2 = 0.0;
      for (std::size_t j = 0; j < pt.positions.size(); ++j)
      {
        double diff = pt.positions[j] - goal.joint_values[j];
        d2 += diff * diff;
      }
      if (std::sqrt(d2) <= goal_tol) { reached = true; break; }
    }
    if (reached)
      last_reached = g;
    else
      break;
  }

  int next = last_reached + 1;
  if (next >= static_cast<int>(agent.goals.size()))
    next = static_cast<int>(agent.goals.size()) - 1;  // already at/after last goal
  return next;
}

// Validity of q for agent_idx against higher-priority agents (+ world + self).
// Lower/equal-priority agents are intentionally ignored: they must plan around
// this agent, not vice versa. This mirrors TimeAwareRRT::isStateValid so the
// freeze seed and the RRT agree on what "valid" means.
bool PBS::stateValidVsHigher(const PBSNode& node, std::size_t agent_idx,
                             const std::vector<double>& q, double t,
                             const std::vector<std::size_t>& higher) const
{
  const auto* jmg = robot_model_->getJointModelGroup(node.agents[agent_idx].move_group);
  if (!jmg || q.size() != jmg->getVariableCount())
    return false;

  moveit::core::RobotState& rs = *work_state_;
  rs.setToDefaultValues();
  rs.setJointGroupPositions(jmg, q);

  for (std::size_t idx : higher)
  {
    if (idx >= node.agents.size())
      continue;
    const auto* jo = robot_model_->getJointModelGroup(node.agents[idx].move_group);
    if (!jo)
      continue;
    std::vector<double> qo = getAgentPositionsAtTime(node.agents[idx], t);
    if (qo.size() == jo->getVariableCount())
      rs.setJointGroupPositions(jo, qo);
  }
  rs.update();

  collision_detection::CollisionRequest creq;
  collision_detection::CollisionResult  cres;
  planning_scene_->checkCollision(creq, cres, rs);
  return !cres.collision;
}

std::vector<double> PBS::jointVelocityLimits(const std::string& group) const
{
  std::vector<double> vmax;
  const auto* jmg = robot_model_->getJointModelGroup(group);
  if (!jmg)
    return vmax;
  const std::vector<std::string>& names = jmg->getVariableNames();
  vmax.resize(names.size(), 1.0);
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    const auto& vb = robot_model_->getVariableBounds(names[i]);
    if (vb.velocity_bounded_ && vb.max_velocity_ > 0.0)
      vmax[i] = vb.max_velocity_ * vel_scale_;
  }
  return vmax;
}

// === Low-level utilities ===
std::vector<double> PBS::getAgentPositionsAtTime(const AgentInfo& agent, double t_query) const
{
  const auto& traj = agent.trajectory;
  if (traj.empty())
    return {};

  if (t_query <= traj.front().time)
    return traj.front().positions;
  if (t_query >= traj.back().time)
    return traj.back().positions;

  for (std::size_t i = 0; i + 1 < traj.size(); ++i)
  {
    double t0 = traj[i].time, t1 = traj[i + 1].time;
    if (t_query >= t0 && t_query <= t1)
    {
      double a = (t1 > t0) ? (t_query - t0) / (t1 - t0) : 0.0;
      const auto& q0 = traj[i].positions;
      const auto& q1 = traj[i + 1].positions;
      std::vector<double> q(q0.size());
      for (std::size_t j = 0; j < q0.size(); ++j)
        q[j] = (1.0 - a) * q0[j] + a * q1[j];
      return q;
    }
  }
  return traj.back().positions;
}

double PBS::getMaxTrajectoryTime(const PBSNode& node) const
{
  double max_t = 0.0;
  for (const auto& ag : node.agents)
    if (!ag.trajectory.empty() && ag.trajectory.back().time > max_t)
      max_t = ag.trajectory.back().time;
  return max_t;
}

}  // namespace rhcr_mapf_planners
