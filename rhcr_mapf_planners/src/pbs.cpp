#include "rhcr_mapf_planners/pbs.h"

#include <ros/ros.h>
#include <unordered_map>
#include <cmath>

#include <moveit/collision_detection/collision_common.h>

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
}

// === Public entry point ===
bool PBS::resolveConflicts(std::vector<AgentInfo>& agents)
{
  PBSNode root;
  root.agents = agents;
  root.constraints.clear();

  std::size_t max_depth = agents.size() * 2;  // heuristic limit

  bool ok = solveNode(root, 0, max_depth);
  if (ok)
  {
    agents = root.agents;
  }
  else
  {
    ROS_WARN("PBS::resolveConflicts: failed to find conflict-free solution.");
  }
  return ok;
}

// === Core recursive solver ===
bool PBS::solveNode(PBSNode& node, std::size_t depth, std::size_t max_depth)
{
  if (depth > max_depth)
  {
    ROS_WARN("PBS::solveNode: exceeded max depth %zu.", max_depth);
    return false;
  }

  Conflict c = findEarliestConflict(node);
  if (!c.valid)
  {
    // No conflicts: success
    return true;
  }

  ROS_INFO("PBS: conflict at t=%.3f between %s and %s (depth=%zu).",
           c.t,
           node.agents[c.a].name.c_str(),
           node.agents[c.b].name.c_str(),
           depth);

  // Branch 1: a ≺ b (a before b, b yields)
  {
    PBSNode child = node;
    child.constraints.push_back(PriorityConstraint{c.a, c.b});
    double freeze_margin = delta_time_;  // extended interval around conflict
    bool replanned = localReplan(child, c.b, c.t, freeze_margin, 0.05);
    if (replanned)
    {
      if (solveNode(child, depth + 1, max_depth))
      {
        node = child;
        return true;
      }
    }
  }

  // Branch 2: b ≺ a
  {
    PBSNode child = node;
    child.constraints.push_back(PriorityConstraint{c.b, c.a});
    double freeze_margin = delta_time_;
    bool replanned = localReplan(child, c.a, c.t, freeze_margin, 0.05);
    if (replanned)
    {
      if (solveNode(child, depth + 1, max_depth))
      {
        node = child;
        return true;
      }
    }
  }

  return false;
}

// === Conflict detection (earliest in time) ===
PBS::Conflict PBS::findEarliestConflict(const PBSNode& node) const
{
  Conflict result;
  result.valid = false;
  result.a = result.b = 0;
  result.t = 0.0;

  if (node.agents.empty())
    return result;

  // Precompute link->agent mapping for collision contacts
  std::unordered_map<std::string, std::size_t> link_to_agent;
  for (std::size_t i = 0; i < node.agents.size(); ++i)
  {
    const auto* jmg = robot_model_->getJointModelGroup(node.agents[i].move_group);
    if (!jmg)
      continue;
    for (const auto& link : jmg->getLinkModelNames())
      link_to_agent[link] = i;
  }

  planning_scene::PlanningScene local_scene(robot_model_);
  double max_time = getMaxTrajectoryTime(node);
  if (max_time <= 0.0)
    return result;

  int max_step = static_cast<int>(std::floor(max_time / delta_time_));

  double earliest_t = std::numeric_limits<double>::infinity();
  std::size_t ca = 0, cb = 0;

  for (int step = 0; step <= max_step; ++step)
  {
    double t = step * delta_time_;

    moveit::core::RobotState& rs = local_scene.getCurrentStateNonConst();
    rs.setToDefaultValues();

    // set all agents at time t
    for (std::size_t ai = 0; ai < node.agents.size(); ++ai)
    {
      const AgentInfo& ag = node.agents[ai];
      const auto* jmg = robot_model_->getJointModelGroup(ag.move_group);
      if (!jmg)
        continue;
      const auto& q = getAgentPositionsAtTime(ag, t);
      if (q.size() == jmg->getVariableCount())
        rs.setJointGroupPositions(jmg, q);
    }
    rs.update();

    collision_detection::CollisionRequest creq;
    collision_detection::CollisionResult  cres;
    creq.contacts     = true;
    creq.max_contacts = 1;

    local_scene.checkCollision(creq, cres, rs);
    if (!cres.collision)
      continue;

    auto ct = cres.contacts.begin();
    if (ct == cres.contacts.end())
      continue;

    const std::string& link1 = ct->first.first;
    const std::string& link2 = ct->first.second;

    auto it1 = link_to_agent.find(link1);
    auto it2 = link_to_agent.find(link2);
    if (it1 == link_to_agent.end() || it2 == link_to_agent.end())
      continue;

    std::size_t a = it1->second;
    std::size_t b = it2->second;
    if (a == b)
      continue;  // self-collision; ignore in PBS

    if (t < earliest_t)
    {
      earliest_t = t;
      ca = a;
      cb = b;
      result.valid = true;
    }
  }

  if (result.valid)
  {
    result.a = ca;
    result.b = cb;
    result.t = earliest_t;
  }
  return result;
}

// === Priority utilities ===

bool PBS::isBefore(const PBSNode& node, std::size_t before, std::size_t after) const
{
  // Simple DFS over constraints to see if before ≺ after is implied
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
    {
      if (c.before == cur && !visited[c.after])
        stack.push_back(c.after);
    }
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
    // if i ≺ agent_idx according to constraints, then i is higher priority
    if (isBefore(node, i, agent_idx))
      result.push_back(i);
  }
  return result;
}

// === Goal-segment reasoning ===

std::pair<int, double> PBS::findActiveGoalSegment(const AgentInfo& agent,
                                                  double t_conflict,
                                                  double goal_tol) const
{
  int active_g = -1;
  double t_last_goal = 0.0;

  if (agent.goals.empty() || agent.trajectory.empty())
    return {active_g, t_last_goal};

  // Precompute first time each goal is reached along this trajectory
  std::vector<double> goal_times(agent.goals.size(), std::numeric_limits<double>::infinity());

  for (std::size_t g = 0; g < agent.goals.size(); ++g)
  {
    const auto& goal = agent.goals[g];
    if (goal.type != AgentInfo::Goal::JOINT)
      continue;

    for (const auto& pt : agent.trajectory)
    {
      if (pt.positions.size() != goal.joint_values.size())
        continue;
      double d2 = 0.0;
      for (std::size_t j = 0; j < pt.positions.size(); ++j)
      {
        double diff = pt.positions[j] - goal.joint_values[j];
        d2 += diff * diff;
      }
      if (std::sqrt(d2) <= goal_tol)
      {
        goal_times[g] = pt.time;
        break;
      }
    }
  }

  // Find largest g such that goal_times[g] <= t_conflict
  for (int g = static_cast<int>(agent.goals.size()) - 1; g >= 0; --g)
  {
    if (goal_times[g] < std::numeric_limits<double>::infinity() &&
        goal_times[g] <= t_conflict + 1e-6)
    {
      active_g = g;
      t_last_goal = goal_times[g];
      break;
    }
  }

  return {active_g, t_last_goal};
}

// === Local replan with TimeAwareRRT ===

bool PBS::localReplan(PBSNode& node,
                      std::size_t agent_idx,
                      double t_conflict,
                      double t_freeze_margin,
                      double goal_tol)
{
  AgentInfo& agent = node.agents[agent_idx];

  ROS_INFO("PBS::localReplan: agent '%s' at t=%.3f.",
           agent.name.c_str(), t_conflict);

  // 1) Determine active goal segment; this yields the "next goal" index.
  auto seg = findActiveGoalSegment(agent, t_conflict, goal_tol);
  int g_prev = seg.first;
  double t_last_goal = seg.second;

  int g_target = -1;
  if (g_prev < 0)
  {
    // collision before reaching goal[0]: segment start->goal[0]
    if (!agent.goals.empty())
      g_target = 0;
  }
  else if (g_prev < static_cast<int>(agent.goals.size()) - 1)
  {
    // segment goal[g_prev] -> goal[g_prev+1]
    g_target = g_prev + 1;
  }
  else
  {
    // collision after last goal; replan toward last goal again
    g_target = static_cast<int>(agent.goals.size()) - 1;
  }

  if (g_target < 0 || g_target >= static_cast<int>(agent.goals.size()))
  {
    ROS_WARN("PBS::localReplan: no valid target goal for '%s'.", agent.name.c_str());
    return false;
  }

  const auto& goal = agent.goals[g_target];
  if (goal.type != AgentInfo::Goal::JOINT)
  {
    ROS_WARN("PBS::localReplan: goal %d for '%s' is not JOINT.", g_target, agent.name.c_str());
    return false;
  }

  // 2) Extended freeze: we freeze everything up to t_freeze_end
  //    (SIPP-like stronger commitment). We choose:
  //      t_freeze_end = max(t_last_goal, t_conflict - t_freeze_margin)
  double t_freeze_end = t_conflict - t_freeze_margin;
  if (t_last_goal > t_freeze_end)
    t_freeze_end = t_last_goal;

  if (t_freeze_end < 0.0)
    t_freeze_end = 0.0;

  // Replan segment will start at t_replan >= t_freeze_end and <= t_conflict
  double t_replan = t_conflict;
  if (t_replan < t_freeze_end)
    t_replan = t_freeze_end;

  // Get starting configuration for replan
  const std::vector<double>& q_start = getAgentPositionsAtTime(agent, t_replan);
  const std::vector<double>& q_goal  = goal.joint_values;

  // 3) Build list of higher-priority agents for this agent
  std::vector<std::size_t> higher = getHigherPriorityAgents(node, agent_idx);

  // 4) Run time-aware RRT from (q_start, t_replan) to q_goal
  TimeAwareRRT rrt(robot_model_, planning_scene_, node.agents, agent_idx,
                   delta_time_, horizon_, max_rrt_nodes_);
  rrt.setMaxJointVelocity(1.0);  // you can tune this

  std::vector<AgentInfo::TrajectoryPoint> new_segment;
  bool ok = rrt.plan(t_replan, q_start, q_goal, higher, new_segment);
  if (!ok)
  {
    ROS_WARN("PBS::localReplan: time-aware RRT failed for '%s'.", agent.name.c_str());
    return false;
  }

  // 5) Splice new segment into agent.trajectory:
  //    - keep points with time < t_replan
  //    - append new_segment (which starts at t_replan)
  std::vector<AgentInfo::TrajectoryPoint> new_traj;
  new_traj.reserve(agent.trajectory.size() + new_segment.size());

  for (const auto& pt : agent.trajectory)
  {
    if (pt.time < t_replan - 1e-6)
      new_traj.push_back(pt);
    else
      break;
  }
  for (auto& pt : new_segment)
  {
    new_traj.push_back(pt);
  }

  agent.trajectory.swap(new_traj);

  ROS_INFO("PBS::localReplan: agent '%s' replanned to goal %d with %zu points.",
           agent.name.c_str(), g_target, agent.trajectory.size());

  return true;
}

// === Low-level utilities ===

const std::vector<double>& PBS::getAgentPositionsAtTime(const AgentInfo& agent,
                                                        double t_query) const
{
  const auto& traj = agent.trajectory;

  if (traj.empty())
  {
    static const std::vector<double> dummy;
    return dummy;
  }

  if (t_query <= traj.front().time + 1e-6)
    return traj.front().positions;
  if (t_query >= traj.back().time - 1e-6)
    return traj.back().positions;

  for (std::size_t i = 0; i < traj.size(); ++i)
  {
    if (traj[i].time >= t_query - 1e-6)
      return traj[i].positions;
  }
  return traj.back().positions;
}

double PBS::getMaxTrajectoryTime(const PBSNode& node) const
{
  double max_t = 0.0;
  for (const auto& ag : node.agents)
  {
    if (!ag.trajectory.empty() && ag.trajectory.back().time > max_t)
      max_t = ag.trajectory.back().time;
  }
  return max_t;
}

}  // namespace rhcr_mapf_planners
