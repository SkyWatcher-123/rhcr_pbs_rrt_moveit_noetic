#include "rhcr_mapf_planners/time_aware_rrt.h"

#include <ros/ros.h>
#include <random>
#include <limits>
#include <cmath>
#include <algorithm>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_state/robot_state.h>

namespace rhcr_mapf_planners
{

TimeAwareRRT::TimeAwareRRT(const moveit::core::RobotModelConstPtr& robot_model,
                           const planning_scene::PlanningScenePtr& planning_scene,
                           const std::vector<AgentInfo>& agents,
                           std::size_t this_agent_idx,
                           double delta_time,
                           double horizon,
                           std::size_t max_nodes)
  : robot_model_(robot_model)
  , planning_scene_(planning_scene)
  , agents_(agents)
  , this_agent_idx_(this_agent_idx)
  , delta_time_(delta_time)
  , horizon_(horizon)
  , max_nodes_(max_nodes)
  , v_max_(1.0)  // default 1 rad/s
{
  work_state_.reset(new moveit::core::RobotState(robot_model_));
  work_state_->setToDefaultValues();
}

void TimeAwareRRT::setMaxJointVelocity(double v_max)
{
  v_max_ = v_max;
  v_max_per_joint_.clear();
}

void TimeAwareRRT::setMaxJointVelocities(const std::vector<double>& v_max_per_joint)
{
  v_max_per_joint_ = v_max_per_joint;
}

void TimeAwareRRT::setWaitBias(double wait_bias)
{
  wait_bias_ = std::max(0.0, std::min(1.0, wait_bias));
}

std::vector<double> TimeAwareRRT::sampleRandomConfiguration(const moveit::core::JointModelGroup* jmg) const
{
  std::vector<double> q(jmg->getVariableCount());
  const std::vector<std::string>& var_names = jmg->getVariableNames();

  static thread_local std::mt19937 rng(std::random_device{}());

  for (std::size_t i = 0; i < var_names.size(); ++i)
  {
    const auto& bounds = robot_model_->getVariableBounds(var_names[i]);
    std::uniform_real_distribution<double> dist(bounds.min_position_, bounds.max_position_);
    q[i] = dist(rng);
  }

  return q;
}

std::vector<double> TimeAwareRRT::sampleBiasedConfiguration(const moveit::core::JointModelGroup* jmg,
                                                            const std::vector<double>& q_goal,
                                                            double goal_bias) const
{
  static thread_local std::mt19937 rng(std::random_device{}());
  std::uniform_real_distribution<double> uni01(0.0, 1.0);

  if (uni01(rng) < goal_bias)
  {
    // Sample in a small ball around the goal
    std::normal_distribution<double> noise(0.0, 0.1);  // 0.1 rad std-dev
    std::vector<double> q = q_goal;
    for (double& v : q)
      v += noise(rng);
    return q;
  }
  else
  {
    return sampleRandomConfiguration(jmg);
  }
}

int TimeAwareRRT::findNearestNode(const std::vector<Node>& tree,
                                  const std::vector<double>& q_sample) const
{
  int best_idx = -1;
  double best_dist2 = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < tree.size(); ++i)
  {
    double d2 = distanceSquared(tree[i].q, q_sample);
    // Break ties toward larger t so chained WAIT edges advance forward in time.
    if (d2 < best_dist2 - 1e-12 ||
        (std::fabs(d2 - best_dist2) <= 1e-12 &&
         (best_idx < 0 || tree[i].t > tree[best_idx].t)))
    {
      best_dist2 = d2;
      best_idx = static_cast<int>(i);
    }
  }
  return best_idx;
}

std::vector<double> TimeAwareRRT::steer(const std::vector<double>& q_from,
                                        const std::vector<double>& q_to) const
{
  std::vector<double> q_new = q_from;
  if (q_from.size() != q_to.size())
    return q_new;

  // Limit motion so every joint respects its own velocity bound over one step,
  // while preserving the straight-line direction toward q_to.
  double scale = 1.0;
  for (std::size_t i = 0; i < q_from.size(); ++i)
  {
    double diff   = q_to[i] - q_from[i];
    double vmax_i = (i < v_max_per_joint_.size()) ? v_max_per_joint_[i] : v_max_;
    double step_i = vmax_i * delta_time_;
    double ad     = std::fabs(diff);
    if (ad > step_i && ad > 1e-12)
      scale = std::min(scale, step_i / ad);
  }

  for (std::size_t i = 0; i < q_from.size(); ++i)
    q_new[i] = q_from[i] + scale * (q_to[i] - q_from[i]);

  return q_new;
}

bool TimeAwareRRT::isStateValid(const std::vector<double>& q,
                                double t,
                                const std::vector<std::size_t>& higher_priority_indices,
                                const moveit::core::JointModelGroup* jmg) const
{
  if (!jmg)
    return false;

  // Use a private scratch state instead of mutating the shared planning scene.
  moveit::core::RobotState& rs = *work_state_;
  rs.setToDefaultValues();

  // Set this agent
  rs.setJointGroupPositions(jmg, q);

  // Set all higher-priority agents (dynamic obstacles)
  for (std::size_t idx : higher_priority_indices)
  {
    if (idx >= agents_.size())
      continue;
    const AgentInfo& ag = agents_[idx];
    const auto* jmg_other = robot_model_->getJointModelGroup(ag.move_group);
    if (!jmg_other)
      continue;
    const auto& q_other = getAgentPositionsAtTime(idx, t);
    if (q_other.size() == jmg_other->getVariableCount())
      rs.setJointGroupPositions(jmg_other, q_other);
  }

  rs.update();

  collision_detection::CollisionRequest creq;
  collision_detection::CollisionResult  cres;
  creq.contacts     = false;
  creq.max_contacts = 1;

  planning_scene_->checkCollision(creq, cres, rs);
  return !cres.collision;
}

std::vector<double> TimeAwareRRT::getAgentPositionsAtTime(std::size_t agent_index,
                                                          double t_query) const
{
  const AgentInfo& agent = agents_[agent_index];
  const auto& traj = agent.trajectory;

  if (traj.empty())
    return {};

  if (t_query <= traj.front().time)
    return traj.front().positions;
  if (t_query >= traj.back().time)
    return traj.back().positions;

  // Linear interpolation between the two bracketing waypoints.
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

double TimeAwareRRT::distanceSquared(const std::vector<double>& a,
                                     const std::vector<double>& b)
{
  if (a.size() != b.size())
    return std::numeric_limits<double>::infinity();
  double d2 = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i)
  {
    double diff = a[i] - b[i];
    d2 += diff * diff;
  }
  return d2;
}

bool TimeAwareRRT::plan(double t_start,
                        const std::vector<double>& q_start,
                        const std::vector<double>& q_goal,
                        const std::vector<std::size_t>& higher_priority_indices,
                        std::vector<AgentInfo::TrajectoryPoint>& out_traj)
{
  const AgentInfo& this_agent = agents_[this_agent_idx_];
  const auto* jmg = robot_model_->getJointModelGroup(this_agent.move_group);
  if (!jmg)
  {
    ROS_ERROR("TimeAwareRRT: No JointModelGroup '%s' for agent '%s'.",
              this_agent.move_group.c_str(), this_agent.name.c_str());
    return false;
  }

  if (q_start.size() != jmg->getVariableCount() ||
      q_goal.size()  != jmg->getVariableCount())
  {
    ROS_ERROR("TimeAwareRRT: dimension mismatch for agent '%s'.", this_agent.name.c_str());
    return false;
  }

  // Tree initialization
  std::vector<Node> tree;
  tree.reserve(max_nodes_);

  Node root;
  root.q = q_start;
  root.t = t_start;
  root.parent = -1;
  tree.push_back(root);

  // If the start is already invalid (with constraints), we fail early.
  if (!isStateValid(q_start, t_start, higher_priority_indices, jmg))
  {
    ROS_WARN("TimeAwareRRT: start state is invalid for agent '%s'.", this_agent.name.c_str());
    return false;
  }

  static thread_local std::mt19937 rng(std::random_device{}());
  std::uniform_real_distribution<double> uni01(0.0, 1.0);

  const double goal_tol = 0.02;     // joint-space tolerance to goal
  const double goal_bias = 0.1;     // 10% samples near goal
  const double max_time = t_start + horizon_;

  int goal_node_idx = -1;

  // Main RRT loop
  for (std::size_t iter = 0; iter < max_nodes_; ++iter)
  {
    // 1) Choose between a WAIT expansion (hold configuration, advance time) and a
    //    normal sample-and-steer expansion. Waiting lets a lower-priority agent
    //    yield by pausing in place until a higher-priority agent has cleared.
    bool do_wait = (uni01(rng) < wait_bias_);

    int nn_idx;
    std::vector<double> q_new;
    if (do_wait)
    {
      // Wait at the node currently closest to the goal; the larger-t tie-break in
      // findNearestNode lets successive waits chain forward in time.
      nn_idx = findNearestNode(tree, q_goal);
      if (nn_idx < 0)
        continue;
      q_new = tree[nn_idx].q;  // hold configuration
    }
    else
    {
      std::vector<double> q_sample = sampleBiasedConfiguration(jmg, q_goal, goal_bias);
      nn_idx = findNearestNode(tree, q_sample);
      if (nn_idx < 0)
        continue;
      q_new = steer(tree[nn_idx].q, q_sample);
    }

    // 2) Enforce forward time stepping: t_new = t_parent + delta_time_
    double t_new = tree[nn_idx].t + delta_time_;
    if (t_new > max_time)
      continue;  // out of horizon

    // 3) Check state validity with dynamic constraints
    if (!isStateValid(q_new, t_new, higher_priority_indices, jmg))
      continue;

    // 4) Add new node
    Node child;
    child.q = q_new;
    child.t = t_new;
    child.parent = nn_idx;
    tree.push_back(child);
    int child_idx = static_cast<int>(tree.size()) - 1;

    // 5) Check goal condition (joint-space distance only)
    double d2_goal = distanceSquared(q_new, q_goal);
    if (std::sqrt(d2_goal) <= goal_tol)
    {
      goal_node_idx = child_idx;
      ROS_INFO("TimeAwareRRT: goal reached for agent '%s' in %zu iterations (nodes=%zu).",
               this_agent.name.c_str(), iter + 1, tree.size());
      break;
    }
  }

  if (goal_node_idx < 0)
  {
    ROS_WARN("TimeAwareRRT: failed to reach goal for agent '%s' (nodes=%zu).",
             this_agent.name.c_str(), tree.size());
    return false;
  }

  // Reconstruct path (q, t) by following parent links
  std::vector<Node> path_nodes;
  for (int idx = goal_node_idx; idx >= 0; idx = tree[idx].parent)
    path_nodes.push_back(tree[idx]);

  // path_nodes is from goal to start; reverse it
  std::reverse(path_nodes.begin(), path_nodes.end());

  // Convert to trajectory points
  out_traj.clear();
  out_traj.reserve(path_nodes.size());

  for (const Node& n : path_nodes)
  {
    AgentInfo::TrajectoryPoint pt;
    pt.time = n.t;
    pt.positions = n.q;
    pt.velocities.assign(n.q.size(), 0.0);  // can be filled later if needed
    out_traj.push_back(pt);
  }

  // Optional: compute simple finite-difference velocities
  if (out_traj.size() >= 2 && delta_time_ > 0.0)
  {
    for (std::size_t i = 1; i < out_traj.size(); ++i)
    {
      for (std::size_t j = 0; j < out_traj[i].positions.size(); ++j)
      {
        double dq = out_traj[i].positions[j] - out_traj[i - 1].positions[j];
        out_traj[i].velocities[j] = dq / delta_time_;
      }
    }
  }

  return true;
}

}  // namespace rhcr_mapf_planners
