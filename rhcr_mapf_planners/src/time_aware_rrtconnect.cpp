#include "rhcr_mapf_planners/time_aware_rrtconnect.h"

#include <ros/ros.h>

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/PlannerTerminationCondition.h>


#include <algorithm>
#include <cmath>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace rhcr_mapf_planners;

// ---------------- Goal region in joint space (ignores t) ----------------
class JointGoalRegion : public ob::GoalRegion
{
public:
  JointGoalRegion(const ob::SpaceInformationPtr& si,
                  std::size_t dof,
                  double tol)
    : ob::GoalRegion(si), dof_(dof), tol_(tol)
  {
    setThreshold(tol_);
  }

  void setGoal(const std::vector<double>& g)
  {
    goal_ = g;
  }

  double distanceGoal(const ob::State* st) const override
  {
    const auto* compound = st->as<ob::CompoundState>();
    const auto* q = compound->as<ob::RealVectorStateSpace::StateType>(0); // joint subspace
    double d2 = 0.0;
    for (std::size_t i = 0; i < dof_; ++i)
    {
      double diff = (*q)[i] - goal_[i];
      d2 += diff * diff;
    }
    return std::sqrt(d2);
  }

private:
  std::size_t dof_;
  double tol_;
  std::vector<double> goal_;
};

// ------------- StateValidityChecker in [q, t] using PlanningScene -------------
class TimeAwareValidityChecker : public ob::StateValidityChecker
{
public:
  TimeAwareValidityChecker(const ob::SpaceInformationPtr& si,
                           const moveit::core::RobotModelConstPtr& robot_model,
                           const planning_scene::PlanningScenePtr& planning_scene,
                           const std::vector<AgentInfo>& agents,
                           std::size_t this_agent_idx,
                           double delta_time)
    : ob::StateValidityChecker(si)
    , robot_model_(robot_model)
    , planning_scene_(planning_scene)
    , agents_(agents)
    , this_agent_idx_(this_agent_idx)
    , delta_time_(delta_time)
  {
  }

  bool isValid(const ob::State* st) const override
  {
    const auto* compound = st->as<ob::CompoundState>();
    const auto* q = compound->as<ob::RealVectorStateSpace::StateType>(0);
    const auto* t_state = compound->as<ob::RealVectorStateSpace::StateType>(1);
    double t = (*t_state)[0];

    moveit::core::RobotState& rs = planning_scene_->getCurrentStateNonConst();
    rs.setToDefaultValues();

    // Set this agent's joints from q
    const AgentInfo& this_agent = agents_[this_agent_idx_];
    const auto* this_jmg = robot_model_->getJointModelGroup(this_agent.move_group);
    if (!this_jmg)
      return false;

    std::vector<double> q_this(this_jmg->getVariableCount());
    for (std::size_t i = 0; i < q_this.size(); ++i)
      q_this[i] = (*q)[i];
    rs.setJointGroupPositions(this_jmg, q_this);

    // Set only *higher-priority* agents (lower index) as moving obstacles at time t.
    // Lower-priority agents are ignored here; they will adapt later via PBS.
    for (std::size_t i = 0; i < agents_.size(); ++i)
    {
      if (i == this_agent_idx_)
        continue;     // skip self

      if (i > this_agent_idx_)
        continue;     // skip lower-priority agents

      const AgentInfo& ag = agents_[i];
      const auto* jmg = robot_model_->getJointModelGroup(ag.move_group);
      if (!jmg)
        continue;

      const auto& q_other = getAgentPositionsAtTime(i, t);
      if (q_other.size() == jmg->getVariableCount())
        rs.setJointGroupPositions(jmg, q_other);
    }

    rs.update();

    collision_detection::CollisionRequest creq;
    collision_detection::CollisionResult  cres;
    creq.contacts     = false;
    creq.max_contacts = 1;
    planning_scene_->checkCollision(creq, cres, rs);

    return !cres.collision; // valid if no collision
  }

private:
  const std::vector<double>& getAgentPositionsAtTime(std::size_t agent_index,
                                                     double t_query) const
  {
    const AgentInfo& agent = agents_[agent_index];
    const auto& traj = agent.trajectory;

    if (traj.empty())
      return traj.back().positions; // degenerate

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

  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  const std::vector<AgentInfo>& agents_;
  std::size_t this_agent_idx_;
  double delta_time_;
};

// ---------------- TimeAwareRRTConnectPlanner implementation ----------------
TimeAwareRRTConnectPlanner::TimeAwareRRTConnectPlanner(
    const moveit::core::RobotModelConstPtr& robot_model,
    const planning_scene::PlanningScenePtr& planning_scene,
    const std::vector<AgentInfo>& agents,
    std::size_t this_agent_idx,
    double delta_time)
  : robot_model_(robot_model)
  , planning_scene_(planning_scene)
  , agents_(agents)
  , this_agent_idx_(this_agent_idx)
  , delta_time_(delta_time)
{
}

const std::vector<double>& TimeAwareRRTConnectPlanner::getAgentPositionsAtTime(
    std::size_t agent_index, double t_query) const
{
  const AgentInfo& agent = agents_[agent_index];
  const auto& traj = agent.trajectory;

  if (traj.empty())
    return traj.back().positions;

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





bool TimeAwareRRTConnectPlanner::plan(double start_time,
                                      const std::vector<double>& start_positions,
                                      const std::vector<double>& goal_positions,
                                      std::vector<AgentInfo::TrajectoryPoint>& out_traj)
{
  if (!robot_model_ || !planning_scene_)
  {
    ROS_ERROR("TimeAwareRRTConnectPlanner: null robot_model or planning_scene.");
    return false;
  }

  const AgentInfo& this_agent = agents_[this_agent_idx_];
  const auto* jmg = robot_model_->getJointModelGroup(this_agent.move_group);
  if (!jmg)
  {
    ROS_ERROR("TimeAwareRRTConnectPlanner: no JointModelGroup '%s'.",
              this_agent.move_group.c_str());
    return false;
  }

  std::size_t dof = jmg->getVariableCount();
  if (start_positions.size() != dof || goal_positions.size() != dof)
  {
    ROS_ERROR("TimeAwareRRTConnectPlanner: start/goal dim mismatch (expected %zu).", dof);
    return false;
  }

  // ---- 1) Create [q, t] compound space ----
  auto q_space = std::make_shared<ob::RealVectorStateSpace>(dof);
  auto t_space = std::make_shared<ob::RealVectorStateSpace>(1);

  // joint bounds from model
  ob::RealVectorBounds q_bounds(dof);
  const auto& var_names = jmg->getVariableNames();
  for (std::size_t i = 0; i < dof; ++i)
  {
    const auto& vb = robot_model_->getVariableBounds(var_names[i]);
    q_bounds.setLow(i, vb.min_position_);
    q_bounds.setHigh(i, vb.max_position_);
  }
  q_space->setBounds(q_bounds);

  // time bounds: simple finite horizon
  double horizon = 20.0;
  ob::RealVectorBounds t_bounds(1);
  t_bounds.setLow(0, start_time);
  t_bounds.setHigh(0, start_time + horizon);
  t_space->setBounds(t_bounds);

  auto space = std::make_shared<ob::CompoundStateSpace>();
  space->addSubspace(q_space, 1.0);
  space->addSubspace(t_space, 1.0);
  space->lock();

  auto si = std::make_shared<ob::SpaceInformation>(space);
  auto svc = std::make_shared<TimeAwareValidityChecker>(
      si, robot_model_, planning_scene_, agents_, this_agent_idx_, delta_time_);
  si->setStateValidityChecker(svc);
  si->setup();

  // ---- 2) Start state ----
  ob::ScopedState<> start(space);
  {
    ob::State* s_start = start.get();
    auto* compound_start = s_start->as<ob::CompoundState>();

    auto* q = compound_start->as<ob::RealVectorStateSpace::StateType>(0);
    auto* t = compound_start->as<ob::RealVectorStateSpace::StateType>(1);

    for (std::size_t i = 0; i < dof; ++i)
      (*q)[i] = start_positions[i];
    (*t)[0] = start_time;
  }


  // ---- 3) Goal state + goal region (joint goal, loose time) ----
  ob::ScopedState<> goal_state(space);
  {
    ob::State* s_goal = goal_state.get();
    auto* compound_goal = s_goal->as<ob::CompoundState>();

    auto* q = compound_goal->as<ob::RealVectorStateSpace::StateType>(0);
    auto* t = compound_goal->as<ob::RealVectorStateSpace::StateType>(1);

    for (std::size_t i = 0; i < dof; ++i)
      (*q)[i] = goal_positions[i];
    (*t)[0] = start_time + horizon * 0.8;  // some time in the future
  }


  auto pdef = std::make_shared<ob::ProblemDefinition>(si);
  pdef->setStartAndGoalStates(start, goal_state, 1e-2);

  auto goal_region = std::make_shared<JointGoalRegion>(si, dof, 0.01);
  goal_region->setGoal(goal_positions);
  pdef->setGoal(goal_region);

  // ---- 4) Planner: RRTConnect ----
  auto planner = std::make_shared<og::RRTConnect>(si);
  planner->setRange(0.2);  // step in state space; tune if desired
  planner->setProblemDefinition(pdef);
  planner->setup();

  ob::PlannerStatus solved =
    planner->solve(ob::timedPlannerTerminationCondition(10.0)); // 2s planning
  if (!solved)
  {
    ROS_WARN_STREAM("TimeAwareRRTConnectPlanner: no solution for " << this_agent.name
                << " with start_time=" << start_time
                << ", horizon=" << horizon
                << ", t_low=" << start_time
                << ", t_high=" << (start_time + horizon)
                << ", delta_time=" << delta_time_);

    return false;
  }



  auto* path = pdef->getSolutionPath()->as<og::PathGeometric>();
  if (!path || path->getStateCount() < 2)
  {
    ROS_WARN("TimeAwareRRTConnectPlanner: solution path too short.");
    return false;
  }

  // ---- 5) Extract (t, q) along path and sort by time ----
  std::vector<std::pair<double, std::vector<double>>> raw;
  raw.reserve(path->getStateCount());

  for (std::size_t i = 0; i < path->getStateCount(); ++i)
  {
    const ob::State* st = path->getState(i);
    const auto* compound = st->as<ob::CompoundState>();
    const auto* q = compound->as<ob::RealVectorStateSpace::StateType>(0);
    const auto* t = compound->as<ob::RealVectorStateSpace::StateType>(1);

    double time = (*t)[0];
    std::vector<double> qvec(dof);
    for (std::size_t j = 0; j < dof; ++j)
      qvec[j] = (*q)[j];

    raw.emplace_back(time, qvec);
  }

  std::sort(raw.begin(), raw.end(),
            [](const auto& a, const auto& b){ return a.first < b.first; });

  double final_time = raw.back().first;
  int num_steps = static_cast<int>(std::floor((final_time - start_time) / delta_time_));
  std::size_t idx = 0;

  out_traj.clear();
  out_traj.reserve(num_steps + 2);

  for (int step = 0; step <= num_steps; ++step)
  {
    double t_query = start_time + step * delta_time_;

    while (idx < raw.size() - 1 && raw[idx + 1].first <= t_query)
      idx++;

    double t1 = raw[idx].first;
    const auto& p1 = raw[idx].second;
    std::vector<double> interp_positions;

    if (idx == raw.size() - 1 || std::fabs(t1 - t_query) < 1e-6)
    {
      interp_positions = p1;
    }
    else
    {
      double t2 = raw[idx + 1].first;
      const auto& p2 = raw[idx + 1].second;
      double ratio = 0.0;
      if (t2 > t1 + 1e-9)
        ratio = (t_query - t1) / (t2 - t1);

      interp_positions.resize(p1.size());
      for (std::size_t k = 0; k < p1.size(); ++k)
        interp_positions[k] = p1[k] + ratio * (p2[k] - p1[k]);
    }

    AgentInfo::TrajectoryPoint pt;
    pt.time      = t_query;
    pt.positions = interp_positions;
    pt.velocities.clear(); // PBS or RHCR can fill later
    out_traj.push_back(pt);
  }

  if (final_time - (start_time + num_steps * delta_time_) > 1e-6)
  {
    AgentInfo::TrajectoryPoint final_pt;
    final_pt.time      = final_time;
    final_pt.positions = raw.back().second;
    final_pt.velocities.clear();
    out_traj.push_back(final_pt);
  }

  ROS_INFO("TimeAwareRRTConnectPlanner: produced %zu points in [%.3f, %.3f].",
           out_traj.size(), start_time, final_time);

  return true;
}
