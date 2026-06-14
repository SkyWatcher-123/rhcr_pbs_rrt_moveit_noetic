#include "rhcr_mapf_planners/rhcr_planner.h"
#include "rhcr_mapf_planners/pbs.h"

#include <XmlRpc.h>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>

// Geometry / TF / utils
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/package.h>

using namespace rhcr_mapf_planners;

RHCRPlanner::RHCRPlanner(ros::NodeHandle& nh)
  : nh_(nh)
{
  // Default values, can be overridden by parameters in initialize()
  delta_time_      = 1.0;
  max_vel_scaling_ = 1.0;
  max_acc_scaling_ = 1.0;
  max_rrt_nodes_   = 2000;
  horizon_         = 10.0;
}

bool RHCRPlanner::initialize()
{
  // Global parameters
  nh_.param("delta_time",               delta_time_,      1.0);
  nh_.param("max_velocity_scaling",     max_vel_scaling_, 1.0);
  nh_.param("max_acceleration_scaling", max_acc_scaling_, 1.0);
  nh_.param("horizon",                  horizon_,         10.0);
  nh_.param("replan_window",            replan_window_,   4.0);
  nh_.param("commit_horizon",           commit_horizon_,  2.0);
  nh_.param("max_windows",              max_windows_,     100);

  int max_rrt_nodes_int = static_cast<int>(max_rrt_nodes_);
  nh_.param("max_rrt_nodes", max_rrt_nodes_int, 2000);
  if (max_rrt_nodes_int <= 0)
    max_rrt_nodes_int = 2000;
  max_rrt_nodes_ = static_cast<std::size_t>(max_rrt_nodes_int);

  if (delta_time_ <= 0.0)
  {
    ROS_WARN("delta_time <= 0, resetting to 1.0 s");
    delta_time_ = 1.0;
  }

  // Load agents from YAML
  XmlRpc::XmlRpcValue agent_list;
  if (!nh_.getParam("agents", agent_list))
  {
    ROS_ERROR("Failed to find 'agents' parameter. Ensure YAML config is loaded.");
    return false;
  }
  if (agent_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("'agents' parameter must be a list.");
    return false;
  }

  // Parse each agent
  for (int i = 0; i < agent_list.size(); ++i)
  {
    XmlRpc::XmlRpcValue agent_cfg = agent_list[i];
    AgentInfo agent;

    if (!agent_cfg.hasMember("name") ||
        !agent_cfg.hasMember("move_group") ||
        !agent_cfg.hasMember("goals"))
    {
      ROS_ERROR("Agent entry %d missing name, move_group, or goals.", i);
      return false;
    }

    agent.name       = static_cast<std::string>(agent_cfg["name"]);
    agent.move_group = static_cast<std::string>(agent_cfg["move_group"]);

        // Optional folded joint configuration for this agent's group
    if (agent_cfg.hasMember("folded_joint"))
    {
      XmlRpc::XmlRpcValue folded = agent_cfg["folded_joint"];
      if (folded.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("folded_joint for agent '%s' must be a list.", agent.name.c_str());
        return false;
      }

      agent.folded_joint_values.clear(); 

      for (int k = 0; k < folded.size(); ++k)
      {
        if (folded[k].getType() == XmlRpc::XmlRpcValue::TypeInt)
          agent.folded_joint_values.push_back(static_cast<int>(folded[k]));
        else if (folded[k].getType() == XmlRpc::XmlRpcValue::TypeDouble)
          agent.folded_joint_values.push_back(static_cast<double>(folded[k]));
        else
        {
          ROS_ERROR("Non-numeric folded_joint value %d for agent '%s'.",
                    k, agent.name.c_str());
          return false;
        }
      }
    }


    XmlRpc::XmlRpcValue goals_list = agent_cfg["goals"];
    if (goals_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Agent '%s' goals should be a list.", agent.name.c_str());
      return false;
    }

    // Parse each goal
    for (int j = 0; j < goals_list.size(); ++j)
    {
      XmlRpc::XmlRpcValue goal_j = goals_list[j];
      AgentInfo::Goal goal;

      // Case 1: bare array => interpret as joint goal [backwards compatible].
      if (goal_j.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        goal.type = AgentInfo::Goal::JOINT;
        for (int k = 0; k < goal_j.size(); ++k)
        {
          if (goal_j[k].getType() == XmlRpc::XmlRpcValue::TypeInt)
            goal.joint_values.push_back(static_cast<int>(goal_j[k]));
          else if (goal_j[k].getType() == XmlRpc::XmlRpcValue::TypeDouble)
            goal.joint_values.push_back(static_cast<double>(goal_j[k]));
          else
          {
            ROS_ERROR("Non-numeric value in bare joint goal %d for agent '%s'.",
                      j, agent.name.c_str());
            return false;
          }
        }
      }
      // Case 2: struct => can be {joint: [...]} or {pose: {...}}
      else if (goal_j.getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        // 2a. joint goal
        if (goal_j.hasMember("joint"))
        {
          XmlRpc::XmlRpcValue jvals = goal_j["joint"];
          if (jvals.getType() != XmlRpc::XmlRpcValue::TypeArray)
          {
            ROS_ERROR("Agent '%s' joint goal %d must be an array.",
                      agent.name.c_str(), j);
            return false;
          }
          goal.type = AgentInfo::Goal::JOINT;
          for (int k = 0; k < jvals.size(); ++k)
          {
            if (jvals[k].getType() == XmlRpc::XmlRpcValue::TypeInt)
              goal.joint_values.push_back(static_cast<int>(jvals[k]));
            else if (jvals[k].getType() == XmlRpc::XmlRpcValue::TypeDouble)
              goal.joint_values.push_back(static_cast<double>(jvals[k]));
            else
            {
              ROS_ERROR("Non-numeric value in structured joint goal %d for agent '%s'.",
                        j, agent.name.c_str());
              return false;
            }
          }
        }
        // 2b. pose goal
        else if (goal_j.hasMember("pose"))
        {
          XmlRpc::XmlRpcValue pose_cfg = goal_j["pose"];
          if (pose_cfg.getType() != XmlRpc::XmlRpcValue::TypeStruct)
          {
            ROS_ERROR("Agent '%s' pose goal %d must be a struct.",
                      agent.name.c_str(), j);
            return false;
          }

          goal.type = AgentInfo::Goal::POSE;

          // Optional frame_id / end_effector_link
          if (pose_cfg.hasMember("frame_id"))
            goal.frame_id = static_cast<std::string>(pose_cfg["frame_id"]);
          if (pose_cfg.hasMember("end_effector_link"))
            goal.end_effector_link = static_cast<std::string>(pose_cfg["end_effector_link"]);

          // Position (required)
          if (!pose_cfg.hasMember("position") ||
              pose_cfg["position"].getType() != XmlRpc::XmlRpcValue::TypeArray ||
              pose_cfg["position"].size() != 3)
          {
            ROS_ERROR("Pose goal %d for agent '%s' must contain position[3].",
                      j, agent.name.c_str());
            return false;
          }
          XmlRpc::XmlRpcValue pos = pose_cfg["position"];
          goal.pose.position.x = static_cast<double>(pos[0]);
          goal.pose.position.y = static_cast<double>(pos[1]);
          goal.pose.position.z = static_cast<double>(pos[2]);

          // Orientation: orientation_rpy or orientation_quat or identity
          if (pose_cfg.hasMember("orientation_rpy"))
          {
            XmlRpc::XmlRpcValue rpy = pose_cfg["orientation_rpy"];
            if (rpy.getType() != XmlRpc::XmlRpcValue::TypeArray || rpy.size() != 3)
            {
              ROS_ERROR("orientation_rpy must be array[3] for agent '%s'.",
                        agent.name.c_str());
              return false;
            }
            double roll  = static_cast<double>(rpy[0]);
            double pitch = static_cast<double>(rpy[1]);
            double yaw   = static_cast<double>(rpy[2]);
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            goal.pose.orientation = tf2::toMsg(q);
          }
          else if (pose_cfg.hasMember("orientation_quat"))
          {
            XmlRpc::XmlRpcValue quat = pose_cfg["orientation_quat"];
            if (quat.getType() != XmlRpc::XmlRpcValue::TypeArray || quat.size() != 4)
            {
              ROS_ERROR("orientation_quat must be array[4] for agent '%s'.",
                        agent.name.c_str());
              return false;
            }
            goal.pose.orientation.x = static_cast<double>(quat[0]);
            goal.pose.orientation.y = static_cast<double>(quat[1]);
            goal.pose.orientation.z = static_cast<double>(quat[2]);
            goal.pose.orientation.w = static_cast<double>(quat[3]);
          }
          else
          {
            // default identity orientation
            goal.pose.orientation.x = 0.0;
            goal.pose.orientation.y = 0.0;
            goal.pose.orientation.z = 0.0;
            goal.pose.orientation.w = 1.0;
          }
        }
        else
        {
          ROS_ERROR("Goal %d for agent '%s' must contain 'joint' or 'pose'.",
                    j, agent.name.c_str());
          return false;
        }
      }
      else
      {
        ROS_ERROR("Unsupported goal type for agent '%s'.", agent.name.c_str());
        return false;
      }

      agent.goals.push_back(goal);
    }  // for each goal

    agent.current_goal_index = 0;
    agents_.push_back(agent);
  }  // for each agent

  ROS_INFO("Loaded %zu agents (delta_time=%.3f, vel_scale=%.2f, acc_scale=%.2f, horizon=%.2f, max_rrt_nodes=%zu).",
           agents_.size(), delta_time_, max_vel_scaling_, max_acc_scaling_, horizon_, max_rrt_nodes_);

  // --- Initialize shared robot model and planning scene for PBS ---
  robot_model_loader::RobotModelLoader model_loader("robot_description");
  robot_model_ = model_loader.getModel();
  if (!robot_model_)
  {
    ROS_ERROR("RHCRPlanner::initialize: Failed to load robot model from 'robot_description'.");
    return false;
  }

  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
  ROS_INFO("RHCRPlanner: robot_model and planning_scene created successfully.");

  return true;
}

bool RHCRPlanner::planAllAgents()
{
  if (agents_.empty())
  {
    ROS_ERROR("No agents loaded to plan for.");
    return false;
  }

  // PlanningSceneInterface not used directly here, but could be for external objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create a MoveGroupInterface per agent
  std::vector<std::unique_ptr<moveit::planning_interface::MoveGroupInterface>> move_groups;
  move_groups.reserve(agents_.size());
  for (size_t i = 0; i < agents_.size(); ++i)
  {
    const std::string& group_name = agents_[i].move_group;
    try
    {
      move_groups.emplace_back(
          new moveit::planning_interface::MoveGroupInterface(group_name));
    }
    catch (std::runtime_error& e)
    {
      ROS_ERROR("Failed to create MoveGroupInterface for group '%s': %s",
                group_name.c_str(), e.what());
      return false;
    }

    move_groups[i]->setPlannerId("RRTConnectkConfigDefault");
    move_groups[i]->setMaxVelocityScalingFactor(max_vel_scaling_);
    move_groups[i]->setMaxAccelerationScalingFactor(max_acc_scaling_);
    move_groups[i]->setPlanningTime(5.0);
  }

  // Plan for each agent through its sequence of goals
  for (size_t i = 0; i < agents_.size(); ++i)
  {
    AgentInfo& agent = agents_[i];
    ROS_INFO("Planning for agent '%s' (group: %s) with %zu goal(s)...",
             agent.name.c_str(), agent.move_group.c_str(), agent.goals.size());

    double time_offset = 0.0;
    std::vector<std::pair<double, std::vector<double>>> raw_traj_points;

    moveit::core::RobotStatePtr current_state = move_groups[i]->getCurrentState();

        // Option B: Solo planning – place all other agents into their folded configuration (if provided)
    if (current_state && robot_model_)
    {
      for (size_t j = 0; j < agents_.size(); ++j)
      {
        if (j == i)
          continue;

        const AgentInfo& other = agents_[j];
        if (other.folded_joint_values.empty())
          continue;

        const moveit::core::JointModelGroup* other_jmg =
            robot_model_->getJointModelGroup(other.move_group);
        if (!other_jmg)
        {
          ROS_WARN("No JointModelGroup for move_group '%s' when applying folded pose.",
                   other.move_group.c_str());
          continue;
        }

        if (other.folded_joint_values.size() != other_jmg->getVariableCount())
        {
          ROS_WARN("Folded joint size mismatch for agent '%s' (expected %zu, got %zu). Skipping.",
                   other.name.c_str(),
                   other_jmg->getVariableCount(),
                   other.folded_joint_values.size());
          continue;
        }

        current_state->setJointGroupPositions(other_jmg, other.folded_joint_values);
      }

      current_state->update();
    }

    // Local buffers for chaining goals via last waypoint
    std::vector<std::string> group_joint_names;
    std::vector<double>      last_positions;
    bool                     have_last_positions = false;



    for (size_t gi = 0; gi < agent.goals.size(); ++gi)
    {

      // --- Set the proper start state for this goal ---
      // For first goal: start from whatever MoveGroup considers current (already in current_state)
      if (gi == 0)
      {
        // For the first goal, use the current_state we constructed above
        if (current_state)
          move_groups[i]->setStartState(*current_state);
      }
      else if (have_last_positions && !group_joint_names.empty() && current_state)
      {
        // For later goals, start from the last waypoint of the previous plan
        current_state->setVariablePositions(group_joint_names, last_positions);
        current_state->update();
        move_groups[i]->setStartState(*current_state);
      }


      const AgentInfo::Goal& goal = agent.goals[gi]; // current goal

      // Target: joint or pose
      if (goal.type == AgentInfo::Goal::JOINT)
      {
        move_groups[i]->setJointValueTarget(goal.joint_values);
      }
      else // POSE
      {
        if (!goal.frame_id.empty())
          move_groups[i]->setPoseReferenceFrame(goal.frame_id);
        if (!goal.end_effector_link.empty())
          move_groups[i]->setEndEffectorLink(goal.end_effector_link);

        move_groups[i]->setPoseTarget(goal.pose);
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = (move_groups[i]->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (!success)
      {
        ROS_ERROR("Failed to plan for agent '%s' goal %zu.", agent.name.c_str(), gi);
        return false;
      }

      const trajectory_msgs::JointTrajectory& jt = plan.trajectory_.joint_trajectory;
      if (jt.points.empty())
      {
        ROS_ERROR("Empty trajectory for agent '%s' goal %zu.", agent.name.c_str(), gi);
        return false;
      }


      // Remember joint names and last waypoint positions for chaining goals
      if (group_joint_names.empty())
        group_joint_names = jt.joint_names;

      last_positions = jt.points.back().positions;
      have_last_positions = true;


      // Stitch joint trajectory; ensure monotonic time
      double local_start_time = time_offset;
      double local_end_time   = local_start_time + jt.points.back().time_from_start.toSec();

      for (size_t p = 0; p < jt.points.size(); ++p)
      {
        double t_local = jt.points[p].time_from_start.toSec();
        double t_global = local_start_time + t_local;

        raw_traj_points.emplace_back(
            t_global,
            jt.points[p].positions    // copy positions
        );
      }

      time_offset = local_end_time;
    }  // for each goal

    // Now discretize raw_traj_points at fixed delta_time_
    agent.trajectory.clear();
    if (raw_traj_points.empty())
    {
      ROS_WARN("Agent '%s' has no raw trajectory points.", agent.name.c_str());
      continue;
    }

    double total_time = raw_traj_points.back().first;
    int steps = static_cast<int>(std::ceil(total_time / delta_time_));
    if (steps <= 0) steps = 1;

    // Helper lambda: interpolate positions at time t
    auto interpolate = [&](double t) -> std::vector<double>
    {
      if (t <= raw_traj_points.front().first)
        return raw_traj_points.front().second;
      if (t >= raw_traj_points.back().first)
        return raw_traj_points.back().second;

      // find segment [k, k+1] such that tk <= t <= tk+1
      for (size_t k = 0; k + 1 < raw_traj_points.size(); ++k)
      {
        double t0 = raw_traj_points[k].first;
        double t1 = raw_traj_points[k + 1].first;
        if (t >= t0 && t <= t1)
        {
          double alpha = (t1 > t0) ? ((t - t0) / (t1 - t0)) : 0.0;
          const auto& q0 = raw_traj_points[k].second;
          const auto& q1 = raw_traj_points[k + 1].second;
          std::vector<double> q(q0.size(), 0.0);
          for (size_t idx = 0; idx < q0.size(); ++idx)
            q[idx] = (1.0 - alpha) * q0[idx] + alpha * q1[idx];
          return q;
        }
      }
      return raw_traj_points.back().second;
    };

    for (int s = 0; s <= steps; ++s)
    {
      double t = s * delta_time_;
      if (t > total_time)
        t = total_time;

      AgentInfo::TrajectoryPoint pt;
      pt.time      = t;
      pt.positions = interpolate(t);
      pt.velocities.clear();   // will be filled below
      agent.trajectory.push_back(pt);
    }

    // === Compute velocities for each timestep (simple finite differences) ===
    if (agent.trajectory.size() >= 2)
    {
      double dt = delta_time_;

      // First point: forward difference
      agent.trajectory[0].velocities.resize(agent.trajectory[0].positions.size());
      for (size_t j = 0; j < agent.trajectory[0].positions.size(); ++j)
      {
        double dq = agent.trajectory[1].positions[j] - agent.trajectory[0].positions[j];
        agent.trajectory[0].velocities[j] = dq / dt;
      }

      // Remaining points: backward difference
      for (size_t i = 1; i < agent.trajectory.size(); ++i)
      {
        agent.trajectory[i].velocities.resize(agent.trajectory[i].positions.size());
        for (size_t j = 0; j < agent.trajectory[i].positions.size(); ++j)
        {
          double dq = agent.trajectory[i].positions[j] - agent.trajectory[i - 1].positions[j];
          agent.trajectory[i].velocities[j] = dq / dt;
        }
      }
    }
    else if (agent.trajectory.size() == 1)
    {
      // Only one sample: zero velocity
      agent.trajectory[0].velocities.resize(agent.trajectory[0].positions.size(), 0.0);
    }

    ROS_INFO("Agent '%s' trajectory: duration=%.3f s, steps=%zu, dt=%.3f.",
             agent.name.c_str(), total_time, agent.trajectory.size(), delta_time_);
  } // for each agent

  return true;
}

void RHCRPlanner::resolveConflicts()
{
  if (!robot_model_ || !planning_scene_)
  {
    ROS_ERROR("RHCRPlanner::resolveConflicts: robot_model_ or planning_scene_ not initialized.");
    return;
  }

  bool use_replanning = true;
  nh_.param("pbs_use_replanning", use_replanning, true);
  if (!use_replanning)
  {
    ROS_INFO("RHCRPlanner: pbs_use_replanning=false, skipping PBS conflict resolution.");
    return;
  }

  // PBS uses the shared robot model / planning scene and time-aware parameters.
  PBS pbs(robot_model_, planning_scene_, delta_time_, horizon_, max_rrt_nodes_);
  pbs.setVelocityScaling(max_vel_scaling_);

  auto makespan = [&]() -> double {
    double m = 0.0;
    for (const auto& a : agents_)
      if (!a.trajectory.empty())
        m = std::max(m, a.trajectory.back().time);
    return m;
  };

  const double W = replan_window_;
  double H = (commit_horizon_ > 0.0) ? commit_horizon_ : replan_window_;
  if (H > W) H = W;   // RHCR invariant: commit slice h <= resolution window w

  // Degenerate window => single-shot resolution over the whole horizon.
  if (W <= 0.0)
  {
    pbs.resolveConflicts(agents_);
    updateCurrentGoalIndices(0.05);
    return;
  }

  // Rolling horizon: resolve conflicts within [t_now, t_now+W], commit the first
  // H seconds (frozen for later windows), then roll t_now forward by H.
  double t_now = 0.0;
  int round = 0;
  while (t_now < makespan() + 1e-6 && round < max_windows_)
  {
    double w_end = t_now + W;
    ROS_INFO("RHCR round %d: resolving conflicts in [%.2f, %.2f], committing up to %.2f.",
             round, t_now, w_end, t_now + H);

    if (!pbs.resolveConflicts(agents_, t_now, w_end))
      ROS_WARN("RHCR round %d: window not fully resolved; committing best-effort and rolling on.",
               round);

    t_now += H;
    ++round;
  }

  if (round >= max_windows_)
    ROS_WARN("RHCRPlanner::resolveConflicts: hit max_windows cap (%d).", max_windows_);

  updateCurrentGoalIndices(0.05);
}

void RHCRPlanner::updateCurrentGoalIndices(double goal_tolerance)
{
  for (auto& agent : agents_)
  {
    if (agent.goals.empty())
      continue;

    // start from whatever goal index is currently active
    int idx = agent.current_goal_index;
    if (idx < 0)
      idx = 0;
    if (idx >= static_cast<int>(agent.goals.size()))
      continue;

    // scan goals in order: if goal g is reached, bump index past it;
    // stop at the first not-yet-reached goal.
    for (int g = idx; g < static_cast<int>(agent.goals.size()); ++g)
    {
      const AgentInfo::Goal& goal = agent.goals[g];

      if (goal.type != AgentInfo::Goal::JOINT)
        continue;  // we assume you convert poses to joint goals before this

      bool reached = false;

      for (const auto& pt : agent.trajectory)
      {
        if (pt.positions.size() != goal.joint_values.size())
          continue;

        double dist2 = 0.0;
        for (std::size_t j = 0; j < pt.positions.size(); ++j)
        {
          double diff = pt.positions[j] - goal.joint_values[j];
          dist2 += diff * diff;
        }

        if (std::sqrt(dist2) <= goal_tolerance)
        {
          reached = true;
          break;
        }
      }

      if (reached)
      {
        agent.current_goal_index = g + 1;
      }
      else
      {
        // As soon as one goal is not reached, we stop; later goals
        // must wait for a later horizon.
        break;
      }
    }
  }
}

bool RHCRPlanner::outputTrajectories(const std::string& filename_param)
{
  // Get output folder and file name from parameters
  std::string folder_name, file_name;
  nh_.param("output_folder", folder_name, std::string("default_config"));
  nh_.param("output_file",   file_name,   std::string("trajectory.json"));

  // Find package path using rospack
  std::string package_path = ros::package::getPath("rhcr_mapf_planners");
  if (package_path.empty())
  {
    ROS_ERROR("Cannot find rhcr_mapf_planners package path.");
    return false;
  }

  // Construct output directory and full file path
  std::string output_dir  = package_path + "/output/" + folder_name;
  std::string output_path = output_dir + "/" + file_name;

  // Create directory if needed
  std::string mkdir_cmd = "mkdir -p " + output_dir;
  if (system(mkdir_cmd.c_str()) != 0)
  {
    ROS_WARN("Failed to create directory: %s", output_dir.c_str());
  }

  ROS_INFO("Writing trajectories to: %s", output_path.c_str());

  std::ofstream out(output_path);
  if (!out.is_open())
  {
    ROS_ERROR("Failed to open output file '%s' for writing.", output_path.c_str());
    return false;
  }

  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(6);

  oss << "{\n  \"agents\": [\n";
  for (size_t i = 0; i < agents_.size(); ++i)
  {
    const AgentInfo& agent = agents_[i];
    oss << "    {\n";
    oss << "      \"name\": \"" << agent.name << "\",\n";
    oss << "      \"trajectory\": [\n";

    for (size_t j = 0; j < agent.trajectory.size(); ++j)
    {
      const AgentInfo::TrajectoryPoint& pt = agent.trajectory[j];
      oss << "        { \"time\": " << pt.time
          << ", \"positions\": [";

      for (size_t k = 0; k < pt.positions.size(); ++k)
      {
        oss << pt.positions[k];
        if (k + 1 < pt.positions.size())
          oss << ", ";
      }

      oss << "], \"velocities\": [";

      for (size_t k = 0; k < pt.velocities.size(); ++k)
      {
        oss << pt.velocities[k];
        if (k + 1 < pt.velocities.size())
          oss << ", ";
      }

      oss << "] }";
      if (j + 1 < agent.trajectory.size())
        oss << ",";
      oss << "\n";

    }

    oss << "      ]\n";
    oss << "    }";
    if (i + 1 < agents_.size())
      oss << ",";
    oss << "\n";
  }
  oss << "  ]\n}\n";

  out << oss.str();
  out.close();
  ROS_INFO("Trajectories successfully written.");

  return true;
}
