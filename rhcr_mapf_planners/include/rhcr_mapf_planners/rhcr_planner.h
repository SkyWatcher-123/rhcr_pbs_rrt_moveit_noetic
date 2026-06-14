#ifndef RHCR_MAPF_PLANNERS_RHCR_PLANNER_H
#define RHCR_MAPF_PLANNERS_RHCR_PLANNER_H

#include <ros/ros.h>
#include <vector>
#include <string>

#include <geometry_msgs/Pose.h>

// MoveIt core types for PBS / RHCR
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>

namespace rhcr_mapf_planners
{

// Forward declare PBS class for use in RHCRPlanner
class PBS;

/**
 * @brief Data structure representing an agent, its goals, and its trajectory.
 */
struct AgentInfo
{
  struct Goal
  {
    enum Type { JOINT = 0, POSE = 1 } type;
    std::vector<double> joint_values;   // used when type == JOINT
    geometry_msgs::Pose pose;           // used when type == POSE
    std::string frame_id;               // pose frame (optional)
    std::string end_effector_link;      // EE link (optional)
  };

  struct TrajectoryPoint
  {
    double time;                        // absolute time [s] from horizon start
    std::vector<double> positions;      // joint positions
    std::vector<double> velocities;     // joint velocities (finite-diff, used by PBS)
  };

  std::string name;                     // logical agent name
  std::string move_group;               // MoveIt planning group name
  std::vector<double> folded_joint_values; // optional folded config for this agent's group

  std::vector<Goal> goals;              // ordered goal list (from YAML)
  std::vector<TrajectoryPoint> trajectory;
  


  // Index of the next YAML goal this agent is aiming for.
  // RHCR / PBS can bump this when goals are reached.
  int current_goal_index;
};


/**
 * @brief Rolling Horizon Collision Resolution Planner class.
 *
 * This class orchestrates multi-agent planning using an RHCR-style workflow:
 *  - Reads configuration of agents and their goals from ROS parameters (YAML).
 *  - Uses MoveIt (RRTConnect) to plan individual paths for each agent to each
 *    goal, then stitches them together.
 *  - Samples the resulting trajectories at fixed time steps (delta_time_)
 *    and computes simple finite-difference velocities.
 *  - Hands those discrete trajectories to PBS, which performs (time-aware)
 *    priority-based collision resolution, using the MoveIt robot model and
 *    planning scene.
 */
class RHCRPlanner
{
public:
  explicit RHCRPlanner(ros::NodeHandle& nh);

  /// Load agent configurations and global parameters from ROS params (YAML file).
  bool initialize();

  /// Plan paths for all agents through their goals using time_aware_rrt.
  bool planAllAgents();

  /// Resolve inter-agent collisions among the already-planned trajectories using PBS.
  void resolveConflicts();

  /// Write the resulting trajectories to a JSON file (path is resolved using params).
  bool outputTrajectories(const std::string& filename_param);

  /// Inspect trajectories and update which goals are considered reached, per agent.
  void updateCurrentGoalIndices(double goal_tolerance);

private:
  ros::NodeHandle nh_;
  std::vector<AgentInfo> agents_;            ///< List of agents with configurations and trajectories

  // Time step and kinodynamic knobs
  double delta_time_;                        ///< Discretization step [s]
  double max_vel_scaling_;                   ///< Velocity scaling (0–1) passed to MoveIt
  double max_acc_scaling_;                   ///< Acceleration scaling (0–1) passed to MoveIt

  // Global time-aware RRT / RHCR parameters
  std::size_t max_rrt_nodes_ = 2000;         ///< Default maximum nodes for time-aware RRT
  double      horizon_       = 10.0;         ///< Default planning horizon [s]

  // Shared MoveIt model & planning scene used by PBS (and potentially RHCR)
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;

  // Allow PBS class to access internal agent list if needed
  friend class rhcr_mapf_planners::PBS;
};

}  // namespace rhcr_mapf_planners

#endif  // RHCR_MAPF_PLANNERS_RHCR_PLANNER_H
