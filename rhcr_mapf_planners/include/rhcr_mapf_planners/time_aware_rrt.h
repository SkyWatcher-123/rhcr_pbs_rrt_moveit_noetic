#ifndef RHCR_MAPF_PLANNERS_TIME_AWARE_RRT_H
#define RHCR_MAPF_PLANNERS_TIME_AWARE_RRT_H

#include <vector>
#include <cstddef>
#include <memory>
#include <string>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

#include "rhcr_mapf_planners/rhcr_planner.h"

namespace rhcr_mapf_planners {

/**
 * @brief Single-tree, forward-only RRT in [q, t].
 *
 *  - State x = (q, t) where q is joint configuration and t is scalar time.
 *  - Time is strictly increasing: every edge advances t by a fixed delta_t.
 *  - Validity:
 *      * joint limits
 *      * optional simple velocity bound
 *      * collision-free w.r.t. static world and higher-priority agents'
 *        time-varying trajectories at time t.
 *
 * This planner does NOT use OMPL’s RRT; it is a minimal custom
 * implementation tailored for PBS/RHCR usage.
 */
class TimeAwareRRT
{
public:
  /**
   * @param robot_model       Robot model loaded from /robot_description.
   * @param planning_scene    Shared planning scene (world geometry).
   * @param agents            Reference to all agent info; only trajectories
   *                          of higher-priority agents are used as obstacles.
   * @param this_agent_idx    Index of the agent this planner is planning for.
   * @param delta_time        Fixed time step Δt (seconds) between nodes.
   * @param horizon           Planning horizon (seconds); t never exceeds
   *                          start_time + horizon.
   * @param max_nodes         Max number of nodes in the RRT before giving up.
   */
  TimeAwareRRT(const moveit::core::RobotModelConstPtr& robot_model,
               const planning_scene::PlanningScenePtr& planning_scene,
               const std::vector<AgentInfo>& agents,
               std::size_t this_agent_idx,
               double delta_time,
               double horizon,
               std::size_t max_nodes);

  /**
   * @brief Plan a segment from (q_start, t_start) to goal joint configuration.
   *
   *  - Single tree growing from (q_start, t_start).
   *  - Every expansion advances time by exactly delta_time_.
   *  - Higher-priority agents (given by higher_priority_indices) are treated
   *    as dynamic obstacles via their trajectories.
   *  - A simple velocity bound is enforced per joint.
   *
   * @param t_start                Starting time.
   * @param q_start                Starting joint configuration.
   * @param q_goal                 Target joint configuration (for the next goal).
   * @param higher_priority_indices List of agent indices that are higher priority.
   * @param out_traj               Output DT-discretized trajectory, including
   *                               the start state and all intermediate nodes,
   *                               up to reaching q_goal or failing.
   * @return true if a path was found; false otherwise.
   */
  bool plan(double t_start,
            const std::vector<double>& q_start,
            const std::vector<double>& q_goal,
            const std::vector<std::size_t>& higher_priority_indices,
            std::vector<AgentInfo::TrajectoryPoint>& out_traj);

  /**
   * @brief Set the maximum joint velocity (rad/s) used by the simple
   *        velocity constraint.
   *
   * If you don't call this, a default value of 1.0 rad/s is used.
   */
  void setMaxJointVelocity(double v_max);

  /**
   * @brief Set per-joint velocity bounds (rad/s, or m/s for prismatic joints).
   *
   * Overrides the scalar bound from setMaxJointVelocity() when non-empty, so the
   * step size respects each joint's own limit (e.g. a rail vs. a wrist joint).
   */
  void setMaxJointVelocities(const std::vector<double>& v_max_per_joint);

  /**
   * @brief Fraction of expansions [0,1] that attempt a pure "wait" edge:
   *        hold the configuration and advance time by delta_time_.
   *
   * Waiting lets a lower-priority agent yield by pausing until a higher-priority
   * agent clears. Default is 0.2.
   */
  void setWaitBias(double wait_bias);

private:
  struct Node
  {
    std::vector<double> q;  ///< joint configuration
    double t;               ///< time stamp
    int parent;             ///< parent index in tree (-1 for root)
  };

  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  const std::vector<AgentInfo>& agents_;
  std::size_t this_agent_idx_;
  double delta_time_;
  double horizon_;
  std::size_t max_nodes_;
  double v_max_;                          ///< scalar velocity bound (fallback)
  std::vector<double> v_max_per_joint_;   ///< per-joint bounds (empty => use v_max_)
  double wait_bias_ = 0.2;                ///< probability of a wait expansion
  mutable moveit::core::RobotStatePtr work_state_;  ///< scratch state (no scene mutation)

  /**
   * @brief Sample a random configuration in joint space.
   *
   *  - We use the JointModelGroup for this agent.
   *  - Joint bounds are taken from the robot model.
   */
  std::vector<double> sampleRandomConfiguration(const moveit::core::JointModelGroup* jmg) const;

  /**
   * @brief Sample a random configuration biased toward the goal.
   *
   *  With some probability, we sample near q_goal; otherwise fully random.
   */
  std::vector<double> sampleBiasedConfiguration(const moveit::core::JointModelGroup* jmg,
                                                const std::vector<double>& q_goal,
                                                double goal_bias) const;

  /**
   * @brief Find index of nearest node in the tree (in joint space distance).
   *
   * Time is not used for the nearest-neighbor metric; we only use joint-space
   * Euclidean distance. We do, however, enforce monotonic time by always
   * setting t_new = t_parent + delta_time_.
   */
  int findNearestNode(const std::vector<Node>& tree,
                      const std::vector<double>& q_sample) const;

  /**
   * @brief Generate a new configuration by stepping from q_from toward q_to.
   *
   *  - The step is limited such that max |dq_i| <= v_max_ * delta_time_.
   *  - If the distance is smaller than that, we snap directly to q_to.
   */
  std::vector<double> steer(const std::vector<double>& q_from,
                            const std::vector<double>& q_to) const;

  /**
   * @brief Check if a configuration is valid at a given time t.
   *
   *  Uses MoveIt to:
   *   - set this agent's joints to q,
   *   - set all higher-priority agents' joints from their trajectories at t,
   *   - perform a collision check.
   */
  bool isStateValid(const std::vector<double>& q,
                    double t,
                    const std::vector<std::size_t>& higher_priority_indices,
                    const moveit::core::JointModelGroup* jmg) const;

  /**
   * @brief Get an agent's joint positions at time t_query by scanning its
   *        trajectory and returning the first point whose time >= t_query.
   *
   *  - If t_query is before the first waypoint, returns the first.
   *  - If t_query is after the last waypoint, returns the last.
   *  - Otherwise linearly interpolates between the bracketing waypoints.
   */
  std::vector<double> getAgentPositionsAtTime(std::size_t agent_index,
                                              double t_query) const;

  /**
   * @brief Compute squared Euclidean distance between two joint vectors.
   */
  static double distanceSquared(const std::vector<double>& a,
                                const std::vector<double>& b);
};

}  // namespace rhcr_mapf_planners

#endif  // RHCR_MAPF_PLANNERS_TIME_AWARE_RRT_H
