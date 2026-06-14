#ifndef RHCR_MAPF_PLANNERS_PBS_H
#define RHCR_MAPF_PLANNERS_PBS_H

#include <vector>
#include <utility>

#include "rhcr_mapf_planners/rhcr_planner.h"
#include "rhcr_mapf_planners/time_aware_rrt.h"

namespace rhcr_mapf_planners {

/**
 * @brief Priority-Based Search (PBS) with time-aware RRT low-level planner.
 *
 * High-level behavior:
 *  - Start from a set of per-agent trajectories (from RHCR/time_aware_RRT).
 *  - Detect earliest conflict in spacetime.
 *  - Branch on priority: i ≺ j and j ≺ i.
 *  - In each branch, replan only the lower-priority agent with TimeAwareRRT,
 *    treating all higher-priority agents as dynamic obstacles.
 *  - Repeat until no conflicts remain or a search limit is reached.
 *
 * This is a simplified, single-file PBS implementation approximating the
 * original algorithmic spirit from the RHCR repo, adapted to continuous joint
 * space and MoveIt-based collision checking.
 */
class PBS
{
public:
  PBS(const moveit::core::RobotModelConstPtr& robot_model,
      const planning_scene::PlanningScenePtr& planning_scene,
      double delta_time,
      double horizon,
      std::size_t max_rrt_nodes);

  /**
   * @brief Resolve conflicts for the given set of agents / trajectories.
   *
   * On success, modifies @p agents in-place to be collision-free (within
   * this RHCR horizon). On failure, agents are left in the last attempted
   * node (may still contain collisions).
   *
   * This function is intended to be called once per RHCR time window.
   */
  bool resolveConflicts(std::vector<AgentInfo>& agents);

private:
  struct PriorityConstraint
  {
    std::size_t before;  ///< agent index that must go first
    std::size_t after;   ///< agent index that must yield
  };

  struct PBSNode
  {
    std::vector<AgentInfo> agents;               ///< current trajectories
    std::vector<PriorityConstraint> constraints; ///< partial priority order
  };

  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  double delta_time_;
  double horizon_;
  std::size_t max_rrt_nodes_;

  // === Core PBS recursion ===
  bool solveNode(PBSNode& node, std::size_t depth, std::size_t max_depth);

  // === Conflict detection ===
  struct Conflict
  {
    bool valid;
    std::size_t a;  ///< agent index
    std::size_t b;  ///< agent index
    double t;       ///< time of conflict
  };

  Conflict findEarliestConflict(const PBSNode& node) const;

  // === Priority utilities ===

  /**
   * @brief Check if constraints imply that "before" must go before "after".
   */
  bool isBefore(const PBSNode& node, std::size_t before, std::size_t after) const;

  /**
   * @brief Collect all higher-priority agents for a given agent index
   *        according to the node's constraints.
   */
  std::vector<std::size_t> getHigherPriorityAgents(const PBSNode& node,
                                                   std::size_t agent_idx) const;

  // === Goal-segment reasoning ===

  /**
   * @brief For a given agent and conflict time t_conflict, determine
   *        which goal segment it is currently on.
   *
   *  - Returns (segment_goal_index, t_reached_prev_goal):
   *      * segment_goal_index = index g such that the agent is on the
   *        segment from goal[g] to goal[g+1] (or from start to goal[0]
   *        if g == -1).
   *      * t_reached_prev_goal = time when goal[g] was first reached on
   *        the current trajectory (or 0 if none).
   *
   *  The local replan will target goal[g+1] and will NOT skip further goals.
   */
  std::pair<int, double> findActiveGoalSegment(const AgentInfo& agent,
                                               double t_conflict,
                                               double goal_tol) const;

  // === Local replan with time-aware RRT ===

  /**
   * @brief Locally replan the given agent from a replan start time toward the
   *        next goal, respecting higher-priority agents as constraints.
   *
   *  - We freeze the prefix of the agent's trajectory up to t_freeze_end
   *    (strong commitment / SIPP-like behavior).
   *  - We choose a replan start time t_replan such that:
   *      t_replan >= t_freeze_end and t_replan <= t_conflict.
   *  - The new segment replaces everything from t_replan onward.
   */
  bool localReplan(PBSNode& node,
                   std::size_t agent_idx,
                   double t_conflict,
                   double t_freeze_margin,
                   double goal_tol);

  // === Low-level utilities ===

  /**
   * @brief Get agent's configuration at given time by scanning its trajectory.
   */
  const std::vector<double>& getAgentPositionsAtTime(const AgentInfo& agent,
                                                     double t_query) const;

  /**
   * @brief Compute maximum time across all agents' trajectories.
   */
  double getMaxTrajectoryTime(const PBSNode& node) const;
};

}  // namespace rhcr_mapf_planners

#endif  // RHCR_MAPF_PLANNERS_PBS_H
