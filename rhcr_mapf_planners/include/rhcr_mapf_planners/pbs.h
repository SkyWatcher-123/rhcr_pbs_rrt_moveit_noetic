#ifndef RHCR_MAPF_PLANNERS_PBS_H
#define RHCR_MAPF_PLANNERS_PBS_H

#include <vector>
#include <utility>
#include <string>
#include <set>
#include <unordered_map>

#include <moveit/robot_state/robot_state.h>

#include "rhcr_mapf_planners/rhcr_planner.h"
#include "rhcr_mapf_planners/time_aware_rrt.h"

namespace rhcr_mapf_planners {

/**
 * @brief Priority-Based Search (PBS) with time-aware RRT low-level planner.
 *
 * High-level behavior:
 *  - Start from a set of per-agent trajectories (from RHCR / time_aware_RRT).
 *  - Detect the earliest spacetime conflict (optionally restricted to a time
 *    window, for rolling-horizon use).
 *  - Branch on priority: i precedes j and j precedes i.
 *  - In each branch, replan the lower-priority agent (and, transitively, any
 *    still-lower-priority agent it disturbs, in topological order) with the
 *    time-aware RRT, treating all higher-priority agents as moving obstacles.
 *  - Repeat until the window is conflict-free or a search limit is reached.
 *
 * Key properties of this (fixed) implementation:
 *  - A replan starts from a *valid* state strictly before the conflict
 *    (committed prefix is frozen), so the low-level seed is never the
 *    colliding configuration itself.
 *  - A replan re-plans the agent's *entire remaining goal sequence*, so
 *    yielding never truncates an agent's ordered goals.
 *  - Conflict detection / resolution can be bounded to [window_start,
 *    window_end), which is what the RHCR rolling-horizon loop drives.
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
   * @brief Resolve all conflicts over the whole horizon (backwards compatible).
   */
  bool resolveConflicts(std::vector<AgentInfo>& agents);

  /**
   * @brief Resolve only conflicts whose time t lies in [window_start, window_end).
   *
   * The trajectory prefix before @p window_start is treated as committed and is
   * never modified. On success, @p agents is updated in place.
   */
  bool resolveConflicts(std::vector<AgentInfo>& agents,
                        double window_start, double window_end);

  /// Velocity scaling (0..1) applied to per-joint model velocity limits.
  void setVelocityScaling(double s) { vel_scale_ = s; }

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

  struct Conflict
  {
    bool valid;
    std::size_t a;  ///< agent index
    std::size_t b;  ///< agent index
    double t;       ///< time of conflict
  };

  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  double delta_time_;
  double horizon_;
  std::size_t max_rrt_nodes_;
  double vel_scale_ = 1.0;
  mutable moveit::core::RobotStatePtr work_state_;  ///< scratch state (no scene mutation)

  // === Core PBS recursion ===
  bool solveNode(PBSNode& node, std::size_t depth, std::size_t max_depth,
                 double w_start, double w_end);

  // === Conflict detection ===
  Conflict findEarliestConflict(const PBSNode& node, double w_start, double w_end) const;
  void buildLinkToAgent(const PBSNode& node,
                        std::unordered_map<std::string, std::size_t>& m) const;
  void collidingPairsAtTime(const PBSNode& node, double t,
                            const std::unordered_map<std::string, std::size_t>& m,
                            std::set<std::pair<std::size_t, std::size_t>>& pairs) const;
  bool firstConflictTime(const PBSNode& node, std::size_t a, std::size_t b,
                         double w_start, double w_end, double& t_out) const;

  // === Priority utilities ===
  bool isBefore(const PBSNode& node, std::size_t before, std::size_t after) const;
  std::vector<std::size_t> getHigherPriorityAgents(const PBSNode& node,
                                                   std::size_t agent_idx) const;

  // === Replanning ===

  /**
   * @brief Replan @p yield_agent and propagate to transitively lower-priority
   *        agents (in topological order) until all ordered pairs are consistent.
   *        Unordered conflicts are left for solveNode to branch on.
   */
  bool replanAndMakeConsistent(PBSNode& node, std::size_t yield_agent,
                               double t_seed, double w_start, double w_end);

  /**
   * @brief Freeze the valid prefix and replan the agent's entire remaining goal
   *        sequence from a valid freeze start, vs. its higher-priority agents.
   */
  bool replanAgentSuffix(PBSNode& node, std::size_t agent_idx,
                         double t_seed, double w_start, double w_end);

  /**
   * @brief Step back from @p t_seed (snapped to the time grid) to the latest
   *        still-valid state >= max(0, w_start). Returns -1 if none exists.
   */
  double findValidFreezeStart(const PBSNode& node, std::size_t agent_idx,
                              const std::vector<std::size_t>& higher,
                              double t_seed, double w_start) const;

  /// Index of the next not-yet-reached JOINT goal at time @p t (clamped to last).
  int nextGoalIndexAtTime(const AgentInfo& agent, double t, double goal_tol) const;

  /// Validity of @p q for @p agent_idx vs. its higher-priority agents + world/self.
  bool stateValidVsHigher(const PBSNode& node, std::size_t agent_idx,
                          const std::vector<double>& q, double t,
                          const std::vector<std::size_t>& higher) const;

  /// Per-joint velocity limits for @p group, read from the model and scaled.
  std::vector<double> jointVelocityLimits(const std::string& group) const;

  // === Low-level utilities ===

  /// Interpolated configuration of @p agent at @p t_query (linear, by value).
  std::vector<double> getAgentPositionsAtTime(const AgentInfo& agent, double t_query) const;

  /// Maximum time across all agents' trajectories.
  double getMaxTrajectoryTime(const PBSNode& node) const;
};

}  // namespace rhcr_mapf_planners

#endif  // RHCR_MAPF_PLANNERS_PBS_H
