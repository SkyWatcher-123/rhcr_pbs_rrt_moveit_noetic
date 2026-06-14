#ifndef RHCR_MAPF_PLANNERS_TIME_AWARE_RRTCONNECT_H
#define RHCR_MAPF_PLANNERS_TIME_AWARE_RRTCONNECT_H

#include <vector>
#include <string>
#include <memory>

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>

#include "rhcr_mapf_planners/rhcr_planner.h"

namespace rhcr_mapf_planners {

/**
 * @brief Time-aware RRTConnect in [q, t] for a single agent.
 *
 * State = (q[0..dof-1], t).
 * Validity = MoveIt collision check at time t, where:
 *   - this agent's joints are taken from q,
 *   - all *higher-priority* agents (lower index) are set from their trajectories
 *     at time t (moving obstacles),
 *   - lower-priority agents are ignored (they will adapt later).
 *
 * Used inside PBS local replanning: start from (q_start, t_conflict)
 * and plan to the current goal's joint configuration.
 */
class TimeAwareRRTConnectPlanner
{
public:
  TimeAwareRRTConnectPlanner(const moveit::core::RobotModelConstPtr& robot_model,
                             const planning_scene::PlanningScenePtr& planning_scene,
                             const std::vector<AgentInfo>& agents,
                             std::size_t this_agent_idx,
                             double delta_time);

  /**
   * @brief Plan from q_start at start_time to q_goal.
   *
   * @param start_time      conflict time t_c (planning starts here).
   * @param start_positions current joint positions of this agent at t_c.
   * @param goal_positions  target joint positions for the current goal.
   * @param out_traj        output DT-discretized trajectory points >= start_time.
   * @return true if a solution was found.
   */
  bool plan(double start_time,
            const std::vector<double>& start_positions,
            const std::vector<double>& goal_positions,
            std::vector<AgentInfo::TrajectoryPoint>& out_traj);

private:
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  const std::vector<AgentInfo>& agents_;
  std::size_t this_agent_idx_;
  double delta_time_;

  const std::vector<double>& getAgentPositionsAtTime(std::size_t agent_index,
                                                     double t_query) const;
};

}  // namespace rhcr_mapf_planners

#endif  // RHCR_MAPF_PLANNERS_TIME_AWARE_RRTCONNECT_H
