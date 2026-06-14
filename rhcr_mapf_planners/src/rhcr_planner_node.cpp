#include <ros/ros.h>
#include "rhcr_mapf_planners/rhcr_planner.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rhcr_planner");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start(); // Start a background thread for ROS (required for MoveIt actions)

    rhcr_mapf_planners::RHCRPlanner planner(nh);
    if (!planner.initialize()) {
        ROS_ERROR("Failed to initialize RHCRPlanner. Shutting down.");
        return 1;
    }
    if (!planner.planAllAgents()) {
        ROS_ERROR("Multi-agent path planning failed.");
        return 1;
    }
    // planner.updateCurrentGoalIndices(0.05);
    // Resolve conflicts between agent paths using PBS
    planner.resolveConflicts();
    // Output the resulting trajectories to a JSON file
    planner.outputTrajectories("trajectories.json");

    ROS_INFO("RHCR multi-agent planning completed successfully.");
    return 0;
}
