# rhcr_mapf_planners

This ROS package implements a **Rolling Horizon Collision Resolution (RHCR)** multi-agent path planner with a priority-based conflict resolution module, integrated with MoveIt for motion planning. It allows multiple robotic agents (e.g., multiple arms or robots) to plan collision-free paths to sequential goals while respecting velocity constraints and resolving dynamic conflicts between agents.

## Package Structure

- **Core Modules**:
  - `RHCRPlanner` (src/rhcr_planner.cpp, include/rhcr_mapf_planners/rhcr_planner.h): Orchestrates planning for all agents and coordinates the overall RHCR algorithm. It loads agent configurations, calls MoveIt (using RRTConnect planner) to generate paths, discretizes those trajectories into fixed timesteps, and prepares them for conflict checking.
  - `PBS` (src/pbs.cpp, include/rhcr_mapf_planners/pbs.h): Priority-Based Search conflict resolution. Detects collisions between agents’ trajectories using MoveIt’s state validity (collision checking) and resolves conflicts by inserting wait steps for lower-priority agents (delaying their motion) until all trajectories are collision-free.
  - `rhcr_planner_node.cpp`: The ROS node that ties everything together. It initializes the planner, executes the planning and conflict resolution, and outputs the final trajectories.

- **Config**:
  - `config/demo_agents.yaml`: YAML configuration listing each agent, its corresponding MoveIt planning group, and a list of joint-space goal positions for that agent. This is loaded into the ROS parameter server on launch.

- **Launch**:
  - `launch/demo.launch`: Launch file that loads the configuration and runs the planner node. **Note:** This launch assumes that the MoveIt planning server (`move_group` for your robot(s)) is already running (for example, via your robot’s MoveIt config launch file), so that the planner can query current states and plan trajectories.

- **Output**:
  - The planner outputs a JSON file (`trajectories.json`) in the current directory containing the planned trajectories for each agent. Each trajectory is a list of time-stamped joint positions representing the planned motion (with discrete time steps). An example snippet of the JSON structure:
    ```json
    {
      "agents": [
        {
          "name": "robot1",
          "trajectory": [
            { "time": 0.0, "positions": [0.0, 0.0, ...] },
            { "time": 1.0, "positions": [0.1, 0.05, ...] },
            ...
            { "time": 5.0, "positions": [1.0, 0.5, ...] }
          ]
        },
        { ... robot2 trajectory ... }
      ]
    }
    ```
    Each agent’s trajectory starts at time 0 with its initial joint configuration and includes all subsequent positions at 1-second intervals (plus the final point, which may have a fractional time if the total duration isn’t an integer). If an agent had to wait due to a conflict, you will see consecutive entries with identical positions (indicating a pause).

## Usage Instructions

1. **Setup and Build**: Clone this package into your Catkin workspace (e.g., `catkin_ws/src`) and run `catkin_make` to build. Ensure that MoveIt (Noetic) is installed, as this package depends on MoveIt’s libraries.
   ```bash
   cd ~/catkin_ws
   catkin_make
