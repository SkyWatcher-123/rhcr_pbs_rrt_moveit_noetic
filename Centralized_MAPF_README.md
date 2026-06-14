# Centralized_MAPF: RHCR → PBS → Space-Time RRTConnect for ROS1 MoveIt Noetic

## 1. Purpose

`Centralized_MAPF` is a centralized, time-aware multi-robot planning package for ROS1 Noetic and MoveIt. It is intended to coordinate multiple MoveIt planning groups, especially large industrial manipulators, by combining:

```text
RHCR windowed coordinator
    ↓
PBS high-level multi-agent priority planner
    ↓
Single-manipulator space-time RRTConnect planner in [q, t]
    ↓
MoveIt PlanningScene collision checking and trajectory execution
```

The package is designed for heterogeneous construction robotics, where robots share dense construction space with other robots, evolving geometry, and human workers. The central idea is that space is not only geometric but also temporal: each robot trajectory becomes a space-time reservation that other agents must respect.

The first intended implementation focuses on Type-A industrial manipulators or rail-mounted arms. Type-B mobile cobots can later consume the same reservation database, but their base navigation can be handled by a separate SIPP/A*/DWA or move_base stack.

---

## 2. Why this package is needed

Standard ROS1 MoveIt + OMPL can plan for:

```text
one move_group
```

or for:

```text
one parent move_group containing multiple robots
```

However, the parent-group approach forces all robots to plan and execute together as one large robot. That does not support the target use case, where:

- Robot A may start before Robot B.
- Robot B may plan while Robot A is already executing.
- Robot A may reserve a sweep volume in space-time.
- Robot B must query Robot A's state at time `t`.
- A high-level planner must branch on priority constraints instead of solving one monolithic OMPL problem.

`Centralized_MAPF` keeps each robot as an independent MoveIt group, but coordinates them through a centralized trajectory database and PBS priority search.

---

## 3. High-level architecture

```text
Task graph / digital twin / Right-of-Way constraints
        ↓
RHCRCoordinator
        ↓
PBSPlanner
        ↓
TimeAwareSingleRobotPlanner
        ↓
TimeAwareStateValidityChecker
        ↓
MoveIt PlanningScene
        ↓
RobotTrajectory / JointTrajectory execution
```

Recommended core modules:

```text
RHCRCoordinator
    Selects a bounded task window.
    Assigns candidate tasks to robots.
    Provides nominal start and finish times.
    Sends a small planning problem to PBS.

PBSPlanner
    Builds a high-level priority search tree.
    Detects inter-robot conflicts.
    Branches on priority constraints.
    Replans only affected lower-priority agents.

TimeAwareSingleRobotPlanner
    Plans one MoveIt group in [q, t].
    Uses OMPL RRTConnect.
    Checks robot-environment and robot-robot collision.
    Enforces velocity-safe edges.

MultiRobotTrajectoryDB
    Stores planned trajectories as time-indexed joint states.
    Interpolates q_i(t) for collision checking.
    Serves as the space-time reservation table.

ExecutionManager
    Converts timed paths into MoveIt RobotTrajectory or JointTrajectory.
    Dispatches trajectories to controllers.
    Monitors delay, deviation, and interruption.
```

---

## 4. RHCR layer

### 4.1 Conceptual role

The RHCR layer is a rolling-horizon coordinator. It should not directly sample joint space. Its role is to define a small, bounded planning problem for PBS.

It receives:

```text
current digital twin
current robot states
available tasks
task precedence graph
human / obstacle reservations
previously committed robot reservations
Right-of-Way hierarchy
```

It outputs:

```text
planning window W = [t_now, t_now + H]
candidate tasks inside the window
robot-task assignments
nominal start times
nominal finish times
hard reservations from previous plans
soft reservations / preference costs
```

For Type-A structural arms, RHCR can be relatively conservative and batch-oriented. For Type-B mobile cobots, RHCR can later become a more online receding-horizon coordinator.

### 4.2 Task specification

A generic task can be represented as:

```cpp
struct TaskSpec
{
    int task_id;
    std::string task_type;      // A_PLACE_MEMBER, A_HOLD, A_RETRACT, B_PANEL, etc.
    std::string assigned_group; // MoveIt planning group
    std::vector<int> predecessors;

    std::string start_named_target;
    std::string goal_named_target;

    double earliest_start;
    double nominal_finish;
    double latest_finish;

    double priority_weight;
    bool fixed_assignment;
};
```

For the minimal manipulator-only version, each task can be simplified to:

```text
one MoveIt group
one start joint state or named target
one goal joint state or named target
one time window
```

Later, the task can be expanded into multi-stage manipulation:

```text
home → pregrasp → grasp → transfer → place → retract
```

---

## 5. PBS layer

### 5.1 Conceptual role

PBS sits between RHCR and the low-level RRT planner. PBS does not know how to sample robot joint space. Instead, it manages priority constraints:

```text
Robot A has priority over Robot B.
Robot B must plan around Robot A's trajectory.
```

A PBS node should contain:

```text
priority constraints
current paths for each agent
detected conflicts
cost / makespan
parent / children
```

When a conflict is detected between two robot trajectories, PBS branches:

```text
Branch 1: A has higher priority than B
    Replan B around A.

Branch 2: B has higher priority than A
    Replan A around B.
```

This preserves the logic of classical priority-based search, but replaces grid-based low-level planning with a MoveIt/OMPL planner in continuous joint-time space.

### 5.2 PBS input

```cpp
struct PBSProblem
{
    std::vector<AgentSpec> agents;
    std::vector<TaskSpec> tasks;
    std::vector<ExternalReservation> fixed_reservations;
    double window_start;
    double window_end;
    double dt_conflict_check;
};
```

Each agent maps to a MoveIt group:

```cpp
struct AgentSpec
{
    int agent_id;
    std::string group_name;
    const moveit::core::JointModelGroup* jmg;

    std::vector<double> q_start;
    std::vector<double> q_goal;

    double t_min;
    double t_goal;
    double t_max;

    int robot_type; // TYPE_A_MOVING, TYPE_A_IDLE, TYPE_B, etc.
};
```

### 5.3 PBS output

```cpp
struct PBSResult
{
    bool success;
    std::vector<TimedJointTrajectory> trajectories;
    std::vector<PriorityConstraint> priority_constraints;
    double makespan;
    int high_level_expanded;
    int low_level_calls;
};
```

The output trajectories are then converted into:

```text
MoveIt RobotTrajectory
trajectory_msgs/JointTrajectory
space-time reservations
execution monitor targets
RViz visualization markers
```

---

## 6. Single-manipulator space-time RRTConnect planner

### 6.1 Core idea

Each robot plans in an extended state space:

```text
x = [q, t]
```

where:

```text
q = joint configuration of one MoveIt group
t = global time
```

For an ABB arm on a rail:

```text
q = [track, j1, j2, j3, j4, j5, j6]
x = [track, j1, j2, j3, j4, j5, j6, t]
```

For every candidate state `[q_i, t]`, the validity checker:

```text
1. Sets the current robot group to q_i.
2. Queries all higher-priority robot trajectories at time t.
3. Inserts their interpolated joint states into the same RobotState.
4. Checks full PlanningScene collision.
5. Rejects the state if collision occurs.
```

This is the main integration point between PBS and MoveIt.

### 6.2 Why timing is inside the state

If normal MoveIt RRTConnect is used and the path is retimed afterward with IPTP or TOTG, timing is not known during collision checking. Robot B cannot ask:

```text
Where is Robot A at t = 4.2 s?
```

Therefore, timing must be part of the sampled state. The planner should directly produce:

```text
(q_0, t_0)
(q_1, t_1)
(q_2, t_2)
...
(q_n, t_n)
```

This makes the result immediately usable as a space-time reservation.

---

## 7. Time-aware state validity checker

The `TimeAwareStateValidityChecker` checks:

```text
isValid([q, t]) =
    joint bounds valid
    self-collision free
    environment collision free
    collision free against higher-priority robot states at t
    collision free against human / obstacle reservations at t
```

Robot-robot collision checking:

```cpp
for each higher_priority_robot h:
    q_h = trajectory_db.getJointValuesAtTime(h, t);
    combined_state.setJointGroupPositions(my_group, q);
    combined_state.setJointGroupPositions(h_group, q_h);
    planning_scene->checkCollision(req, res, combined_state);
```

Human and Right-of-Way reservations can initially be represented as coarse dynamic collision objects or reserved volumes. The first version does not need to predict human intent perfectly; it only needs to represent human occupancy as high-priority space-time constraints.

---

## 8. Motion validation: velocity-safe RRT edges

Because the trajectory is not postprocessed with IPTP or TOTG, the RRT edges themselves must satisfy joint velocity limits.

A `TimeAwareVelocityMotionValidator` should check each edge:

```text
checkMotion([q1, t1], [q2, t2])
```

Reject the edge if:

```text
dt = t2 - t1 <= 0
```

or if any joint violates:

```text
abs(q2[i] - q1[i]) / dt > vmax[i]
```

Thus every accepted edge satisfies:

```text
|dq_i| / dt ≤ v_max_i
```

Velocity limits should be read from MoveIt:

```cpp
const moveit::core::VariableBounds& vb =
    robot_model->getVariableBounds(var_name);

v_limits[i] = vb.max_velocity_;
```

### 8.1 Acceleration handling

The first version is not fully kinodynamic. The best simple strategy is:

```text
1. Enforce hard per-edge velocity limits.
2. Limit rrt_range.
3. Enforce min_dt.
4. Estimate acceleration after planning.
5. Reject and replan if acceleration spikes are too high.
```

Acceleration estimate:

```text
v_k = (q_k - q_{k-1}) / (t_k - t_{k-1})

a_k = (v_k - v_{k-1}) / (t_k - t_{k-1})
```

If:

```text
abs(a_k[i]) > a_max[i]
```

then the trajectory can be rejected and replanned with:

```text
smaller rrt_range
larger min_dt
larger t_goal
```

This is not true kinodynamic planning, but it is a practical and defensible engineering compromise for ROS1 MoveIt + OMPL.

---

## 9. MultiRobotTrajectoryDB / reservation table

The package needs a central trajectory database:

```cpp
struct TimedJointWaypoint
{
    double t;
    std::vector<double> joints;
};

using Trajectory = std::vector<TimedJointWaypoint>;
```

Core API:

```cpp
void addTrajectory(const std::string& group_name,
                   const Trajectory& trajectory);

bool getJointValuesAtTime(const std::string& group_name,
                          double t,
                          std::vector<double>& out_joints) const;

bool hasTrajectory(const std::string& group_name) const;

double getStartTime(const std::string& group_name) const;
double getEndTime(const std::string& group_name) const;
```

This database is used by:

```text
PBS conflict detection
single-agent RRT state validity checking
human/right-of-way reservation checking
RViz visualization
execution monitoring
Type-B costmap reservation export
```

Once a Type-A trajectory is approved, it becomes a fixed reservation for other agents.

---

## 10. Conflict detection in PBS

PBS detects conflicts between two timed trajectories:

```text
τ_i(t), τ_j(t)
```

It samples time:

```text
t = window_start : dt_conflict_check : window_end
```

At each sampled time:

```text
q_i(t) = interpolate τ_i
q_j(t) = interpolate τ_j
```

Then it checks:

```text
set both groups in one RobotState
planning_scene->checkCollision()
```

If collision occurs:

```cpp
struct Conflict
{
    int a1;
    int a2;
    double t;
    std::string type; // ROBOT_ROBOT, ROBOT_HUMAN, ROBOT_OBJECT
};
```

PBS then branches:

```text
a1 has higher priority than a2
a2 has higher priority than a1
```

In this package, priority does not only mean earlier start time. It means the lower-priority robot must plan around the already planned trajectory of the higher-priority robot.

---

## 11. Right-of-Way integration

Right-of-Way enters the package in three places.

### 11.1 Priority initialization

The default priority hierarchy should be:

```text
Human > Type-A moving with payload > Type-A idle > Type-B mobile cobot
```

This initializes the priority graph before PBS branching.

### 11.2 Constraint generation

Some conflicts should not branch symmetrically:

```text
robot vs human:
    robot must yield

Type-B vs moving Type-A:
    Type-B must yield

moving Type-A vs idle Type-A:
    idle Type-A should retract if feasible

Type-A vs Type-A both carrying payload:
    PBS may branch, but operator approval may be required
```

### 11.3 Execution repair

The package should expose repair hooks:

```cpp
pauseAgent(agent_id);
delayTrajectory(agent_id, dt);
replanAgentAgainstCurrentReservations(agent_id);
replanWindow(window_id);
insertRetractTask(agent_id);
```

These support runtime response to delay, human interruption, and dynamic changes in the scene.

---

## 12. ROS1 / MoveIt Noetic integration

### 12.1 Dependencies

Core dependencies:

```text
roscpp
moveit_core
moveit_ros_planning
moveit_ros_planning_interface
moveit_msgs
trajectory_msgs
control_msgs
actionlib
ompl
tf2_ros
visualization_msgs
```

### 12.2 What this package should avoid modifying

For the first implementation, avoid modifying:

```text
move_group core
MoveIt OMPL plugin
OMPL source code
robot controller drivers
```

The cleanest implementation is a separate C++ ROS node that uses:

```text
robot_model_loader::RobotModelLoader
planning_scene_monitor::PlanningSceneMonitor
planning_scene::PlanningScene
OMPL SimpleSetup / RRTConnect
MoveIt RobotTrajectory conversion
MoveGroupInterface or FollowJointTrajectory action clients
```

### 12.3 Suggested nodes

```text
centralized_mapf_node
    Owns RHCR + PBS + low-level planning.

trajectory_execution_node
    Optional. Dispatches trajectories to controllers.

reservation_visualizer_node
    Publishes RViz markers for planned space-time trajectories.

execution_monitor_node
    Monitors planned-vs-actual timing, human interruption, and delay.
```

For the first demo, these can be combined into:

```text
centralized_pbs_demo_node
```

---

## 13. Recommended package structure

```text
Centralized_MAPF/
  CMakeLists.txt
  package.xml
  README.md

  include/centralized_mapf/
    task_spec.h
    agent_spec.h

    rhcr_coordinator.h
    rhcr_window.h

    pbs.h
    pbs_node.h
    pbs_conflict.h
    priority_graph.h

    multi_robot_trajectory_db.h
    time_aware_single_robot_planner.h
    time_aware_state_validity_checker.h
    time_aware_velocity_motion_validator.h

    trajectory_conversion.h
    execution_monitor.h
    visualization_utils.h

  src/
    rhcr_coordinator.cpp

    pbs.cpp
    pbs_node.cpp
    pbs_conflict.cpp
    priority_graph.cpp

    multi_robot_trajectory_db.cpp
    time_aware_single_robot_planner.cpp
    time_aware_state_validity_checker.cpp
    time_aware_velocity_motion_validator.cpp

    trajectory_conversion.cpp
    centralized_pbs_demo_node.cpp
    centralized_rhcr_demo_node.cpp

  config/
    centralized_mapf_demo.yaml
    robot_groups.yaml
    planner_params.yaml

  launch/
    centralized_pbs_demo.launch
    centralized_rhcr_demo.launch
    visualize_reservations.launch
```

---

## 14. Complete planning cycle

```text
1. Update planning scene
   - current robot joint states
   - known obstacles
   - attached objects
   - human coarse volumes
   - existing reservations

2. RHCR selects a window
   - choose available tasks
   - assign candidate robots
   - set nominal start/end times
   - pass a small problem to PBS

3. PBS generates root node
   - plan each robot independently or in initial priority order
   - store trajectories in trajectory DB
   - detect conflicts

4. PBS expands conflicts
   - select earliest or most severe conflict
   - branch priority relation
   - replan affected lower-priority robot using time-aware RRT

5. Single robot RRTConnect plans in [q, t]
   - sample joint-time states
   - use MoveIt collision checking
   - query higher-priority trajectories at t
   - enforce velocity-safe edges

6. PBS terminates
   - if conflict-free, return trajectories
   - if timeout, return best partial solution or failure

7. Execution manager dispatches
   - convert to JointTrajectory
   - send to controllers
   - publish reservations

8. Monitor
   - check actual progress against planned timestamps
   - pause or repair if human enters reserved region
```

---

## 15. Minimal viable implementation plan

### Stage 1 — Single manipulator `[q, t]` RRT

Goal:

```text
Plan one MoveIt group from q_start to q_goal with time included.
```

Deliverables:

```text
TimeAwareSingleRobotPlanner
TimeAwareStateValidityChecker
TimeAwareVelocityMotionValidator
MultiRobotTrajectoryDB
```

Success condition:

```text
One robot generates a timed trajectory with monotonic t and velocity-safe edges.
```

### Stage 2 — Two manipulators, fixed priority

Goal:

```text
Plan Robot A first, then plan Robot B around A.
```

Success condition:

```text
Robot B queries A(q,t) during validity checking and avoids collision.
```

### Stage 3 — PBS branching

Goal:

```text
If A-first fails or causes conflict, try B-first.
```

Success condition:

```text
PBS can solve a simple two-arm crossing or swap problem.
```

### Stage 4 — RHCR window

Goal:

```text
Select 2–5 tasks from a task graph and send them to PBS.
```

Success condition:

```text
The package plans a small batch from a task graph instead of hard-coded named targets.
```

### Stage 5 — Execution monitor and repair

Goal:

```text
Track actual execution timing and trigger pause/replan if delayed.
```

Success condition:

```text
If one robot is delayed, the system can either delay dependent trajectories or replan the affected lower-priority agent.
```

---

## 16. What should remain outside this package

To keep the package clean, do not include every perception and task-generation module directly inside `Centralized_MAPF`.

Keep these external:

```text
human tracking
YOLO / SAM / skeleton detection
fiducial localization
Husky SLAM
DWA local controller
xArm task-specific manipulation routines
task-graph generation from Rhino/Grasshopper
FEA-based structural sequencing
```

The planning package should consume their outputs:

```text
human volumes
robot poses
available tasks
obstacle meshes
task precedence graph
named targets / joint targets
```

---

## 17. One-sentence positioning

`Centralized_MAPF` is a ROS1 MoveIt-integrated centralized planning package that converts construction task windows into time-consistent, Right-of-Way-aware multi-robot manipulator trajectories by combining RHCR task-window selection, PBS priority branching, and single-agent RRTConnect planning in joint-time space `[q, t]`.
