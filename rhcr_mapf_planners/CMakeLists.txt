cmake_minimum_required(VERSION 3.0.2)
project(rhcr_mapf_planners)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    xmlrpcpp
    moveit_core
    moveit_ros_planning_interface
    geometry_msgs
    tf2_geometry_msgs
    
)

find_package(PkgConfig REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(ompl REQUIRED)

pkg_check_modules(JSONCPP jsoncpp)

## Specify C++14 standard (ROS Noetic supports C++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

catkin_package(
  CATKIN_DEPENDS roscpp moveit_core moveit_ros_planning_interface moveit_ros_planning geometry_msgs tf2_geometry_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  include
)


# Define the executable for the planner node, including all source files
add_executable(rhcr_planner_node 
  src/rhcr_planner_node.cpp
  src/rhcr_planner.cpp
  src/pbs.cpp
  src/time_aware_rrtconnect.cpp
  src/time_aware_rrt.cpp
)

add_executable(trajectory_visualizer_node
    src/trajectory_visualizer_node.cpp
)
target_link_libraries(rhcr_planner_node 
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)



target_link_libraries(trajectory_visualizer_node
    ${catkin_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    ${OMPL_LIBRARIES}
)



