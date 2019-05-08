# Final Project for 16-350, Planning Techniques for Robotics
### Andrew Saba (asaba)

The planner is written using ROS kinetic and C++ 11.

## Build and run:
- To build and run, just run one of the two provided shell scripts:
  - `build_and_launch.sh`
  - `build_and_launch_without_obstacles.sh`
- The second disables checking for obstacles and only generates a feasible trajectory, but makes no checks for collisions.

## To Visualize:
- Run RVIZ and add visualization for the following topics: 
  - `/fake_target` topic, which displays the goal trajectory to catch
  - `/path` topic, which is the topic the plan is published to
  - `/fake_map`, which is the topic that the map is published to.

### Note: 
- When visualizing the map, the grey is freespace and the black bar is not free, from 0.0 to the max height, so the planner cannot plan a trajectory over it.
