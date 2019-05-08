The planner is written using ROS kinetic and C++ 11.

To build and run, just run one of the two provided shell scripts (build_and_launch.sh and build_and_launch_without_obstacles.sh). The second disables checking for obstacles and only generates a feasible trajectory but makes no checks for collisions.

TO VISUALIZE:
- Run RVIZ and add visualization for the /fake_target topic, which displays the goal trajectory to catch, for the /path topic, which is the topic the plan is published to, and /fake_map, which is the topic that the map is published to.

- NOTE: when visualizing the map, the grey is freespace and the black bar is not free, from 0.0 to the max height, so the planner cannot plan a trajectory over it.
