cd mbz2020_ws
catkin build

source devel/setup.bash

cd src/mbz2020_planner/launch

roslaunch demo_planner.launch --screen
