Start simu
source ~/simulation_ws/install/setup.bash
ros2 launch turtlebot3_gazebo main_turtlebot3_lab.launch.xml

cd ros2_ws
source install/setup.bash
ros2 launch path_planner_server navigation.launch.py

#cd ros2_ws
#source install/setup.bash
#ros2 run path_planner_server transform

cd ros2_ws
source install/setup.bash
ros2 run path_planner_server rectangle_finder

cd ros2_ws/src
rviz2 -d ./pathplanning_rviz_conf.rviz

cd ros2_ws
source install/setup.bash
ros2 run path_planner_server walk




ros2 run teleop_twist_keyboard teleop_twist_keyboard
colcon build --symlink-install --packages-select path_planner_server
cd ros2_ws
colcon build --symlink-install --packages-select path_planner_interfaces path_planner_server
source install/setup.bash
ros2 run path_planner_server transform
ros2 service call transform_ptp path_planner_interfaces/srv/PointToPoint '{x: 1.0, y: 2.0}'
angle in grads:
ros2 service call transform_gtp path_planner_interfaces/srv/DistanceAndGradToPoint '{angle: 0.0, range: 1.0}'
angle in radians:
ros2 service call transform_rtp path_planner_interfaces/srv/DistanceAndRadToPoint '{angle: 0.0, range: 1.0}'

cd ros2_ws
source install/setup.bash
ros2 launch path_planner_server navigation.launch.py

rviz2 -d ~/ros2_ws/src/pathplanning_rviz_conf.rviz

ros2 run path_planner_server walk
