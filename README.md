# mhmr-ros-gazebo-simulation
High fidelity simulation of multiple human multiple robots systems using ROS 2 (Humble Hawksbill) LTS and Gazebo Fortress (Ignition Fortress) LTS on Ubuntu 22.04.3 LTS (Jammy Jellyfish)


copy the ros2_ws workspace folder to the home directory, build, and run waypoint navigation program as follows

$ source /opt/ros/humble/setup.bash
$ cd ~ros2_ws/
$ colcon build
$ source install/setup.bash
$ ros2 run uav_waypoint_navigation uav_waypoint_navigation

