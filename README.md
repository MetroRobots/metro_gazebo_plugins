# metro_gazebo_plugins
Useful Gazebo Plugins - Workshop Edition

## gazebo_base2d_plugin

Fork of the standard [`Gazebo Planar Move Plugin`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/foxy/gazebo_plugins/src/gazebo_ros_planar_move.cpp) with a few differences.
 * Enforces the kinematic limits published by [`base2d_kinematics`](https://github.com/MetroRobots/metro_nav/tree/main/base2d_kinematics)
 * Publishes a noisy odometry signal and a true odometry signal
