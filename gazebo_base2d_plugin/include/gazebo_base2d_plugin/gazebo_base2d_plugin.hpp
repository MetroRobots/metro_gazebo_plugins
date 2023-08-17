// Copyright 2019 Open Source Robotics Foundation, Inc.
//           2023 Metro Robots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Author: David V. Lu!! */

#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <kinematics_2d/kinematic_parameters.hpp>
#include <kinematics_2d/noise_model.hpp>

// Based on https://github.com/ros-simulation/gazebo_ros_pkgs/blob/b6f7bf121d0c607825b65a28b227a5459a71821b/gazebo_plugins/include/gazebo_plugins/gazebo_ros_planar_move.hpp

namespace gazebo_base2d_plugin
{
class GazeboBase2DPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  /// Callback when a velocity command is received.
  /// \param[in] _msg Twist command message.
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);

  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo& _info);

protected:
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  /// Odometry publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr true_odometry_pub_, odometry_pub_;

  /// To broadcast TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  /// Velocity received on command.
  geometry_msgs::msg::Twist target_cmd_vel_;

  /// Keep latest odometry message
  nav_msgs::msg::Odometry odom_, true_odom_;

  /// Pointer to world.
  gazebo::physics::WorldPtr world_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Update period in seconds.
  double update_period_;

  /// Publish period in seconds.
  double publish_period_;

  /// Last update time.
  gazebo::common::Time last_update_time_;

  /// Last publish time.
  gazebo::common::Time last_publish_time_;

  /// Odometry frame ID
  std::string odometry_frame_;

  /// Robot base frame ID
  std::string robot_base_frame_;

  /// True to publish odometry messages.
  bool publish_odom_, publish_true_odom_;

  /// True to publish odom-to-world transforms.
  bool publish_odom_tf_;

  kinematics_2d::KinematicParameters kinematics_;
  std::unique_ptr<kinematics_2d::NoiseModel> cmd_noise_model_, odom_noise_model_;
  geometry_msgs::msg::Pose2D current_pose_;
  nav_2d_msgs::msg::Twist2D last_cmd_;

  bool use_actual_velocity_{false};
};
}  // namespace gazebo_base2d_plugin
