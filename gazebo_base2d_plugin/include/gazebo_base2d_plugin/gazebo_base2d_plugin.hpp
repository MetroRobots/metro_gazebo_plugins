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

#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <base2d_kinematics/kinematic_parameters.hpp>
#include <base2d_kinematics/noise_model.hpp>
#include <thread>

namespace gazebo_base2d_plugin
{
typedef std::chrono::steady_clock::duration SimClock;

class GazeboBase2DPlugin : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPreUpdate,
                           public gz::sim::ISystemPostUpdate
{
public:
  virtual void Configure(const gz::sim::Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
                         gz::sim::EntityComponentManager& _ecm, gz::sim::EventManager& /*_eventMgr*/) override;

  /// Callback when a velocity command is received.
  /// \param[in] _msg Twist command message.
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);

  void PreUpdate(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) override;
  void PostUpdate(const gz::sim::UpdateInfo& /*_info*/, const gz::sim::EntityComponentManager& _ecm) override;

protected:
  void ros_spin()
  {
    executor_->spin();
  }

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread ros_thread_;

  /// Odometry publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr true_odometry_pub_, odometry_pub_;

  /// To broadcast TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  /// Velocity received on command.
  geometry_msgs::msg::Twist target_cmd_vel_;

  /// Keep latest odometry message
  nav_msgs::msg::Odometry odom_, true_odom_;

  /// Pointer to model.
  gz::sim::Model model_;

  // Pointer to canonical link
  gz::sim::Link canonical_link_;

  /// Update period in seconds.
  double update_period_;

  /// Publish period in seconds.
  double publish_period_;

  /// Last update time.
  SimClock last_update_time_{};

  /// Last publish time.
  SimClock last_publish_time_{};

  /// Odometry frame ID
  std::string odometry_frame_;

  /// Robot base frame ID
  std::string robot_base_frame_;

  /// True to publish odometry messages.
  bool publish_odom_, publish_true_odom_;

  /// True to publish odom-to-world transforms.
  bool publish_odom_tf_;

  base2d_kinematics::KinematicParameters kinematics_;
  std::unique_ptr<base2d_kinematics::NoiseModel> cmd_noise_model_, odom_noise_model_;
  geometry_msgs::msg::Pose2D current_pose_;
  nav_2d_msgs::msg::Twist2D last_cmd_;

  nav_2d_msgs::msg::Twist2D cmd_vel_;
  nav_2d_msgs::msg::Twist2D actual_cmd_vel_;  // with command noise
  nav_2d_msgs::msg::Twist2D odom_cmd_vel_;    // with odom noise

  bool use_actual_velocity_{false};
};
}  // namespace gazebo_base2d_plugin
