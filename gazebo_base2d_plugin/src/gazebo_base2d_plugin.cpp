/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Metro Robots
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Metro Robots nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: David V. Lu!! */

#include <gazebo_base2d_plugin/gazebo_base2d_plugin.hpp>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <nav_2d_utils/conversions.hpp>

namespace gazebo_base2d_plugin
{
void GazeboBase2DPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  world_ = model->GetWorld();
  model_ = model;

  // parameters from SDF
  odometry_frame_ = sdf->Get<std::string>("odometry_frame", "odom").first;
  robot_base_frame_ = sdf->Get<std::string>("robot_base_frame", "base_footprint").first;
  auto update_rate = sdf->Get<double>("update_rate", 20.0).first;
  auto publish_rate = sdf->Get<double>("publish_rate", 20.0).first;
  publish_odom_ = sdf->Get<bool>("publish_odom", true).first;
  publish_odom_tf_ = sdf->Get<bool>("publish_odom_tf", true).first;

  // Update rate
  if (update_rate > 0.0)
  {
    update_period_ = 1.0 / update_rate;
  }
  else
  {
    update_period_ = 0.0;
  }
  last_update_time_ = world_->SimTime();

  // Update rate
  if (update_rate > 0.0)
  {
    publish_period_ = 1.0 / publish_rate;
  }
  else
  {
    publish_period_ = 0.0;
  }
  last_publish_time_ = world_->SimTime();

  // ROS Interface
  ros_node_ = gazebo_ros::Node::Get(sdf);
  const gazebo_ros::QoS& qos = ros_node_->get_qos();

  // Advertise odometry topic
  if (publish_odom_)
  {
    odometry_pub_ =
        ros_node_->create_publisher<nav_msgs::msg::Odometry>("odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

    RCLCPP_INFO(ros_node_->get_logger(), "Advertise odometry on [%s]", odometry_pub_->get_topic_name());
  }

  // Broadcast TF
  if (publish_odom_tf_)
  {
    transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);

    RCLCPP_INFO(ros_node_->get_logger(), "Publishing odom transforms between [%s] and [%s]", odometry_frame_.c_str(),
                robot_base_frame_.c_str());
  }

  cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
      std::bind(&GazeboBase2DPlugin::OnCmdVel, this, std::placeholders::_1));

  RCLCPP_INFO(ros_node_->get_logger(), "Subscribed to [%s]", cmd_vel_sub_->get_topic_name());

  // Set header
  odom_.header.frame_id = odometry_frame_;
  odom_.child_frame_id = robot_base_frame_;

  // Listen to the update event (broadcast every simulation iteration)
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboBase2DPlugin::OnUpdate, this, std::placeholders::_1));
}

void GazeboBase2DPlugin::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  target_cmd_vel_ = *_msg;
}

/// Callback to be called at every simulation iteration.
/// \param[in] _info Updated simulation info.
void GazeboBase2DPlugin::OnUpdate(const gazebo::common::UpdateInfo& _info)
{
  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

  std::lock_guard<std::mutex> scoped_lock(lock_);

  ignition::math::Pose3d pose = model_->WorldPose();
  auto yaw = static_cast<float>(pose.Rot().Yaw());
  auto linear = model_->WorldLinearVel();

  // Get Current velocity in proper frame
  nav_2d_msgs::msg::Twist2D current_vel;
  current_vel.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  current_vel.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
  current_vel.theta = model_->WorldAngularVel().Z();

  // Calculate reachable cmd_vel
  nav_2d_msgs::msg::Twist2D target_vel = nav_2d_utils::twist3Dto2D(target_cmd_vel_), actual_cmd_vel = target_vel;

  if (seconds_since_last_update >= update_period_)
  {
    // Convert to global twist
    model_->SetLinearVel(ignition::math::Vector3d(actual_cmd_vel.x * cosf(yaw) - actual_cmd_vel.y * sinf(yaw),
                                                  actual_cmd_vel.y * cosf(yaw) + actual_cmd_vel.x * sinf(yaw), 0));
    model_->SetAngularVel(ignition::math::Vector3d(0, 0, actual_cmd_vel.theta));

    last_update_time_ = _info.simTime;
  }

  if (publish_odom_ || publish_odom_tf_)
  {
    double seconds_since_last_publish = (_info.simTime - last_publish_time_).Double();

    if (seconds_since_last_publish < publish_period_)
    {
      return;
    }

    auto pose = model_->WorldPose();
    odom_.pose.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(pose);
    odom_.twist.twist = nav_2d_utils::twist2Dto3D(current_vel);

    // Set timestamp
    odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_info.simTime);

    if (publish_odom_)
    {
      odometry_pub_->publish(odom_);
    }
    if (publish_odom_tf_)
    {
      geometry_msgs::msg::TransformStamped msg;
      msg.header = odom_.header;
      msg.child_frame_id = robot_base_frame_;
      msg.transform = gazebo_ros::Convert<geometry_msgs::msg::Transform>(odom_.pose.pose);

      transform_broadcaster_->sendTransform(msg);
    }

    last_publish_time_ = _info.simTime;
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboBase2DPlugin)
}  // namespace gazebo_base2d_plugin
