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

  // ROS Interface
  ros_node_ = gazebo_ros::Node::Get(sdf);
  const gazebo_ros::QoS& qos = ros_node_->get_qos();

  // parameters from SDF
  if (sdf->HasElement("ros"))
  {
    sdf = sdf->GetElement("ros");
  }
  odometry_frame_ = sdf->Get<std::string>("odometry_frame", "odom").first;
  robot_base_frame_ = sdf->Get<std::string>("robot_base_frame", "base_footprint").first;
  auto update_rate = sdf->Get<double>("update_rate", 20.0).first;
  auto publish_rate = sdf->Get<double>("publish_rate", 20.0).first;
  publish_odom_ = sdf->Get<bool>("publish_odom", true).first;
  publish_true_odom_ = sdf->Get<bool>("publish_true_odom", true).first;
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

  kinematics_.initialize(ros_node_);
  kinematics_.startSubscriber(ros_node_);

  // Advertise odometry topic
  if (publish_odom_)
  {
    odometry_pub_ =
        ros_node_->create_publisher<nav_msgs::msg::Odometry>("odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

    RCLCPP_INFO(ros_node_->get_logger(), "Advertise odometry on [%s]", odometry_pub_->get_topic_name());
  }
  if (publish_true_odom_)
  {
    true_odometry_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(
        "true_odom", qos.get_publisher_qos("true_odom", rclcpp::QoS(1)));

    RCLCPP_INFO(ros_node_->get_logger(), "Advertise true odometry on [%s]", true_odometry_pub_->get_topic_name());
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

  auto covariance_x = sdf->Get<double>("covariance_x", 0.00001).first;
  auto covariance_y = sdf->Get<double>("covariance_y", 0.00001).first;
  auto covariance_yaw = sdf->Get<double>("covariance_yaw", 0.001).first;

  cmd_noise_model_ =
      std::make_unique<kinematics_2d::NoiseModel>(0.0, covariance_x, 0.0, covariance_y, 0.0, covariance_yaw);
  true_odom_.pose.covariance = cmd_noise_model_->getCovarianceMatrix();
  true_odom_.twist.covariance = true_odom_.pose.covariance;
  true_odom_.header.frame_id = odometry_frame_;
  true_odom_.child_frame_id = robot_base_frame_;

  std::vector<double> odom_params;
  for (const std::string& dimension : {"x", "y", "theta"})
  {
    for (const std::string& type : {"mean", "covariance"})
    {
      std::string name = "odom_" + type + "_" + dimension;
      odom_params.push_back(sdf->Get<double>(name, 0.0).first);
    }
  }
  odom_noise_model_ = std::make_unique<kinematics_2d::NoiseModel>(odom_params);
  odom_.pose.covariance = odom_noise_model_->getCovarianceMatrix();
  odom_.twist.covariance = odom_.pose.covariance;
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
  if (use_actual_velocity_)
  {
    // This method does not work currently.
    current_vel.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    current_vel.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
    current_vel.theta = model_->WorldAngularVel().Z();
  }
  else
  {
    current_vel = last_cmd_;
  }

  // Calculate reachable cmd_vel
  nav_2d_msgs::msg::Twist2D target_vel = nav_2d_utils::twist3Dto2D(target_cmd_vel_);
  nav_2d_msgs::msg::Twist2D cmd_vel =
      kinematics_.calculateNewVelocity(target_vel, current_vel, seconds_since_last_update);
  nav_2d_msgs::msg::Twist2D actual_cmd_vel = cmd_noise_model_->applyNoise(cmd_vel);
  nav_2d_msgs::msg::Twist2D odom_cmd_vel = odom_noise_model_->applyNoise(cmd_vel);

  if (seconds_since_last_update >= update_period_)
  {
    // Convert to global twist
    model_->SetLinearVel(ignition::math::Vector3d(actual_cmd_vel.x * cosf(yaw) - actual_cmd_vel.y * sinf(yaw),
                                                  actual_cmd_vel.y * cosf(yaw) + actual_cmd_vel.x * sinf(yaw), 0));
    model_->SetAngularVel(ignition::math::Vector3d(0, 0, actual_cmd_vel.theta));
    last_cmd_ = actual_cmd_vel;

    last_update_time_ = _info.simTime;
  }

  if (publish_odom_ || publish_odom_tf_ || publish_true_odom_)
  {
    double seconds_since_last_publish = (_info.simTime - last_publish_time_).Double();

    if (seconds_since_last_publish < publish_period_)
    {
      return;
    }

    current_pose_ = kinematics_.calculateNewPosition(current_pose_, odom_cmd_vel, seconds_since_last_update);

    odom_.pose.pose = nav_2d_utils::pose2DToPose(current_pose_);
    odom_.twist.twist = nav_2d_utils::twist2Dto3D(odom_noise_model_->applyNoise(cmd_vel));

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
    if (publish_true_odom_)
    {
      true_odom_.pose.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(pose);
      true_odom_.twist.twist = nav_2d_utils::twist2Dto3D(actual_cmd_vel);
      true_odom_.header.stamp = odom_.header.stamp;
      true_odometry_pub_->publish(true_odom_);
    }

    last_publish_time_ = _info.simTime;
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboBase2DPlugin)
}  // namespace gazebo_base2d_plugin
