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
#include <nav_2d_utils/conversions.hpp>

namespace gazebo_base2d_plugin
{

void GazeboBase2DPlugin::Configure(const gz::sim::Entity& _entity, const std::shared_ptr<const sdf::Element>& sdf,
                                   gz::sim::EntityComponentManager& _ecm, gz::sim::EventManager& /*_eventMgr*/)
{
  model_ = gz::sim::Model(_entity);

  auto canonical_link = model_.CanonicalLink(_ecm);
  if (canonical_link == gz::sim::kNullEntity)
  {
    RCLCPP_ERROR(ros_node_->get_logger(), "Failed to resolve canonical link");
    return;
  }
  canonical_link_ = gz::sim::Link(canonical_link);

  // ROS Interface
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }
  ros_node_ = rclcpp::Node::make_shared("base_2d_plugin");

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  executor_->add_node(ros_node_);

  ros_thread_ = std::thread(&GazeboBase2DPlugin::ros_spin, this);

  // parameters from SDF
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

  // Update rate
  if (update_rate > 0.0)
  {
    publish_period_ = 1.0 / publish_rate;
  }
  else
  {
    publish_period_ = 0.0;
  }

  kinematics_.initialize(ros_node_);
  kinematics_.startSubscriber(ros_node_);

  // Advertise odometry topic
  if (publish_odom_)
  {
    odometry_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>("odom", 1);

    RCLCPP_INFO(ros_node_->get_logger(), "Advertise odometry on [%s]", odometry_pub_->get_topic_name());
  }
  if (publish_true_odom_)
  {
    true_odometry_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>("true_odom", 1);

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
      "cmd_vel", 1, std::bind(&GazeboBase2DPlugin::OnCmdVel, this, std::placeholders::_1));

  RCLCPP_INFO(ros_node_->get_logger(), "Subscribed to [%s]", cmd_vel_sub_->get_topic_name());

  auto covariance_x = sdf->Get<double>("covariance_x", 0.00001).first;
  auto covariance_y = sdf->Get<double>("covariance_y", 0.00001).first;
  auto covariance_yaw = sdf->Get<double>("covariance_yaw", 0.001).first;

  cmd_noise_model_ =
      std::make_unique<base2d_kinematics::NoiseModel>(0.0, covariance_x, 0.0, covariance_y, 0.0, covariance_yaw);
  true_odom_.pose.covariance = cmd_noise_model_->getCovarianceMatrix();
  true_odom_.twist.covariance = true_odom_.pose.covariance;
  true_odom_.header.frame_id = odometry_frame_;
  true_odom_.child_frame_id = robot_base_frame_;

  std::vector<double> odom_params;
  for (const auto& dimension : {"x", "y", "theta"})
  {
    for (const auto& type : {"mean", "covariance"})
    {
      std::string name = std::string("odom_") + type + "_" + dimension;
      odom_params.push_back(sdf->Get<double>(name, 0.0).first);
    }
  }
  odom_noise_model_ = std::make_unique<base2d_kinematics::NoiseModel>(odom_params);
  odom_.pose.covariance = odom_noise_model_->getCovarianceMatrix();
  odom_.twist.covariance = odom_.pose.covariance;
  odom_.header.frame_id = odometry_frame_;
  odom_.child_frame_id = robot_base_frame_;
}

void GazeboBase2DPlugin::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  target_cmd_vel_ = *_msg;
}

void GazeboBase2DPlugin::PreUpdate(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm)
{
  using namespace std::literals;
  double seconds_since_last_update = (_info.simTime - last_update_time_) / 1.0s;

  const gz::math::Pose3d pose = worldPose(model_.Entity(), _ecm);
  auto yaw = static_cast<float>(pose.Rot().Yaw());

  // Get Current velocity in proper frame
  nav_2d_msgs::msg::Twist2D current_vel;
  if (use_actual_velocity_)
  {
    // This method does not work currently.
    // auto linear = model_->WorldLinearVel();
    // current_vel.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    // current_vel.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
    // current_vel.theta = model_->WorldAngularVel().Z();
  }
  else
  {
    current_vel = last_cmd_;
  }

  // Calculate reachable cmd_vel
  nav_2d_msgs::msg::Twist2D target_vel = nav_2d_utils::twist3Dto2D(target_cmd_vel_);
  cmd_vel_ = kinematics_.calculateNewVelocity(target_vel, current_vel, seconds_since_last_update);
  actual_cmd_vel_ = cmd_noise_model_->applyNoise(cmd_vel_);
  odom_cmd_vel_ = odom_noise_model_->applyNoise(cmd_vel_);

  if (seconds_since_last_update >= update_period_)
  {
    geometry_msgs::msg::Pose2D start_pose;
    start_pose.x = pose.Pos().X();
    start_pose.y = pose.Pos().Y();
    start_pose.theta = yaw;

    geometry_msgs::msg::Pose2D new_pose =
        kinematics_.calculateNewPosition(start_pose, actual_cmd_vel_, seconds_since_last_update);

    model_.SetWorldPoseCmd(_ecm, gz::math::Pose3d(new_pose.x, new_pose.y, pose.Pos().Z(), 0.0, 0.0, new_pose.theta));

    current_pose_ = kinematics_.calculateNewPosition(current_pose_, odom_cmd_vel_, seconds_since_last_update);
    last_cmd_ = actual_cmd_vel_;

    last_update_time_ = _info.simTime;
  }
}

void GazeboBase2DPlugin::PostUpdate(const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm)
{
  using namespace std::literals;

  const gz::math::Pose3d pose = worldPose(model_.Entity(), _ecm);

  if (publish_odom_ || publish_odom_tf_ || publish_true_odom_)
  {
    double seconds_since_last_publish = (_info.simTime - last_publish_time_) / 1.0s;

    if (seconds_since_last_publish < publish_period_)
    {
      return;
    }

    odom_.pose.pose = nav_2d_utils::pose2DToPose(current_pose_);
    odom_.twist.twist = nav_2d_utils::twist2Dto3D(odom_cmd_vel_);

    // Set timestamp
    odom_.header.stamp = rclcpp::Time(_info.simTime / 1ns);

    if (publish_odom_)
    {
      odometry_pub_->publish(odom_);
    }
    if (publish_odom_tf_)
    {
      geometry_msgs::msg::TransformStamped msg;
      msg.header = odom_.header;
      msg.child_frame_id = robot_base_frame_;

      msg.transform.translation.x = odom_.pose.pose.position.x;
      msg.transform.translation.y = odom_.pose.pose.position.y;
      msg.transform.translation.z = odom_.pose.pose.position.z;

      msg.transform.rotation.x = odom_.pose.pose.orientation.x;
      msg.transform.rotation.y = odom_.pose.pose.orientation.y;
      msg.transform.rotation.z = odom_.pose.pose.orientation.z;
      msg.transform.rotation.w = odom_.pose.pose.orientation.w;

      transform_broadcaster_->sendTransform(msg);
    }
    if (publish_true_odom_)
    {
      auto gpos = pose.Pos();
      true_odom_.pose.pose.position.x = gpos.X();
      true_odom_.pose.pose.position.y = gpos.Y();
      true_odom_.pose.pose.position.z = gpos.Z();

      auto grot = pose.Rot();
      true_odom_.pose.pose.orientation.x = grot.X();
      true_odom_.pose.pose.orientation.y = grot.Y();
      true_odom_.pose.pose.orientation.z = grot.Z();
      true_odom_.pose.pose.orientation.w = grot.W();

      true_odom_.twist.twist = nav_2d_utils::twist2Dto3D(actual_cmd_vel_);
      true_odom_.header.stamp = odom_.header.stamp;
      true_odometry_pub_->publish(true_odom_);
    }

    last_publish_time_ = _info.simTime;
  }
}
}  // namespace gazebo_base2d_plugin

// Register plugin
GZ_ADD_PLUGIN(gazebo_base2d_plugin::GazeboBase2DPlugin, gz::sim::System,
              gazebo_base2d_plugin::GazeboBase2DPlugin::ISystemConfigure,
              gazebo_base2d_plugin::GazeboBase2DPlugin::ISystemPreUpdate,
              gazebo_base2d_plugin::GazeboBase2DPlugin::ISystemPostUpdate)
