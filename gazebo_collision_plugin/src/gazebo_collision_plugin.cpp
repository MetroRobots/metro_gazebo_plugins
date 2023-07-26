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

#include <gazebo_collision_plugin/gazebo_collision_plugin.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>

namespace gazebo_collision_plugin
{
GazeboCollisionPlugin::GazeboCollisionPlugin()
{
}

GazeboCollisionPlugin::~GazeboCollisionPlugin()
{
    if (gazebo_node_) {
    gazebo_node_->Fini();
  }
  gazebo_node_.reset();
}

void GazeboCollisionPlugin::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  ros_node_ = gazebo_ros::Node::Get(_sdf);
  gazebo_node_ = boost::make_shared<gazebo::transport::Node>();
  gazebo_node_->Init();
  collision_sub_ = gazebo_node_->Subscribe(
      "/gazebo/default/physics/contacts", &GazeboCollisionPlugin::collisionCB, this);
}

void GazeboCollisionPlugin::collisionCB(ConstContactsPtr & _msg)
{
}

GZ_REGISTER_WORLD_PLUGIN(GazeboCollisionPlugin)
}  // namespace gazebo_collision_plugin
