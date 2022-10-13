/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/config.hpp"
#include "rviz_common/visibility_control.hpp"
#include "rviz_common/visibility_control.hpp"
#include "rviz_default_plugins/tools/interaction/interaction_tool.hpp"

#include "goal_tool.h"

namespace rviz_common {

Goal3DTool::Goal3DTool() {
  shortcut_key_ = 'g';

  topic_property_ = new properties::StringProperty("Topic", "goal",
                                       "The topic on which to publish navigation goals.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);
}

void Goal3DTool::onInitialize() {
  Pose3DTool::onInitialize();
  setName("3D Nav Goal");
  updateTopic();
}

void Goal3DTool::updateTopic() {
  pub_ = nh_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_property_->getStdString(), 1);
}

void Goal3DTool::onPoseSet(double x, double y, double z, double theta) {
  /*RCLCPP_WARN(nh_->get_logger(),*/printf( "3D Goal Set");
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
//  tf2::Stamped<tf2::Pose> //todo eric
//      p = tf2::Stamped<tf2::Pose>(tf2::Pose(quat, tf2::Point(x, y, z)), rclcpp::Clock().now(), fixed_frame);
  geometry_msgs::msg::PoseStamped goal;
//  tf2::poseStampedTFToMsg(p, goal);//todo eric
  /*RCLCPP_INFO(nh_->get_logger(),*/printf(
              "Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n",
              fixed_frame.c_str(),
              goal.pose.position.x,
              goal.pose.position.y,
              goal.pose.position.z,
              goal.pose.orientation.x,
              goal.pose.orientation.y,
              goal.pose.orientation.z,
              goal.pose.orientation.w,
              theta);
  pub_->publish(goal);
}

} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_common::Goal3DTool, rviz_common::Tool)