// Copyright 2023 Amadeusz Szymko
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>

#include "twist2odom/twist2odom_node.hpp"

namespace twist2odom
{

Twist2OdomNode::Twist2OdomNode(const rclcpp::NodeOptions & options)
:  Node("twist2odom", options)
{
  publish_tf_ = this->declare_parameter("publish_tf", false);
  odom_.pose.covariance[0] = 0.000036;
  odom_.pose.covariance[7] = 0.000036;
  odom_.pose.covariance[35] = 0.0009;
  twist_with_covariance_sub_ = this->create_subscription<
    geometry_msgs::msg::TwistWithCovarianceStamped>(
    "input/twist_with_covariance", 1,
    std::bind(&Twist2OdomNode::twistWithCovarianceCallback, this, std::placeholders::_1));
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("output/odometry", 1);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void Twist2OdomNode::twistWithCovarianceCallback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  if (last_time_.seconds() == 0) {
    last_time_ = rclcpp::Time(msg->header.stamp);
    return;
  }

  auto current_time = rclcpp::Time(msg->header.stamp);
  double dt = (current_time - last_time_).seconds();
  last_time_ = current_time;
  yaw_ += msg->twist.twist.angular.z * dt;

  odom_.header.stamp = msg->header.stamp;
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base_link";
  odom_.twist = msg->twist;
  odom_.pose.pose.position.x += msg->twist.twist.linear.x * cos(yaw_) * dt;
  odom_.pose.pose.position.y += msg->twist.twist.linear.x * sin(yaw_) * dt;
  odom_.pose.pose.orientation.z = sin(yaw_ / 2);
  odom_.pose.pose.orientation.w = cos(yaw_ / 2);

  odom_pub_->publish(odom_);

  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom_.header;
    transform.child_frame_id = odom_.child_frame_id;
    transform.transform.translation.x = odom_.pose.pose.position.x;
    transform.transform.translation.y = odom_.pose.pose.position.y;
    transform.transform.rotation = odom_.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }
}

}  // namespace twist2odom

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(twist2odom::Twist2OdomNode)
