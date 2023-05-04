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

#ifndef TWIST_TO_ODOM__TWIST_TO_ODOM_NODE_HPP_
#define TWIST_TO_ODOM__TWIST_TO_ODOM_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "twist2odom/visibility_control.hpp"


namespace twist2odom
{

class TWIST_TO_ODOM_PUBLIC Twist2OdomNode : public rclcpp::Node
{
public:
  explicit Twist2OdomNode(const rclcpp::NodeOptions & options);
  void twistWithCovarianceCallback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

private:
  bool publish_tf_{false};
  double yaw_{0.0};
  nav_msgs::msg::Odometry odom_;
  rclcpp::Time last_time_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    twist_with_covariance_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
}  // namespace twist2odom

#endif  // TWIST_TO_ODOM__TWIST_TO_ODOM_NODE_HPP_
