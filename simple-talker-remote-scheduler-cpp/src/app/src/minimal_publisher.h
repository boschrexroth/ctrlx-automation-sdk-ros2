
// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/*
  * @brief The MinimalPublisher class is responsible for publishing messages to a ROS2 topic.
  * It inherits from rclcpp::Node and provides a method for publishing string messages.
*/
class MinimalPublisher : public rclcpp::Node
{
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;

  public:
  MinimalPublisher()
  : Node("minimal_publisher")
  {
    m_publisher = this->create_publisher<std_msgs::msg::String>("ros2_simple_talker_cpp", 10);
  }

  /// @brief This method publishes a string message to the ROS2 topic.
  /// @param input The string message to be published.
  void myprint(const std::string& input)

  {
    auto message = std_msgs::msg::String();
    message.data = "Remote Scheduler value: " + input;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    m_publisher->publish(message);
  }
};
