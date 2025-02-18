// Copyright (c) 2025 Kazuki Takada
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

#ifndef PRODUCT_OF_EXPONENTIALS__RVIZ_VISUALIZE_HPP_
#define PRODUCT_OF_EXPONENTIALS__RVIZ_VISUALIZE_HPP_

#include <functional>
#include <iostream>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace product_of_exponentials
{

class Visualize : public rclcpp::Node
{
  public:
    Visualize();
    virtual ~Visualize();

  private:
    void joyCallback(const sensor_msgs::msg::Joy & msg) const;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

}  // namespace product_of_exponentials

#endif  // PRODUCT_OF_EXPONENTIALS__RVIZ_VISUALIZE_HPP_
