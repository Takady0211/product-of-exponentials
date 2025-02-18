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

#include "product_of_exponentials/rviz_visualize.hpp"

namespace product_of_exponentials
{

Visualize::Visualize() : Node("visualize")
{
  std::cout << "Visualize class is established." << std::endl;

  joy_sub_ = rclcpp::Node::create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10, std::bind(&Visualize::joyCallback, this, std::placeholders::_1));
  twist_pub_ = rclcpp::Node::create_publisher<geometry_msgs::msg::Twist>("/twist", 1);
}

Visualize::~Visualize()
{
  std::cout << "Visualize class is destructed." << std::endl;
}

void Visualize::joyCallback(const sensor_msgs::msg::Joy & msg) const
{
  std::cout << msg.header.stamp.sec << std::endl;

  geometry_msgs::msg::Twist v;

  v.linear.x = msg.axes.at(1);
  v.linear.y = msg.axes.at(0);
  v.linear.z = msg.buttons.at(4) - msg.buttons.at(6);

  twist_pub_->publish(v);
}

}  // namespace product_of_exponentials
