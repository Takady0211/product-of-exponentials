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

#include "product_of_exponentials/product_of_exponentials.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  const auto product_of_exponentials =
    std::make_shared<product_of_exponentials::POE>(rclcpp::NodeOptions());
  exec.add_node(product_of_exponentials);

  exec.spin();

  rclcpp::shutdown();
}
