cmake_minimum_required(VERSION 3.8)
project(product_of_exponentials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(product_of_exponentials src/product_of_exponentials.cpp)
rclcpp_components_register_nodes(product_of_exponentials "product_of_exponentials::POE")

ament_auto_add_executable(product_of_exponentials_node
  src/product_of_exponentials_node.cpp
  src/product_of_exponentials.cpp
  src/rviz_visualize.cpp)

# gtest
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(unit_test_product_of_exponentials test/unit_test_product_of_exponentials.cpp)
  target_include_directories(unit_test_product_of_exponentials PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(unit_test_product_of_exponentials product_of_exponentials)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
