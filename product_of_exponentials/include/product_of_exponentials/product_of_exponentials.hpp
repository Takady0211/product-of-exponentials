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

#ifndef PRODUCT_OF_EXPONENTIALS__PRODUCT_OF_EXPONENTIALS_HPP_
#define PRODUCT_OF_EXPONENTIALS__PRODUCT_OF_EXPONENTIALS_HPP_

// clang-format off
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <cfloat>
#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
// clang-format on

#include "product_of_exponentials/visibility_control.h"

namespace Eigen
{
using Vector6d = Eigen::Matrix<double, 6, 1>;
}

namespace product_of_exponentials
{

class POE : public rclcpp::Node
{
public:
  PRODUCT_OF_EXPONENTIALS_PUBLIC
  explicit POE(const rclcpp::NodeOptions & options);

  virtual ~POE();

  /**
   * @brief Get the zero position matrix
   *
   * @return The end-effector configuration in zero (home) position: M in SE(3)
   */
  Eigen::Isometry3d getZeroPosition();

  /**
   * @brief Get the Screw Axes vector that are predefined.
   *
   * @return Screw axes: S
   */
  std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>>
  getLimberoLimbScrewAxes();

  /**
   * @brief Calculate matrix exponential: exp([S] * theta) in R^{4x4}
   *
   * @param S screw axis: S = (omega, v) in R^{1x6}
   * @param theta distance that is traveled along the screw axis
   * @return matrix exponential of the given screw axis and travel distance
   */
  Eigen::Isometry3d calculateMatrixExponentialSE3(
    const Eigen::Vector6d & S, const double & theta);

  /**
   * @brief Calculate space Jacobian matrix (Jacobian in fixed/space frame coordinates)
   *
   * @param joint_position current joint positions
   * @return Jacobian matrix (6x6)
   */
  Eigen::MatrixXd calculateJacobian(const sensor_msgs::msg::JointState & joint_position);

  /**
   * @brief Calculate body Jacobian matrix (Jacobian in end-effector/body frame coordinates)
   *
   * @param joint_position current joint positions
   * @return Jacobian matrix (6x6)
   */
  Eigen::MatrixXd calculateBodyJacobian(const sensor_msgs::msg::JointState & joint_position);

  /**
   * @brief Calculate manipulability scaler value
   *
   * @param joint_position joint angles
   * @return manipulability in R
   */
  double calculateManipulability(const sensor_msgs::msg::JointState & joint_position);

private:
  /**
   * @brief Get the Screw Axes in body frame
   *
   * @return Screw axes in the end-effector/body frame at zero position: B
   */
  std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>>
  getScrewAxesInBodyFrame();

  /**
   * @brief Calculate twist
   *
   * @param omega angular velocity vector in R^{1x3}
   * @param point any point on the axis in R^{1x3}
   * @return twist V = (omega, v) in R^{1x6}
   */
  Eigen::Vector6d calculateTwist(
    const Eigen::Vector3d & omega, const Eigen::Vector3d & point);

  /**
   * @brief Calculate skew symmetric matrix [v] in R^{3x3}
   *
   * @param v 1x3 vector
   * @return 3x3 skew symmetric matrix of input vector in so(3)
   */
  Eigen::Matrix3d calculateSkew(const Eigen::Vector3d & v);

  /**
   * @brief Inverse calculation of calculateSkew
   *
   * @param skew_v 3x3 matrix
   * @return 1x3 vector
   */
  Eigen::Vector3d calculateSkewInverse(const Eigen::Matrix3d & skew_v);

  /**
   * @brief Calculate matrix representation of twist
   *
   * @param twist 1x6 twist vector = (omega, velocity)
   * @return 4x4 matrix representation of input twist in se(3)
   */
  Eigen::Matrix4d calculateMatrixRepresentation(const Eigen::Vector6d & twist);

  /**
   * @brief Inverse calculation of calculateMatrixRepresentation
   *
   * @param twist_matrix 4x4 matrix representation of twist in se(3)
   * @return 1x6 twist vector = (omega, velocity)
   */
  Eigen::Vector6d calculateVectorRepresentation(const Eigen::Matrix4d & twist_matrix);

  /**
   * @brief Calculate adjoint matrix for i-th screw axis
   *
   * @param joint_position current joint positions
   * @param screw_axis S_i = [omega_i; v_i] in R^6
   * @param screw_axis_index index of screw axis
   * @return Eigen::Vector6d adjoint mapping for i-th screw axis
   */
  Eigen::Vector6d calculateAdjointMapping(
    const sensor_msgs::msg::JointState & joint_position,
    const std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> & screw_axis,
    const int & screw_axis_index);

  /**
   * @brief Calculate adjoint matrix for i-th screw axis in body frame
   *
   * @param joint_position current joint positions
   * @param B screw axes in body frame: B_i = [omega_i; v_i] in R^6
   * @param screw_axis_index index of screw axis
   * @return adjoint mapping for i-th screw axis in body frame
   */
  Eigen::Vector6d calculateAdjointMappingInBodyFrame(
    const sensor_msgs::msg::JointState & joint_position,
    const std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> & B,
    const int & screw_axis_index);

  /**
   * @brief Calculate matrix exponential: exp([omega] * theta) in R^{3x3}
   *
   * @param omega rotation axis as unit vector in R^{1x3}
   * @param theta rotation angle
   * @return matrix exponential in SO(3)
   */
  Eigen::Matrix3d calculateMatrixExponentialSO3(
    const Eigen::Vector3d & omega, const double & theta);

  /**
   * @brief Calculate screw axis in the case of rotation
   *
   * @param omega angular velocity in R^{1x3}
   * @param q point on the axis of the screw in R^{1x3}
   * @param h pitch of the screw in R
   * @return screw axis (normalized twist): S in R^{1x6}
   */
  Eigen::Vector6d calculateScrewAxis(
    const Eigen::Vector3d & omega, const Eigen::Vector3d & q, const double & h);

  void setLinkParameters(const std::vector<double> & link_lengths);

  int joint_num_;
  int link_num_;
  std::vector<double> link_lengths_;
};

}  // namespace product_of_exponentials

#endif  // PRODUCT_OF_EXPONENTIALS__PRODUCT_OF_EXPONENTIALS_HPP_
