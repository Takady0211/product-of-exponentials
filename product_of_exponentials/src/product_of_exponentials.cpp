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

#include <rclcpp_components/register_node_macro.hpp>

#define DEBUG false

namespace product_of_exponentials
{

POE::POE(const rclcpp::NodeOptions & options) : rclcpp::Node("product_of_exponentials", options)
{
  std::cout << "POE class is established." << std::endl;

  vis_ = new Visualize();
}

POE::~POE()
{
  std::cout << "POE class is destructed." << std::endl;

  delete vis_;
}


Eigen::Isometry3d POE::setZeroPosition(const Eigen::Vector3d & translation, const Eigen::Matrix3d & rotation)
{
  // Reference: Eq. (4.14) in Modern Robotics
  Eigen::Isometry3d M = Eigen::Isometry3d::Identity();

  M.translate(translation);
  M.rotate(rotation);

  return M;
}

std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>>
POE::getScrewAxesInBodyFrame()
{
  // Reference: Eq. (4.16) in Modern Robotics
  // [B] is given by M^-1 * [S] * M (i.e. B = [Ad_{M^-1}] * S)
  std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> S =
    getLimberoLimbScrewAxes();
  Eigen::Vector3d v(1, 1, 1);  // TENTATIVE
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();  // TENTATIVE
  Eigen::Isometry3d M = setZeroPosition(v, R);
#if DEBUG
  std::cout << "M = " << std::endl << M.matrix() << std::endl;
  std::cout << "M.inverse() = " << std::endl << M.inverse().matrix() << std::endl;
#endif  // DEBUG

  std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> B(S.size());
  for (uint i = 0; i < S.size(); ++i) {
    auto skew_B = M.inverse().matrix() * calculateMatrixRepresentation(S.at(i)) * M.matrix();
    B.at(i) = calculateVectorRepresentation(skew_B);
  }
  return B;
}

Eigen::Vector6d POE::calculateTwist(const Eigen::Vector3d & omega, const Eigen::Vector3d & point)
{
  // Reference: Definition 3.24 in Modern Robotics
  Eigen::Vector6d twist;

  twist.head(3) = omega;
  twist.tail(3) = -omega.cross(point);  //  v = -omega x point

  return twist;
}

Eigen::Matrix3d POE::calculateSkew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d skew_v;

  skew_v.row(0) << 0.0, -v(2), v(1);
  skew_v.row(1) << v(2), 0.0, -v(0);
  skew_v.row(2) << -v(1), v(0), 0.0;

  return skew_v;
}

Eigen::Vector3d POE::calculateSkewInverse(const Eigen::Matrix3d & skew_v)
{
  Eigen::Vector3d v(skew_v(2, 1), skew_v(0, 2), skew_v(1, 0));
  return v;
}

Eigen::Matrix4d POE::calculateMatrixRepresentation(const Eigen::Vector6d & twist)
{
  // Reference: Equation (3.74) in Modern Robotics
  Eigen::Matrix4d twist_matrix = Eigen::Matrix4d::Zero();

  // [omega_s] and v_s
  twist_matrix.block(0, 0, 3, 3) = calculateSkew(twist.head(3));
  twist_matrix.block(0, 3, 3, 1) = twist.tail(3);

  return twist_matrix;
}

Eigen::Vector6d POE::calculateVectorRepresentation(const Eigen::Matrix4d & twist_matrix)
{
  Eigen::Vector6d twist;

  // omega_s and v_s
  twist.head(3) = calculateSkewInverse(twist_matrix.block(0, 0, 3, 3));
  twist.tail(3) = twist_matrix.block(0, 3, 3, 1);

  return twist;
}

Eigen::MatrixXd POE::calculateJacobian(const sensor_msgs::msg::JointState & joint_position)
{
  // Reference:
  // Equation (5.6) in Modern Robotics and
  // Eq. (3.54) in A Mathmatical Introduction to Robotic Manipulation
  auto S = getLimberoLimbScrewAxes();
  std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> J(6);
  Eigen::Matrix<double, 6, 6> Jacobian;
  for (uint i = 0; i < joint_position.position.size(); ++i) {
    Jacobian.col(i) = calculateAdjointMapping(joint_position, S, i);
  }
  return Jacobian;
}

Eigen::MatrixXd POE::calculateBodyJacobian(const sensor_msgs::msg::JointState & joint_position)
{
  // Reference: Equation (5.13) in Modern Robotics
  auto B = getScrewAxesInBodyFrame();

#if DEBUG
  std::cout << "calculateBodyJacobian()" << std::endl;
  std::cout << "B.size() = " << B.size() << std::endl;
  for (uint i = 0; i < B.size(); ++i) {
    std::cout << "B.at(" << i << ") = " << std::endl << B.at(i) << std::endl;
  }
#endif  // DEBUG

  std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> J_b(6);
  Eigen::Matrix<double, 6, 6> Jacobian_b;
  for (uint i = 0; i < J_b.size(); ++i) {
    Jacobian_b.col(i) = calculateAdjointMappingInBodyFrame(joint_position, B, i);
  }
  return Jacobian_b;
}

double POE::calculateManipulability(const sensor_msgs::msg::JointState & joint_position)
{
  Eigen::MatrixXd J = calculateJacobian(joint_position);
  Eigen::MatrixXd A = J * J.transpose();

#if DEBUG
  std::cout << "J = " << std::endl << J << std::endl;
  std::cout << "J.determinant() = " << std::endl << J.determinant() << std::endl;
  std::cout << "A = " << std::endl << A << std::endl;
  std::cout << "A.determinant() = " << std::endl << A.determinant() << std::endl;
#endif  // DEBUG
  double manipulability = sqrt(A.determinant());
  return manipulability;
}

Eigen::Vector6d POE::calculateAdjointMapping(
  const sensor_msgs::msg::JointState & joint_position,
  const std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> & screw_axis,
  const int & screw_axis_index)
{
  Eigen::Matrix4d adjoint_mapping_matrix;
  Eigen::Vector6d adjoint_mapping_vector;

  // Eq. (5.6) of Modern Robotics
  // i-th adjoint mapping matrix =
  // exp([S_1] * theta_1) * ... * exp([S_{i-1}] * theta_{i-1}) * [S_i] *exp([S_{i-1}] * theta_{i-1}) * ... * exp([S_1] * theta_1)
  // TODO(Takady0211): This is hard coding. Can be improved by using recursive function.
  // calculateAdjointMappingInBodyFrame is also.
  if (screw_axis_index == 0) {
    adjoint_mapping_vector = screw_axis.at(0);
  } else if (screw_axis_index == 1) {
    adjoint_mapping_matrix =
      calculateMatrixExponentialSE3(screw_axis.at(0), joint_position.position.at(0)).matrix() *
      calculateMatrixRepresentation(screw_axis.at(screw_axis_index)) *
      calculateMatrixExponentialSE3(screw_axis.at(0), joint_position.position.at(0))
        .inverse()
        .matrix();
    adjoint_mapping_vector = calculateVectorRepresentation(adjoint_mapping_matrix);
  } else if (screw_axis_index == 2) {
    adjoint_mapping_matrix =
      calculateMatrixExponentialSE3(screw_axis.at(0), joint_position.position.at(0)).matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(1), joint_position.position.at(1)).matrix() *
      calculateMatrixRepresentation(screw_axis.at(screw_axis_index)) *
      calculateMatrixExponentialSE3(screw_axis.at(1), joint_position.position.at(1))
        .inverse()
        .matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(0), joint_position.position.at(0))
        .inverse()
        .matrix();
    adjoint_mapping_vector = calculateVectorRepresentation(adjoint_mapping_matrix);
  } else if (screw_axis_index == 3) {
    adjoint_mapping_matrix =
      calculateMatrixExponentialSE3(screw_axis.at(0), joint_position.position.at(0)).matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(1), joint_position.position.at(1)).matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(2), joint_position.position.at(2)).matrix() *
      calculateMatrixRepresentation(screw_axis.at(screw_axis_index)) *
      calculateMatrixExponentialSE3(screw_axis.at(2), joint_position.position.at(2))
        .inverse()
        .matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(1), joint_position.position.at(1))
        .inverse()
        .matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(0), joint_position.position.at(0))
        .inverse()
        .matrix();
    adjoint_mapping_vector = calculateVectorRepresentation(adjoint_mapping_matrix);
  } else if (screw_axis_index == 4) {
    adjoint_mapping_matrix =
      calculateMatrixExponentialSE3(screw_axis.at(0), joint_position.position.at(0)).matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(1), joint_position.position.at(1)).matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(2), joint_position.position.at(2)).matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(3), joint_position.position.at(3)).matrix() *
      calculateMatrixRepresentation(screw_axis.at(screw_axis_index)) *
      calculateMatrixExponentialSE3(screw_axis.at(3), joint_position.position.at(3))
        .inverse()
        .matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(2), joint_position.position.at(2))
        .inverse()
        .matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(1), joint_position.position.at(1))
        .inverse()
        .matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(0), joint_position.position.at(0))
        .inverse()
        .matrix();
    adjoint_mapping_vector = calculateVectorRepresentation(adjoint_mapping_matrix);
  } else if (screw_axis_index == 5) {
    adjoint_mapping_matrix =
      calculateMatrixExponentialSE3(screw_axis.at(0), joint_position.position.at(0)).matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(1), joint_position.position.at(1)).matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(2), joint_position.position.at(2)).matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(3), joint_position.position.at(3)).matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(4), joint_position.position.at(4)).matrix() *
      calculateMatrixRepresentation(screw_axis.at(screw_axis_index)) *
      calculateMatrixExponentialSE3(screw_axis.at(4), joint_position.position.at(4))
        .inverse()
        .matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(3), joint_position.position.at(3))
        .inverse()
        .matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(2), joint_position.position.at(2))
        .inverse()
        .matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(1), joint_position.position.at(1))
        .inverse()
        .matrix() *
      calculateMatrixExponentialSE3(screw_axis.at(0), joint_position.position.at(0))
        .inverse()
        .matrix();
    adjoint_mapping_vector = calculateVectorRepresentation(adjoint_mapping_matrix);
  }
  return adjoint_mapping_vector;
}

Eigen::Vector6d POE::calculateAdjointMappingInBodyFrame(
  const sensor_msgs::msg::JointState & joint_position,
  const std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> & B,
  const int & screw_axis_index)
{
  Eigen::Matrix4d adjoint_mapping_matrix;
  Eigen::Vector6d adjoint_mapping_vector;
#if DEBUG
  std::cout << "calculateAdjointMappingInBodyFrame()" << std::endl;
  std::cout << "screw_axis_index = " << screw_axis_index << std::endl;
#endif  // DEBUG

  // Reference: Eq. (5.13) of Modern Robotics
  if (screw_axis_index == 5) {
    adjoint_mapping_vector = B.at(5);
  } else if (screw_axis_index == 4) {
    adjoint_mapping_matrix =
      calculateMatrixExponentialSE3(B.at(5), joint_position.position.at(5)).inverse().matrix() *
      calculateMatrixRepresentation(B.at(4)) *
      calculateMatrixExponentialSE3(B.at(5), joint_position.position.at(5)).matrix();
    adjoint_mapping_vector = calculateVectorRepresentation(adjoint_mapping_matrix);
  } else if (screw_axis_index == 3) {
    adjoint_mapping_matrix =
      calculateMatrixExponentialSE3(B.at(5), joint_position.position.at(5)).inverse().matrix() *
      calculateMatrixExponentialSE3(B.at(4), joint_position.position.at(4)).inverse().matrix() *
      calculateMatrixRepresentation(B.at(3)) *
      calculateMatrixExponentialSE3(B.at(4), joint_position.position.at(4)).matrix() *
      calculateMatrixExponentialSE3(B.at(5), joint_position.position.at(5)).matrix();
    adjoint_mapping_vector = calculateVectorRepresentation(adjoint_mapping_matrix);
  } else if (screw_axis_index == 2) {
    adjoint_mapping_matrix =
      calculateMatrixExponentialSE3(B.at(5), joint_position.position.at(5)).inverse().matrix() *
      calculateMatrixExponentialSE3(B.at(4), joint_position.position.at(4)).inverse().matrix() *
      calculateMatrixExponentialSE3(B.at(3), joint_position.position.at(3)).inverse().matrix() *
      calculateMatrixRepresentation(B.at(2)) *
      calculateMatrixExponentialSE3(B.at(3), joint_position.position.at(3)).matrix() *
      calculateMatrixExponentialSE3(B.at(4), joint_position.position.at(4)).matrix() *
      calculateMatrixExponentialSE3(B.at(5), joint_position.position.at(5)).matrix();
    adjoint_mapping_vector = calculateVectorRepresentation(adjoint_mapping_matrix);
  } else if (screw_axis_index == 1) {
    adjoint_mapping_matrix =
      calculateMatrixExponentialSE3(B.at(5), joint_position.position.at(5)).inverse().matrix() *
      calculateMatrixExponentialSE3(B.at(4), joint_position.position.at(4)).inverse().matrix() *
      calculateMatrixExponentialSE3(B.at(3), joint_position.position.at(3)).inverse().matrix() *
      calculateMatrixExponentialSE3(B.at(2), joint_position.position.at(2)).inverse().matrix() *
      calculateMatrixRepresentation(B.at(1)) *
      calculateMatrixExponentialSE3(B.at(2), joint_position.position.at(2)).matrix() *
      calculateMatrixExponentialSE3(B.at(3), joint_position.position.at(3)).matrix() *
      calculateMatrixExponentialSE3(B.at(4), joint_position.position.at(4)).matrix() *
      calculateMatrixExponentialSE3(B.at(5), joint_position.position.at(5)).matrix();
    adjoint_mapping_vector = calculateVectorRepresentation(adjoint_mapping_matrix);
  } else if (screw_axis_index == 0) {
    adjoint_mapping_matrix =
      calculateMatrixExponentialSE3(B.at(5), joint_position.position.at(5)).inverse().matrix() *
      calculateMatrixExponentialSE3(B.at(4), joint_position.position.at(4)).inverse().matrix() *
      calculateMatrixExponentialSE3(B.at(3), joint_position.position.at(3)).inverse().matrix() *
      calculateMatrixExponentialSE3(B.at(2), joint_position.position.at(2)).inverse().matrix() *
      calculateMatrixExponentialSE3(B.at(1), joint_position.position.at(1)).inverse().matrix() *
      calculateMatrixRepresentation(B.at(0)) *
      calculateMatrixExponentialSE3(B.at(1), joint_position.position.at(1)).matrix() *
      calculateMatrixExponentialSE3(B.at(2), joint_position.position.at(2)).matrix() *
      calculateMatrixExponentialSE3(B.at(3), joint_position.position.at(3)).matrix() *
      calculateMatrixExponentialSE3(B.at(4), joint_position.position.at(4)).matrix() *
      calculateMatrixExponentialSE3(B.at(5), joint_position.position.at(5)).matrix();
    adjoint_mapping_vector = calculateVectorRepresentation(adjoint_mapping_matrix);
  }
  return adjoint_mapping_vector;
}

Eigen::Matrix3d POE::calculateMatrixExponentialSO3(
  const Eigen::Vector3d & omega, const double & theta)
{
  // Reference: Proposition 3.11 in Modern Robotics
  Eigen::Matrix3d skew_omega = calculateSkew(omega);
  auto I = Eigen::Matrix3d::Identity();
  // Rodrigues' formula
  Eigen::Matrix3d Rot = I + sin(theta) * skew_omega + (1.0 - cos(theta)) * skew_omega * skew_omega;
  return Rot;
}

Eigen::Isometry3d POE::calculateMatrixExponentialSE3(
  const Eigen::Vector6d & S, const double & theta)
{
  // Reference: Proposition 3.25 in Modern Robotics
  Eigen::Matrix4d matrix_exp = Eigen::Matrix4d::Identity();

  Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
  if (S.head(3).norm() == 0) {
    matrix_exp.block(0, 0, 3, 3) = I;
    matrix_exp.block(0, 3, 3, 1) = S.tail(3) * theta;  // = v * theta
  } else {
    matrix_exp.block(0, 0, 3, 3) = calculateMatrixExponentialSO3(S.head(3), theta);

    Eigen::Matrix3d skew_omega = calculateSkew(S.head(3));
    Eigen::Vector3d u =
      (I * theta + (1 - cos(theta)) * skew_omega + (theta - sin(theta)) * skew_omega * skew_omega) *
      S.tail(3);
    matrix_exp.block(0, 3, 3, 1) = u;
  }

  // Eigen::Matrix4d -> Eigen::Isometry3d
  Eigen::Isometry3d exp;
  const int kMatrixSize = 4;
  for (int row = 0; row < kMatrixSize; ++row) {
    for (int col = 0; col < kMatrixSize; ++col) {
      exp(row, col) = matrix_exp(row, col);
    }
  }
  return exp;
}

std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>>
POE::getLimberoLimbScrewAxes()
{
  std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> S;
  S.resize(joint_num_);

  // Define rotation axes: omega
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> omega(joint_num_);
  omega.at(0) << 0, 0, 1;
  omega.at(1) << 0, 1, 0;
  omega.at(2) << 0, 1, 0;
  omega.at(3) << 0, 1, 0;
  omega.at(4) << 1, 0, 0;
  omega.at(5) << 0, 0, 1;

  // Define points on rotation axes: q
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> q(joint_num_);
  q.at(0) << 0.0, 0.0, 0.0;
  q.at(1) << link_lengths_.at(0), 0.0, 0.0;
  q.at(2) << link_lengths_.at(0) + link_lengths_.at(1), 0.0, 0.0;
  q.at(3) << link_lengths_.at(0) + link_lengths_.at(1) + link_lengths_.at(2), 0.0, 0.0;
  q.at(4) << link_lengths_.at(0) + link_lengths_.at(1) + link_lengths_.at(2) + link_lengths_.at(3), 0.0, 0.0;
  q.at(5) << link_lengths_.at(0) + link_lengths_.at(1) + link_lengths_.at(2) + link_lengths_.at(3) + link_lengths_.at(4), 0.0, 0.0;

  for (int i = 0; i < joint_num_; ++i) {
    S.at(i) = calculateScrewAxis(omega.at(i), q.at(i), 0.0);
  }

  return S;
}

Eigen::Vector6d POE::calculateScrewAxis(
  const Eigen::Vector3d & omega, const Eigen::Vector3d & q, const double & h)
{
  // Definition 3.24 in Modern Robotics
  Eigen::Vector6d S;
  if (fabs(omega.norm() - 1.0) < DBL_EPSILON) {  // If |omega| = 1
    S.head(3) = omega;
    Eigen::Vector3d v = -omega.cross(q) + h * omega;
    S.tail(3) = v;
  } else {
    // TODO(Takady0211): implement pure translation motion case (|v| = 1)
    std::cerr << "Unexpected inputs" << std::endl;
    std::abort();
  }

  return S;
}

void POE::setLinkParameters(const std::vector<double> & link_lengths)
{
  link_lengths_ = link_lengths;
  link_num_ = link_lengths_.size();
  joint_num_ = link_num_;

}

}  // namespace product_of_exponentials

RCLCPP_COMPONENTS_REGISTER_NODE(product_of_exponentials::POE)
