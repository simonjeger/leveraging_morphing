#include "flightlib/common/robot_state.hpp"

namespace flightlib {

RobotState::RobotState() {}

RobotState::RobotState(const Vector<IDX::SIZE>& x, const Scalar t)
  : x(x), t(t) {}

RobotState::RobotState(const RobotState& state) : x(state.x), t(state.t) {}

RobotState::~RobotState() {}

Quaternion RobotState::q() const {
  return Quaternion(x(ATTW), x(ATTX), x(ATTY), x(ATTZ));
}

void RobotState::q(const Quaternion quaternion) {
  x(IDX::ATTW) = quaternion.w();
  x(IDX::ATTX) = quaternion.x();
  x(IDX::ATTY) = quaternion.y();
  x(IDX::ATTZ) = quaternion.z();
}

Vector<3> RobotState::e() const {
  // Converts quaternion attitude to euler angles (Tait-Bryan ZYX convention)
  Scalar sy = std::sqrt(R()(0,0)*R()(0,0) + R()(1,0)*R()(1,0));
  Scalar roll = std::atan2(R()(2,1), R()(2,2));
  Scalar pitch = std::atan2(-R()(2,0), sy);
  Scalar yaw = std::atan2(R()(1,0), R()(0,0));
  return Vector<3>(roll, pitch, yaw);
}

Matrix<3, 3> RobotState::R() const {
  return Quaternion(x(ATTW), x(ATTX), x(ATTY), x(ATTZ)).toRotationMatrix();
}

void RobotState::setZero() {
  modelInRange = true;
  t = 0.0;
  x.setZero();
  x(ATTW) = 1.0;
}

Matrix<3, Dynamic> RobotState::moveEuclWorldToEuclBodyFrame(
  Matrix<3, Dynamic> points) {
  Matrix<3, Dynamic> residual = (points).colwise() - p;
  return R().transpose() * residual;
}

Matrix<3, Dynamic> RobotState::moveEuclWorldToPolarBodyFrame(
  Matrix<3, Dynamic> points) {
  Matrix<3, Dynamic> eucl = moveEuclWorldToEuclBodyFrame(points);
  Matrix<3, Dynamic> result = eucl;
  for (int i = 0; i < eucl.cols(); i++) {
    result(0, i) = eucl.block<3, 1>(0, i).norm();
    result(1, i) = atan2(eucl(1, i), eucl(0, i));
    result(2, i) = atan2(eucl(2, i), eucl.block<2, 1>(0, i).norm());
  }
  return result;
}

Matrix<3, Dynamic> RobotState::rotEuclBodyToEuclWorldFrame(
  Matrix<3, Dynamic> directions) {
  return R() * directions;
}

Matrix<3, Dynamic> RobotState::rotEuclWorldToEuclBodyFrame(
  Matrix<3, Dynamic> directions) {
  return R().transpose() * directions;
}

Matrix<3, Dynamic> RobotState::rotEuclWorldToNEDWorldFrame(
  Matrix<3, Dynamic> directions) {
  // return rotEul(M_PI, 0, 0) * directions; //the following is nummerically better
  Matrix<3, 3> R_rot;
  R_rot << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;
  return R_rot * directions;
}
Matrix<3, Dynamic> RobotState::rotNEDWorldToEuclWorldFrame(
  Matrix<3, Dynamic> directions) {
  // return rotEul(-M_PI, 0, 0) * directions; //the following is nummerically better
  Matrix<3, 3> R_rot;
  R_rot << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;
  return R_rot * directions;
}

Matrix<3, Dynamic> RobotState::rotEuclWorldToFRDBodyFrame(
  Matrix<3, Dynamic> directions) {
  Matrix<3, Dynamic> eucl = rotEuclWorldToEuclBodyFrame(directions);
  return rotEuclBodyToFRDBodyFrame(eucl);
}

Matrix<3, Dynamic> RobotState::rotFRDBodyToEuclWorldFrame(
  Matrix<3, Dynamic> directions) {
  Matrix<3,Dynamic> eucl = rotFRDBodyToEuclBodyFrame(directions);
  return rotEuclBodyToEuclWorldFrame(eucl);
}

Matrix<3, Dynamic> RobotState::rotEuclBodyToFRDBodyFrame(
  Matrix<3, Dynamic> directions) {
  // return rotEul(M_PI, 0, 0) * directions; //the following is nummerically better
  Matrix<3, 3> R_rot;
  R_rot << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;
  return R_rot * directions;
}

Matrix<3, Dynamic> RobotState::rotFRDBodyToEuclBodyFrame(
  Matrix<3, Dynamic> directions) {
  // return rotEul(-M_PI, 0, 0) * directions; //the following is nummerically better
  Matrix<3, 3> R_rot;
  R_rot << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;
  return R_rot * directions;
}

Matrix<3, 3> RobotState::rotEul(Scalar alpha, Scalar beta, Scalar gamma) {
  Matrix<3, 3> rot;
  rot << cos(beta) * cos(gamma),
    sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma),
    cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma),
    cos(beta) * sin(gamma),
    sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma),
    cos(alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma), -sin(beta),
    sin(alpha) * cos(beta), cos(alpha) * cos(beta);
  return rot;
}

std::ostream& operator<<(std::ostream& os, const RobotState& state) {
  os.precision(3);
  os << "State at " << state.t << "s: [" << state.x.transpose() << "]";
  os.precision();
  return os;
}

}  // namespace flightlib