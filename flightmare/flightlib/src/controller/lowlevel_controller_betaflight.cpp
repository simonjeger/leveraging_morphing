#include "flightlib/controller/lowlevel_controller_betaflight.hpp"

namespace flightlib {

LowLevelControllerBetaflight::LowLevelControllerBetaflight(const YAML::Node &cfg_node)
  : fs(1000), I(fs), D(fs) {
    cfg_ = cfg_node;
  }

LowLevelControllerBetaflight::LowLevelControllerBetaflight(const float fs)
  : fs(fs), I(fs), D(fs) {}

bool LowLevelControllerBetaflight::reset(RobotState state) {
  // not needed for this ll-controller
  return true;
}

bool LowLevelControllerBetaflight::setRobotDynamics(
  const std::shared_ptr<DynamicsBase> dynamics_ptr,  std::shared_ptr<IntegratorBase> integrator_ptr) {
  cum_ = 0.0;
  dynamics_ptr_ = dynamics_ptr;
  B_allocation_ = dynamics_ptr_->getAllocationMatrix();
  B_allocation_inv_ = B_allocation_.inverse();
  return true;
}

bool LowLevelControllerBetaflight::setOffset(const Vector<Dynamic> offset) {
  // not needed for this ll-controller
  return true;
}

bool LowLevelControllerBetaflight::updateRobotDynamics(const RobotState state, const Command cmd, Vector<3> wind_pred) {
  cmd_ = cmd;
  return true;
}

Vector<Dynamic> LowLevelControllerBetaflight::run_ctrl(const RobotState state_gt) {

  //some passed arguments are not used in this controller, but needed for others in the same baseclass
  
  Vector<4> motor_thrust;
  if (!cmd_.isRaw()) {
    const Scalar force = dynamics_ptr_->getMass() * cmd_.getCollectiveThrust();

    const Vector<3> omega_des = cmd_.getOmega();

    const Vector<3> p = P.update(omega_des, state_gt.w);
    // const Vector<3> i = I.update(omega_des, state_gt.w);
    const Vector<3> d = D.update(state_gt.w);
    const Vector<3> body_torque_des = pid_scale * (p + d);


    const Vector<4> tlmn(force, body_torque_des.x(), body_torque_des.y(),
                         body_torque_des.z());
    motor_thrust = B_allocation_inv_ * tlmn;
  } else {
    motor_thrust = cmd_.getRaw();
  }

  motor_thrust = dynamics_ptr_->clampRaw(motor_thrust);
  return motor_thrust;
}

bool LowLevelControllerBetaflight::setGains(Vector<4> gains, const int idx){
  // not implemented
  return true;
}

bool LowLevelControllerBetaflight::storeReservoir(){
  // not implemented
  return true;
}

bool LowLevelControllerBetaflight::restoreReservoir(){
  // not implemented
  return true;
}

bool LowLevelControllerBetaflight::getInfo(std::map<std::string, Eigen::VectorXd> &info){
  info["example0"] = Eigen::VectorXd::Ones(1);
  info["example1"] = Eigen::VectorXd::Ones(3);
  return true;
}

}  // namespace flightlib