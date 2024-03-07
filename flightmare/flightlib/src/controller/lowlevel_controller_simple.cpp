#include "flightlib/controller/lowlevel_controller_simple.hpp"

namespace flightlib {

LowLevelControllerSimple::LowLevelControllerSimple(const YAML::Node &cfg_node) {
  cfg_ = cfg_node;
}

bool LowLevelControllerSimple::reset(RobotState state) {
  // not needed for this ll-controller
  return true;
}

bool LowLevelControllerSimple::setRobotDynamics(
  const std::shared_ptr<DynamicsBase> dynamics_ptr,  std::shared_ptr<IntegratorBase> integrator_ptr) {
  dynamics_ptr_ = dynamics_ptr;
  B_allocation_ = dynamics_ptr->getAllocationMatrix();
  B_allocation_inv_ = B_allocation_.inverse();
  return true;
}

bool LowLevelControllerSimple::setOffset(const Vector<Dynamic> offset) {
  // not needed for this ll-controller
  return true;
}

bool LowLevelControllerSimple::updateRobotDynamics(const RobotState state, const Command cmd, Vector<3> wind_pred) {
  cmd_ = cmd;
  return true;
}

Vector<Dynamic> LowLevelControllerSimple::run_ctrl(RobotState state) {
  //some passed arguments are not used in this controller, but needed for others in the same baseclass
  
  Vector<Dynamic> motor_thrust;
  if (!cmd_.isRaw()) {
    const Scalar force = dynamics_ptr_->getMass() * cmd_.getCollectiveThrust();
    const Vector<3> omega_err = cmd_.getOmega() - state.w;
    const Vector<3> body_torque_des =
      dynamics_ptr_->getJ() * Kinv_ang_vel_tau_ * omega_err +
      state.w.cross(dynamics_ptr_->getJ() * state.w);

    const Vector<4> thrust_torque(force, body_torque_des.x(),
                                  body_torque_des.y(), body_torque_des.z());

    motor_thrust = B_allocation_inv_ * thrust_torque;
  } else {
    motor_thrust = cmd_.getRaw();
  }

  motor_thrust = dynamics_ptr_->clampRaw(motor_thrust);
  return motor_thrust;
}

bool LowLevelControllerSimple::setGains(Vector<4> gains, const int idx){
  // not implemented
  return true;
}

bool LowLevelControllerSimple::storeReservoir(){
  // not implemented
  return true;
}

bool LowLevelControllerSimple::restoreReservoir(){
  // not implemented
  return true;
}

bool LowLevelControllerSimple::getInfo(std::map<std::string, Eigen::VectorXd> &info){
  info["example0"] = Eigen::VectorXd::Ones(1);
  info["example1"] = Eigen::VectorXd::Ones(3);
  return true;
}


}  // namespace flightlib