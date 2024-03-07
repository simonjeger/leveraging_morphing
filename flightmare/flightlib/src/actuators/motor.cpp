
#include "flightlib/actuators/motor.hpp"

namespace flightlib {

Motor::Motor(const Vector<3> thrust_map, const Scalar offset, const Scalar motor_tau_inv, const Scalar delay, const Scalar thrust_min, const Scalar thrust_max, const Scalar motor_omega_min, const Scalar motor_omega_max) {
  motor_tau_inv_ = motor_tau_inv;
  thrust_min_ = thrust_min;
  thrust_max_ = thrust_max;
  thrust_mean_ = (thrust_max_ + thrust_min_)/2.0;
  thrust_std_ = (thrust_max_ - thrust_min_)/2.0;
  motor_omega_min_ = motor_omega_min;
  motor_omega_max_ = motor_omega_max;
  omega_mean_ = (motor_omega_max_ + motor_omega_min_)/2.0;
  omega_std_ = (motor_omega_max_ - motor_omega_min_)/2.0;
  delay_ = delay;
  reset(thrust_map, offset);
}

Motor::~Motor(){}

Scalar Motor::run(const Scalar sim_dt, const Scalar x_des, const Scalar u){
  // Delay
  Scalar x_des_del = delay(sim_dt, x_des);

  const Scalar motor_omega_des = motorThrustToOmega(x_des_del);
  const Scalar motor_omega_clamped = std::clamp(motor_omega_des, motor_omega_min_, motor_omega_max_);

  // simulate motors as a first-order system
  const Scalar c = std::exp(-sim_dt * motor_tau_inv_);
  motor_omega_ = c * motor_omega_ + (1.0 - c) * motor_omega_clamped;
  motor_thrust_ = motorOmegaToThrust(motor_omega_);
  motor_thrust_ = std::clamp(motor_thrust_, thrust_min_, thrust_max_);
  consumption_ = motorThrustToCurrent(motor_thrust_) * 8; // Motor is powered at 8 Volt

  // velocity dependence of the thrust
  motor_thrust_ = thrustUnderVelocity(motor_thrust_, u);

  // add offset
  return motor_thrust_ + offset_*thrust_std_;
}

Scalar Motor::motorThrustToOmega(const Scalar thrust) const {
  const Scalar scale = 1.0 / (2.0 * thrust_map_(0));
  const Scalar offset = -thrust_map_(1) * scale;
  const Scalar root = std::sqrt(std::max(0.0,std::pow(thrust_map_(1), 2.0) - 4.0 * thrust_map_(0) * (thrust_map_(2) - thrust)));

  return offset + scale * root;
}

Scalar Motor::motorOmegaToThrust(const Scalar omega) const {
  const Matrix<1, 3> omega_poly = (Matrix<1, 3>() << std::pow(omega,2.0), omega, 1.0).finished();
  return omega_poly * thrust_map_;
}

Scalar Motor::motorThrustToCurrent(const Scalar thrust) const {
  Scalar c0 = 0.07942750281281408;
  Scalar c1 = 1.6538267237059698;
  Scalar c2 = 0.2939986322634528;
  return abs(c0 + c1*thrust + c2*std::pow(thrust,2.0));
}

Scalar Motor::thrustUnderVelocity(const Scalar thrust, const Scalar u) const {
  Scalar u_c = std::max(0.0, u); //this effect has only been studied for positive u
  Scalar c0 = 0.00598456;
  Scalar c1 = 0.00613726;
  Scalar c2 = 0.00186911;

  return std::max(0.0,thrust - c0*u_c - c1*std::pow(u_c,2.0) - c2*thrust*std::pow(u_c,2.0));
}

Scalar Motor::delay(const Scalar sim_dt, const Scalar x_des){
  sim_dts_.push_front(sim_dt);
  commands_.push_front(x_des);

  Scalar sim_dt_int = 0;
  int index = 0;
  for(std::size_t i=0;i<sim_dts_.size();i++){
    if (sim_dt_int <= delay_){
      sim_dt_int += sim_dts_[i];
      index = i;
    }
    else{
      sim_dts_.pop_back();
      commands_.pop_back();
      break;
    }
  }

  Scalar x_des_del = commands_[index];

  return x_des_del;
}

bool Motor::setState(const Scalar motor_thrust){
  reset(thrust_map_, offset_); // reset the servo to the current position
  motor_thrust_ = motor_thrust;
  motor_thrust_ = std::clamp(motor_thrust_, thrust_min_, thrust_max_);
  return true;
}

bool Motor::reset(const Vector<3> thrust_map, const Scalar offset){
  sim_dts_.clear();
  commands_.clear();
  thrust_map_ = thrust_map;
  motor_thrust_ = thrust_mean_;
  motor_omega_ = omega_mean_;
  offset_ = offset;
  consumption_ = 0;
  return true;
}

}  // namespace flightlib