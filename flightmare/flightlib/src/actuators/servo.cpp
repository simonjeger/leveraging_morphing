
#include "flightlib/actuators/servo.hpp"

namespace flightlib {

Servo::Servo(const Scalar c, const Scalar offset, const Scalar x_min, const Scalar x_max) {
  x_min_ = x_min;
  x_max_ = x_max;
  x_mean_ = (x_max_ + x_min_) / 2.0;
  x_std_ = (x_max_ - x_min_) / 2.0;

  // Following data is fit to the KST X08 V6.0 8mm HV servo
  p_ = 640.8632177492942;
  i_ = 0.0;
  d_ = 34.17905509647496;
  i_sat_ = 10;
  v_sat_ = 6.900620311813582;
  u_sat_ = 161.4372081503928;
  delay_ = 0.050203802954224636;

  reset(c, offset);
}

Servo::~Servo() {}

Scalar Servo::run(const Scalar sim_dt, const Scalar x_des){
  // Delay
  Scalar x_des_del = delay(sim_dt, x_des);

  // To model the servo correctly, we have to normalize it
  Scalar x_cur_norm = (x_cur_ - x_mean_)/x_std_;
  Scalar x_des_norm = (x_des_del - x_mean_)/x_std_;

  // PID servo controller
  Scalar err_norm = x_des_norm - x_cur_norm;
  x_int_norm_ += err_norm*sim_dt;
  
  Scalar u = p_*err_norm  + std::clamp(i_*x_int_norm_,-i_sat_, i_sat_) + d_*(err_norm - err_prev_norm_)/sim_dt;
  u = std::clamp(u,-u_sat_,u_sat_);
  consumption_ = outputToCurrent(u)*7; // servos are powered with 7 Volts

  x_vel_norm_ += u*c_*sim_dt;
  x_vel_norm_ = std::clamp(x_vel_norm_, -v_sat_, v_sat_);
  x_cur_norm += x_vel_norm_*sim_dt;

  err_prev_norm_ = err_norm;

  x_cur_ = x_cur_norm*x_std_ + x_mean_;

  // add offset
  return x_cur_ + offset_*x_std_;
}

Scalar Servo::delay(const Scalar sim_dt, const Scalar x_des){
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

Scalar Servo::outputToCurrent(const Scalar u) const {
  return abs(u)/u_sat_;
}

bool Servo::setState(const Scalar x){
  reset(c_, offset_); // reset the servo to the current position
  x_cur_ = x;
  return true;
}

bool Servo::reset(const Scalar c, const Scalar offset){
  sim_dts_.clear();
  commands_.clear();
  c_ = c;
  offset_ = offset;
  x_cur_ = x_mean_;
  x_vel_norm_ = 0;
  err_prev_norm_ = 0;
  x_int_norm_ = 0;
  consumption_ = 0;
  return true;
}

}  // namespace flightlib