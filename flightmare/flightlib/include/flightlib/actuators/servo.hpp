#pragma once

#include <iostream>
#include <stdlib.h>
#include <deque>

#include "flightlib/common/math.hpp"
#include "flightlib/common/types.hpp"

namespace flightlib {
class Servo {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Servo(const Scalar c, const Scalar offset, const Scalar x_min, const Scalar x_max);
  ~Servo();

  Scalar run(const Scalar sim_dt, const Scalar x_des);
  Scalar delay(const Scalar sim_dt, const Scalar x_des);

  Scalar outputToCurrent(const Scalar u) const;
  inline Scalar getState(){return x_cur_;}  // offset not included because I don't want to show this in unity
  inline Scalar getConsumption(){return consumption_;}
  bool setState(const Scalar x);
  bool reset(const Scalar c, const Scalar offset);

 private:
  Scalar x_min_;
  Scalar x_max_;
  Scalar x_std_;
  Scalar x_mean_;
  Scalar c_;
  Scalar offset_;
  Scalar x_cur_;
  Scalar x_vel_norm_;
  Scalar x_int_norm_;
  Scalar err_prev_norm_;
  Scalar consumption_;

  //delay
  Scalar delay_;
  std::deque<Scalar> commands_;
  std::deque<Scalar> sim_dts_;

  //PID controller
  Scalar p_, i_, d_, i_sat_, v_sat_, u_sat_;
  //YAML::Node cfg_;
};

}  // namespace flightlib
