#pragma once

#include <iostream>
#include <stdlib.h>
#include <deque>

#include "flightlib/common/math.hpp"
#include "flightlib/common/types.hpp"

namespace flightlib {
class Motor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Motor(const Vector<3> thrust_map, const Scalar offset, const Scalar motor_tau_inv, const Scalar delay, const Scalar thrust_min, const Scalar thrust_max, const Scalar motor_omega_min, const Scalar motor_omega_max);
  ~Motor();

  Scalar run(const Scalar sim_dt, const Scalar x_des, const Scalar V);
  inline Scalar getState(){return motor_thrust_;} // offset not included because I don't want to show this in unity
  inline Scalar getConsumption(){return consumption_;}
  bool setState(const Scalar motor_thrust);
  Scalar delay(const Scalar sim_dt, const Scalar x_des);
  bool reset(const Vector<3> thrust_map, const Scalar offset);

 private:
  Scalar motorThrustToOmega(const Scalar thrust) const;
  Scalar motorOmegaToThrust(const Scalar omega) const;
  Scalar motorThrustToCurrent(const Scalar thrust) const;
  Scalar thrustUnderVelocity(const Scalar thrust, const Scalar u) const;
  Scalar motor_thrust_;
  Scalar motor_omega_;
  Scalar motor_tau_inv_;
  Scalar thrust_min_;
  Scalar thrust_max_;
  Vector<3> thrust_map_;
  Scalar motor_omega_min_;
  Scalar motor_omega_max_;
  Scalar consumption_;
  Scalar omega_mean_;
  Scalar omega_std_;
  Scalar thrust_mean_;
  Scalar thrust_std_;
  Scalar offset_;

  //delay
  Scalar delay_;
  std::deque<Scalar> commands_;
  std::deque<Scalar> sim_dts_;
};

}  // namespace flightlib