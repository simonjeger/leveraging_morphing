#pragma once

#include <iostream>
#include <memory>

#include "flightlib/common/command.hpp"
#include "flightlib/common/logger.hpp"

#include "flightlib/common/math.hpp"
#include "flightlib/dynamics/dynamics_base.hpp"
#include "flightlib/actuators/motor.hpp"

namespace flightlib {

class QuadrotorDynamics : public DynamicsBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  QuadrotorDynamics(const YAML::Node &cfg_node, const Scalar mass = 1.0);
  ~QuadrotorDynamics();

  // main functions
  DynamicsFunction getDynamicsFunction() const;
  Vector<7> getAccelerations(RobotState state, const Vector<Dynamic>& cmd_ll,
                             const Scalar sim_dt, Vector<6> wind_curl, bool direct=false);

  // help functions
  bool valid() const;
  bool reset();
  bool randomizeParams();
  bool setModelParams(const Vector<Dynamic>& params);
  bool loadModelParams(const Vector<Dynamic>& params);
  bool setActuator(const Vector<Dynamic>& actuator);

  // Helpers to apply limits.
  Vector<Dynamic> clampRaw(const Vector<Dynamic> raw) const;
  Scalar clampCollectiveThrust(const Scalar thrust) const;
  Vector<Dynamic> clampMotorOmega(const Vector<Dynamic>& omega) const;
  Vector<3> clampTorque(const Vector<3>& torque) const;
  Vector<3> clampBodyrates(const Vector<3>& omega) const;
  Scalar clampVelu(const Scalar vel) const;

  // Get functions
  inline Scalar getMass(void) const { return mass_; }
  inline Matrix<3, 3> getJ(void) const { return J_; }
  Matrix<4, 4> getAllocationMatrix(void) const;
  inline Vector<3> getOmegaMax(void) const { return omega_max_; }
  inline Scalar getVeluMax(void) const { return velu_max_; }
  Vector<Dynamic> getRawMean(void) const;
  Vector<Dynamic> getRawStd(void) const;
  inline Scalar getForceMax(void) const { return force_torque_max_(0); };
  Scalar getConsumption(void) const;
  Vector<Dynamic> getActuator(void) const;

 private:
  // main functions
  bool dState(const Ref<const Vector<RobotState::SIZE>> state,
              Ref<Vector<RobotState::SIZE>> derivative) const;

  // helper functions
  Vector<3> getBodyDrag(const Vector<3>& body_vel);
  Scalar randomize();
  Scalar randomize(Scalar x);
  Matrix<Dynamic,Dynamic> randomize(Matrix<Dynamic,Dynamic> x);

  // auxiliary variables
  YAML::Node cfg_;
  
  Scalar mass_, mass_r_;
  Matrix<3, 4> t_BM_;
  Matrix<4, 4> B_allocation_, B_allocation_r_;
  Matrix<3, 3> J_, J_r_;
  Matrix<3, 3> J_inv_, J_inv_r_;

  // motors
  Scalar motor_omega_min_;
  Scalar motor_omega_max_;
  Scalar motor_tau_inv_;

  // actuators
  std::shared_ptr<Motor> motor_ptr_0_;
  std::shared_ptr<Motor> motor_ptr_1_;
  std::shared_ptr<Motor> motor_ptr_2_;
  std::shared_ptr<Motor> motor_ptr_3_;

  //
  Vector<4> force_torque_min_;
  Vector<4> force_torque_max_;

  // Propellers
  Vector<3> thrust_map_, thrust_map_0_r_, thrust_map_1_r_, thrust_map_2_r_, thrust_map_3_r_;
  Scalar kappa_;
  Scalar thrust_min_;
  Scalar thrust_max_;
  Scalar collective_thrust_min_;
  Scalar collective_thrust_max_;

  Scalar motor_o_0_r_;
  Scalar motor_o_1_r_;
  Scalar motor_o_2_r_;
  Scalar motor_o_3_r_;

  // body drag coefficients
  Vector<3> cd1_, cd1_r_;
  Vector<3> cd3_, cd3_r_;
  Scalar cdz_h_, cdz_h_r_;

  // Robot limits
  Vector<3> omega_max_;
  Scalar velu_max_;
};

}  // namespace flightlib
