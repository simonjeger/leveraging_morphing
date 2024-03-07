#pragma once

#include <iostream>
#include <memory>

#include "flightlib/common/command.hpp"
#include "flightlib/common/logger.hpp"

#include "flightlib/common/math.hpp"
#include "flightlib/dynamics/dynamics_base.hpp"
#include "flightlib/actuators/motor.hpp"
#include "flightlib/actuators/servo.hpp"

namespace flightlib {

class BixlerDynamics : public DynamicsBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BixlerDynamics(const YAML::Node &cfg_node);
  ~BixlerDynamics();

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
  inline Scalar getMass(void) const { return mass_; };
  inline Matrix<3, 3> getJ(void) const { return J_; };
  Matrix<4, 4> getAllocationMatrix(void) const;
  inline Vector<3> getOmegaMax(void) const { return omega_max_; }
  inline Scalar getVeluMax(void) const { return velu_max_; }
  Vector<Dynamic> getRawMean(void) const;
  Vector<Dynamic> getRawStd(void) const;
  inline Scalar getForceMax(void) const { return force_torque_max_(0); }
  Scalar getConsumption(void) const;
  Vector<Dynamic> getActuator(void) const;

 private:
  // main functions
  bool dState(const Ref<const Vector<RobotState::SIZE>> state,
              Ref<Vector<RobotState::SIZE>> derivative) const;

  // helper functions
  Matrix<3, 3> R_body_wind(const Scalar alpha, const Scalar beta);
  Matrix<3, 3> R_inertial_body(const Scalar phi, const Scalar theta,
                               const Scalar psi);
  Scalar randomize();
  Scalar randomize(Scalar x);
  Matrix<Dynamic,Dynamic> randomize(Matrix<Dynamic,Dynamic> x);

  // auxiliary variables
  YAML::Node cfg_;

  Matrix<3, 4> t_BM_;
  Matrix<4, 4> B_allocation_;

  // motor
  Scalar motor_omega_min_;
  Scalar motor_omega_max_;
  Scalar motor_tau_inv_;

  //
  Vector<4> force_torque_min_;
  Vector<4> force_torque_max_;

  //
  Vector<3> thrust_map_, thrust_map_r_;
  Scalar kappa_;
  Scalar thrust_min_;
  Scalar thrust_max_;
  //
  Scalar servo_c_ail_, servo_c_ail_r_;
  Scalar servo_c_ele_, servo_c_ele_r_;
  Scalar servo_c_rud_, servo_c_rud_r_;

  Scalar motor_o_r_;
  Scalar servo_o_ail_r_;
  Scalar servo_o_ele_r_;
  Scalar servo_o_rud_r_;

  // actuators
  std::shared_ptr<Motor> motor_ptr_;
  std::shared_ptr<Servo> servo_ptr_ail_;
  std::shared_ptr<Servo> servo_ptr_ele_;
  std::shared_ptr<Servo> servo_ptr_rud_;

  // control surfaces
  Scalar ail_min_;
  Scalar ail_max_;
  Scalar ele_min_;
  Scalar ele_max_;
  Scalar rud_min_;
  Scalar rud_max_;
  Scalar collective_thrust_min_;
  Scalar collective_thrust_max_;

  // Robot limits
  Vector<3> omega_max_;
  Scalar velu_max_;

  Matrix<3, 3> J_, J_r_;  // intertia matrix
  Matrix<3, 3> J_inv_, J_inv_r_;

  Scalar mass_, mass_r_;  // mass
  Scalar S_, S_r_;     // wing surface
  Scalar c_, c_r_;     // chord length
  Scalar b_, b_r_;     // wing span

  // aerodynamic constants
  // linearized for alpha = 0 and u = 12 m/s
  Scalar CL0_, CL0_r_;
  Scalar CL_alpha_, CL_alpha_r_;
  Scalar CL_q_, CL_q_r_;
  Scalar CL_del_e_, CL_del_e_r_;
  Scalar CD0_, CD0_r_;
  Scalar CD_alpha_, CD_alpha_r_;
  Scalar CD_q_, CD_q_r_;
  Scalar CD_del_e_, CD_del_e_r_;
  Scalar CY0_, CY0_r_;
  Scalar CY_beta_, CY_beta_r_;
  Scalar CY_p_, CY_p_r_;
  Scalar CY_r_, CY_r_r_;
  Scalar CY_del_a_, CY_del_a_r_;
  Scalar CY_del_r_, CY_del_r_r_;
  Scalar Cl0_, Cl0_r_;
  Scalar Cl_beta_, Cl_beta_r_;
  Scalar Cl_p_, Cl_p_r_;
  Scalar Cl_r_, Cl_r_r_;
  Scalar Cl_del_a_, Cl_del_a_r_;
  Scalar Cl_del_r_, Cl_del_r_r_;
  Scalar Cm0_, Cm0_r_;
  Scalar Cm_alpha_, Cm_alpha_r_;
  Scalar Cm_q_, Cm_q_r_;
  Scalar Cm_del_e_, Cm_del_e_r_;
  Scalar Cn0_, Cn0_r_;
  Scalar Cn_beta_, Cn_beta_r_;
  Scalar Cn_p_, Cn_p_r_;
  Scalar Cn_r_, Cn_r_r_;
  Scalar Cn_del_a_, Cn_del_a_r_;
  Scalar Cn_del_r_, Cn_del_r_r_;
  Scalar epsilon_, epsilon_r_;  // pitch angle of thrust motor in radiant
};

}  // namespace flightlib
