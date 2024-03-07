#pragma once

#include <iostream>
#include <memory>

#include <string>
// #include <experimental/filesystem>
#include "fdeep/fdeep.hpp"

#include "flightlib/common/command.hpp"
#include "flightlib/common/logger.hpp"

#include "flightlib/common/math.hpp"
#include "flightlib/dynamics/dynamics_base.hpp"
#include "flightlib/actuators/motor.hpp"
#include "flightlib/actuators/servo.hpp"

namespace flightlib {

class mpextra330mlDynamics : public DynamicsBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  mpextra330mlDynamics(const YAML::Node &cfg_node);
  ~mpextra330mlDynamics();

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

  // Past states for ML-prediction
  std::list<fdeep::model> models_;
  Matrix<5,10> states_;
  Scalar pred_std_;
};

}  // namespace flightlib
