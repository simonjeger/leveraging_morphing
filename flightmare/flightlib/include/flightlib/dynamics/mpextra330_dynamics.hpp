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

class mpextra330Dynamics : public DynamicsBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  mpextra330Dynamics(const YAML::Node &cfg_node);
  ~mpextra330Dynamics();

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

  // flat plate
  Vector<3> flatPlateForce(Vector<3> n_s, const Vector<3> vel, const Scalar s);
  
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

  // flat plates
  Scalar s_topl_, s_topl_r_;
  Scalar s_topr_, s_topr_r_;
  Scalar s_ail_, s_ail_r_;
  Scalar s_ele_, s_ele_r_;
  Scalar s_side_, s_side_r_;
  Scalar s_rud_, s_rud_r_;
  Scalar s_fro_, s_fro_r_;

  Vector<3> pos_topl_, pos_topl_r_;
  Vector<3> pos_topr_, pos_topr_r_;
  Vector<3> pos_aill_, pos_aill_r_;
  Vector<3> pos_ailr_, pos_ailr_r_;
  Vector<3> pos_ele_, pos_ele_r_;
  Vector<3> pos_side_, pos_side_r_;
  Vector<3> pos_rud_, pos_rud_r_;
  Vector<3> pos_fro_, pos_fro_r_;

  // actuator kinematics
  Vector<3> rot_axis_aill_;
  Vector<3> rot_axis_ailr_;
  Vector<3> rot_axis_ele_;
  Vector<3> rot_axis_rud_;

  Vector<3> pos_rot_point_aill_;
  Vector<3> pos_rot_point_ailr_;
  Vector<3> pos_rot_point_ele_;
  Vector<3> pos_rot_point_rud_;

  Scalar dist_rot_axis_aill_;
  Scalar dist_rot_axis_ailr_;
  Scalar dist_rot_axis_ele_;
  Scalar dist_rot_axis_rud_;
};

}  // namespace flightlib
