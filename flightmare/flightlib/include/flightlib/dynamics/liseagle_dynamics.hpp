#pragma once

#include <iostream>
#include <memory>
#include <unsupported/Eigen/CXX11/Tensor>

#include "flightlib/common/command.hpp"
#include "flightlib/common/logger.hpp"
#include "flightlib/common/math.hpp"

#include "flightlib/common/read_csv.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/dynamics/dynamics_base.hpp"
#include "flightlib/actuators/motor.hpp"
#include "flightlib/actuators/servo.hpp"

// debugging
#include <iomanip>

namespace flightlib {

class LiseagleDynamics : public DynamicsBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LiseagleDynamics(const YAML::Node &cfg_node);
  ~LiseagleDynamics();

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
  inline Matrix<3, 3> getJ(void) const { return J_base_; }
  Matrix<4, 4> getAllocationMatrix(void) const;
  inline Vector<3> getOmegaMax(void) const { return omega_max_; }
  inline Scalar getVeluMax(void) const { return velu_max_; }
  Vector<Dynamic> getRawMean(void) const;
  Vector<Dynamic> getRawStd(void) const;
  inline Scalar getForceMax(void) const { return thrust_max_; }
  Scalar getConsumption(void) const;
  Vector<Dynamic> getActuator(void) const;

 private:
  // main functions
  bool dState(const Ref<const Vector<RobotState::SIZE>> state,
              Ref<Vector<RobotState::SIZE>> derivative) const;

  // helper functions
  // Tensor needs to be fixed size otherwise it can't be initialized in header file.
  // This leads to two identical functions (couldn't think of a better way)
  Scalar interp1D(Vector<Dynamic> X, Scalar x_des, Vector<Dynamic> Y, std::string extrap="linear");
  Matrix<Dynamic, Dynamic> interp1D(
    Vector<3> X, Scalar x_des,
    Eigen::TensorFixedSize<Scalar, Eigen::Sizes<3, 3, 3>> Y, std::string extrap="linear");
  Scalar interp2D(Vector<Dynamic> X0, Vector<Dynamic> X1, Scalar x0_des,
                  Scalar x1_des, Matrix<Dynamic, Dynamic> Y, std::string extrap="linear");
  Scalar interp3D(Vector<Dynamic> X0, Vector<Dynamic> X1, Vector<Dynamic> X2,
                  Scalar x0_des, Scalar x1_des, Scalar x2_des,
                  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 2, 4>> Y, std::string extrap="linear");
  Scalar interp3D(Vector<Dynamic> X0, Vector<Dynamic> X1, Vector<Dynamic> X2,
                  Scalar x0_des, Scalar x1_des, Scalar x2_des,
                  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> Y, std::string extrap="linear");
  int closestIdxLow(Vector<Dynamic> X, Scalar x);
  Scalar fitLinEval(Vector<Dynamic> X, Vector<Dynamic> Y, Scalar y);
  Scalar randomizeOffset(Scalar x);
  Matrix<Dynamic,Dynamic> randomizeOffset(Matrix<Dynamic,Dynamic> x);
  Scalar randomize();
  Scalar randomize(Scalar x);
  Matrix<Dynamic,Dynamic> randomize(Matrix<Dynamic,Dynamic> x);
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> randomize(Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> x);
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 2, 4>> randomize(Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 2, 4>> x);
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<3, 3, 3>> randomize(Eigen::TensorFixedSize<Scalar, Eigen::Sizes<3, 3, 3>> x);
  Scalar offset(Scalar x, const Vector<Dynamic>& params);
  Matrix<Dynamic,Dynamic> offset(Matrix<Dynamic,Dynamic> x, const Vector<Dynamic>& params);
  Scalar scale(Scalar x, const Vector<Dynamic>& params);
  Matrix<Dynamic,Dynamic> scale(Matrix<Dynamic,Dynamic> x, const Vector<Dynamic>& params);
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> scale(Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> x, const Vector<Dynamic>& params);
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 2, 4>> scale(Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 2, 4>> x, const Vector<Dynamic>& params);
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<3, 3, 3>> scale(Eigen::TensorFixedSize<Scalar, Eigen::Sizes<3, 3, 3>> x, const Vector<Dynamic>& params);

  // actuators
  std::shared_ptr<Motor> motor_ptr_;
  std::shared_ptr<Servo> servo_ptr_ele_;
  std::shared_ptr<Servo> servo_ptr_rud_;
  std::shared_ptr<Servo> servo_ptr_tswe_;
  std::shared_ptr<Servo> servo_ptr_lswe_;
  std::shared_ptr<Servo> servo_ptr_rswe_;
  std::shared_ptr<Servo> servo_ptr_ltwi_;
  std::shared_ptr<Servo> servo_ptr_rtwi_;
  
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
  
  Scalar servo_c_ele_, servo_c_ele_r_;
  Scalar servo_c_rud_, servo_c_rud_r_;
  Scalar servo_c_tswe_, servo_c_tswe_r_;
  Scalar servo_c_lswe_, servo_c_lswe_r_;
  Scalar servo_c_rswe_, servo_c_rswe_r_;
  Scalar servo_c_ltwi_, servo_c_ltwi_r_;
  Scalar servo_c_rtwi_, servo_c_rtwi_r_;

  Scalar motor_o_, motor_o_r_;
  Scalar servo_o_ele_, servo_o_ele_r_;
  Scalar servo_o_rud_, servo_o_rud_r_;
  Scalar servo_o_tswe_, servo_o_tswe_r_;
  Scalar servo_o_lswe_, servo_o_lswe_r_;
  Scalar servo_o_rswe_, servo_o_rswe_r_;
  Scalar servo_o_ltwi_, servo_o_ltwi_r_;
  Scalar servo_o_rtwi_, servo_o_rtwi_r_;

  // control surfaces
  Scalar elevator_min_;
  Scalar elevator_max_;
  Scalar rudder_min_;
  Scalar rudder_max_;
  Scalar tail_sweep_min_;
  Scalar tail_sweep_max_;
  Scalar left_sweep_min_;
  Scalar left_sweep_max_;
  Scalar right_sweep_min_;
  Scalar right_sweep_max_;
  Scalar left_twist_min_;
  Scalar left_twist_max_;
  Scalar right_twist_min_;
  Scalar right_twist_max_;
  Scalar collective_thrust_min_;
  Scalar collective_thrust_max_;

  // Robot limits
  Vector<3> omega_max_;
  Scalar velu_max_;

  Scalar x_shift_, x_shift_r_; // shift of center of gravity in x direction

  Scalar mass_, mass_r_;  // mass
  Scalar S_, S_r_;     // wing surface
  Scalar c_, c_r_;     // chord length
  Scalar b_, b_r_;     // wing span

  // aerodynamic constants
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> xdlw_;
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> ydlw_;
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> zdlw_;
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> ldlw_;
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> mdlw_;
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> ndlw_;
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> xdrw_;
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> ydrw_;
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> zdrw_;
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> ldrw_;
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> mdrw_;
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> ndrw_;

  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 2, 4>> xdt_;
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 2, 4>> zdt_;
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 2, 4>> mdt_;

  Vector<11> xa_;
  Matrix<11, 5> xb_;
  Matrix<11, 5> xdr_;
  Matrix<11, 5> yb_;
  Matrix<11, 5> ydr_;
  Vector<11> za_;
  Matrix<11, 5> lb_;
  Matrix<11, 5> ldr_;
  Vector<11> ma_;
  Matrix<11, 5> nb_;
  Matrix<11, 5> ndr_;

  Scalar yp_;
  Scalar lp_;
  Scalar np_;
  Scalar zq_;
  Scalar mq_;
  Scalar yr_;
  Scalar lr_;
  Scalar nr_;

  Vector<11> aoa_;
  Vector<5> aos_;

  Vector<4> elevator_deflection_;
  Vector<5> rudder_deflection_;
  Matrix<1, 2> tail_sweep_;
  Vector<3> wing_sweep_;
  Vector<3> wing_twist_;

  Matrix<1, 3> lplw_;
  Matrix<1, 3> lprw_;
  Matrix<1, 2> zqt_;
  Matrix<1, 3> zqlw_;
  Matrix<1, 3> zqrw_;
  Matrix<1, 2> mqt_;
  Matrix<1, 3> mqlw_;
  Matrix<1, 3> mqrw_;
  Matrix<1, 3> lrlw_;
  Matrix<1, 3> lrrw_;
  Matrix<1, 3> nrlw_;
  Matrix<1, 3> nrrw_;

  // mechanical data
  Matrix<3, 3> J_base_, J_base_r_;
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<3, 3, 3>> J_dlw_, J_dlw_r_;
  Eigen::TensorFixedSize<Scalar, Eigen::Sizes<3, 3, 3>> J_drw_, J_drw_r_;

  Matrix<3, 3> cg_dlw_;
  Matrix<3, 3> cg_drw_;

  // Helper
  Matrix<3, 3> J_change_;
  int param_idx_;

  // fitmodel
  Vector<3> scale_star_, scale_star_r_;
  Vector<25> params_;
};

}  // namespace flightlib