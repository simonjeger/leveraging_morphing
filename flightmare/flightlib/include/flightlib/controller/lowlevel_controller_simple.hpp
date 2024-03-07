#pragma once

#include "flightlib/common/command.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/controller/lowlevel_controller_base.hpp"
#include "flightlib/dynamics/quadrotor_dynamics.hpp"

namespace flightlib {

class LowLevelControllerSimple : public LowLevelControllerBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LowLevelControllerSimple(const YAML::Node &cfg_node);
  bool reset(RobotState state);

  bool setRobotDynamics(const std::shared_ptr<DynamicsBase> dynamics_ptr,  std::shared_ptr<IntegratorBase> integrator_ptr);
  bool setOffset(const Vector<Dynamic> offset);
  bool updateRobotDynamics(const RobotState state, const Command cmd, Vector<3> wind_pred);

  Vector<Dynamic> run_ctrl(RobotState state);
  bool setGains(Vector<4> gains, const int idx);
  bool storeReservoir();
  bool restoreReservoir();
  bool getInfo(std::map<std::string, Eigen::VectorXd>& info);


 private:
  // Robot properties
  Matrix<4, 4> B_allocation_;
  Matrix<4, 4> B_allocation_inv_;

  // P gain for body rate control
  const Matrix<3, 3> Kinv_ang_vel_tau_ =
    Vector<3>(20.0, 20.0, 40.0).asDiagonal();

  // Robot to which the controller is applied
  std::shared_ptr<DynamicsBase> dynamics_ptr_;

  // Motor speeds calculated by the controller
  Vector<4> motor_omega_des_;

  // Command
  Command cmd_;
};

}  // namespace flightlib