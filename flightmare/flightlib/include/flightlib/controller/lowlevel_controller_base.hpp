// from agilicious
#pragma once

#include <map>

#include "flightlib/common/command.hpp"
#include "flightlib/common/robot_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/dynamics/quadrotor_dynamics.hpp"
#include "flightlib/common/integrator_euler.hpp"
#include "flightlib/common/integrator_rk4.hpp"

namespace flightlib {

class LowLevelControllerBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  LowLevelControllerBase();
  virtual ~LowLevelControllerBase() = default;
  virtual bool reset(RobotState state) = 0;

  virtual bool setRobotDynamics(const std::shared_ptr<DynamicsBase> dynamics_ptr,  std::shared_ptr<IntegratorBase> integrator_ptr) = 0;
  virtual bool setOffset(const Vector<Dynamic> offset) = 0;
  virtual bool updateRobotDynamics(const RobotState state, const Command cmd, Vector<3> wind_pred) = 0;

  virtual Vector<Dynamic> run_ctrl(RobotState state) = 0;
  virtual bool setGains(Vector<4> gains, const int idx) = 0;
  virtual bool storeReservoir() = 0;
  virtual bool restoreReservoir() = 0;
  virtual bool getInfo(std::map<std::string, Eigen::VectorXd>& info) = 0;

 protected:
  // Params
  YAML::Node cfg_;

  // Command
  Command cmd_;

  // State of Robot
  RobotState state_;

  // Motor speeds calculated by the controller
  Vector<4> motor_omega_des_;

  // Robot to which the controller is applied
  std::shared_ptr<DynamicsBase> dynamics_ptr_;
  std::shared_ptr<IntegratorBase> integrator_ptr_;
};


}  // namespace flightlib