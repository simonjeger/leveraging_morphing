#pragma once
#include <unsupported/Eigen/FFT>

#include "flightlib/common/command.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/controller/lowlevel_controller_base.hpp"
#include "flightlib/dynamics/liseagle_dynamics.hpp"
#include "flightlib/controller/MiniPID.hpp"

namespace flightlib {

class LowLevelControllerCustom : public LowLevelControllerBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LowLevelControllerCustom(const YAML::Node &cfg_node);
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
  Vector<6> wind_curl_pred_;
  Matrix<8, 3> influence_lin_;
  Matrix<8, 3> influence_ang_;
  Scalar t_;
  Scalar dt_ll_;
  Scalar dt_hl_;

  // Passed in info
  Vector<8> motor_thrust_;
  Vector<8> action_m_;
  Vector<3> gt_omega_;
  Vector<3> gt_vel_;
  Vector<3> ref_omega_;
  Vector<3> ref_vel_;
  Vector<8> proj_err_;

  Vector<8> offset_;

  // PID
  MiniPID pid_0_ = MiniPID(0.0,0.0,0.0);
  MiniPID pid_1_ = MiniPID(0.0,0.0,0.0);
  MiniPID pid_2_ = MiniPID(0.0,0.0,0.0);
  MiniPID pid_3_ = MiniPID(0.0,0.0,0.0);
  MiniPID pid_4_ = MiniPID(0.0,0.0,0.0);
  MiniPID pid_5_ = MiniPID(0.0,0.0,0.0);
  MiniPID pid_6_ = MiniPID(0.0,0.0,0.0);
  MiniPID pid_7_ = MiniPID(0.0,0.0,0.0);

  // Command
  Command cmd_;

  // Influence range
  Scalar infl_range_; //[action space unit]
  Scalar window_; //[s]
  std::deque<RobotState> states_;
  std::deque<Vector<8>> actuators_;
  Scalar weigh_lin_;

  // Noise
  Scalar pos_std_;
  Scalar vel_std_;
  Scalar acc_std_;
  Scalar quat_std_;
  Scalar omega_std_;
  Scalar aac_std_;

  // Fascilitators
  Vector<8> mean_;
  Vector<8> std_;
};

}  // namespace flightlib