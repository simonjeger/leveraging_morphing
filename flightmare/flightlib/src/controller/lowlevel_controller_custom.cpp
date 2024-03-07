#include "flightlib/controller/lowlevel_controller_custom.hpp"

namespace flightlib {

LowLevelControllerCustom::LowLevelControllerCustom(const YAML::Node &cfg_node) {
  cfg_ = cfg_node;
  dt_ll_ = 1e-3;
  dt_hl_ = cfg_["simulation"]["ctr_dt"].as<Scalar>();
  t_ = 0.0;

  infl_range_ = 0.2;
  window_ = 0.16;

  pid_0_.setDt(dt_ll_);
  pid_1_.setDt(dt_ll_);
  pid_2_.setDt(dt_ll_);
  pid_3_.setDt(dt_ll_);
  pid_4_.setDt(dt_ll_);
  pid_5_.setDt(dt_ll_);
  pid_6_.setDt(dt_ll_);
  pid_7_.setDt(dt_ll_);

  // noise
  pos_std_ = cfg_["state_estimation"]["pos_std"].as<Scalar>();
  vel_std_ = cfg_["state_estimation"]["vel_std"].as<Scalar>();
  acc_std_ = cfg_["state_estimation"]["acc_std"].as<Scalar>();
  quat_std_ = cfg_["state_estimation"]["att_std"].as<Scalar>();
  omega_std_ = cfg_["state_estimation"]["omega_std"].as<Scalar>();
  aac_std_ = cfg_["state_estimation"]["aac_std"].as<Scalar>();
}

bool LowLevelControllerCustom::reset(RobotState state) {
  motor_thrust_ = mean_;
  offset_.Vector<8>::Zero();

  states_.clear();
  for (int i=0;i<window_/dt_hl_+1;i++){
    states_.push_back(state);
  }
  
  pid_0_.reset();
  pid_1_.reset();
  pid_2_.reset();
  pid_3_.reset();
  pid_4_.reset();
  pid_5_.reset();
  pid_6_.reset();
  pid_7_.reset();

  if (cfg_["robot"]["rotor_ctrl"].as<Scalar>() == 0){ //at this point in time cmd).isRaw() is not set yet
    //the following weights have never been tested
    pid_0_.setPID(0.1, 0.05, 0.01);
    pid_1_.setPID(0.024, 0.006, 0.003);
    pid_2_.setPID(0.48, 0.16, 0.13);
    pid_3_.setPID(0.1, 0.03, 0.06);
    pid_4_.setPID(0.1, 0.03, 0.06);
    pid_5_.setPID(0.1, 0.03, 0.06);
    pid_6_.setPID(0.052, 0.0, 0.032);
    pid_7_.setPID(0.052, 0.0, 0.032);
  } else{
    pid_0_.setPID(0.0, 0.0, 0.0, 0.0);
    pid_1_.setPID(1.1, 100.0, 0.02, 5.1);
    pid_2_.setPID(2.9, 100.0, 0.1, 1.4);
    pid_3_.setPID(6.0, 0.0, 0.1, 0.0);
    pid_4_.setPID(3.5, 0.0, 0.1, 0.0);
    pid_5_.setPID(3.5, 0.0, 0.1, 0.0);
    pid_6_.setPID(0.89, 0.0, 0.27, 2.4);
    pid_7_.setPID(0.89, 0.0, 0.27, 2.4);
  }

  weigh_lin_ = 0.5;

  return true;
}

bool LowLevelControllerCustom::setRobotDynamics(
  const std::shared_ptr<DynamicsBase> dynamics_ptr,  std::shared_ptr<IntegratorBase> integrator_ptr) {
  dynamics_ptr_ = dynamics_ptr;
  integrator_ptr_ = integrator_ptr;

  dt_ll_ = integrator_ptr_->dtMax();
  mean_ = dynamics_ptr_->getRawMean();
  std_ = dynamics_ptr_->getRawStd();
  return true;
}

bool LowLevelControllerCustom::setOffset(const Vector<Dynamic> offset) {
  offset_ = offset.cwiseProduct(std_) + mean_;
  return true;
}

bool LowLevelControllerCustom::updateRobotDynamics(const RobotState state, const Command cmd, Vector<3> wind_pred) {
  t_ = 0.0;
  cmd_ = cmd;
  wind_curl_pred_.setZero();
  wind_curl_pred_.segment<3>(0) = wind_pred;
  states_.pop_front();

  // add noise
  state_ = state;
  state_.p += MatrixXd::Random(3,1)*pos_std_;
  state_.v += MatrixXd::Random(3,1)*vel_std_;
  state_.a += MatrixXd::Random(3,1)*acc_std_;
  state_.qx += MatrixXd::Random(4,1)*quat_std_;
  state_.w += MatrixXd::Random(3,1)*omega_std_;
  state_.aa += MatrixXd::Random(3,1)*aac_std_;
  states_.push_back(state_);

  Vector<8> actuator = dynamics_ptr_->getActuator() + offset_;

  // Calculate the influence of each actuator on the velocity and angular velocity
  for (int i=0;i<8;i++){
    Vector<8> cmd_infl;
    if (!cmd_.isRaw()){
      cmd_infl = actuator;
    } else {
      cmd_infl = cmd_.getRaw();
    }

    Scalar act_i = (cmd_infl(i) - mean_(i))/std_(i);

    cmd_infl(i) = (act_i - infl_range_)*std_(i) + mean_(i);
    Vector<7> acc_min = dynamics_ptr_->getAccelerations(state_, cmd_infl, 0.1, wind_curl_pred_, true);  //simulate for 0.1 s but this actually doesn't matter since we don't simulate the actuators (true)

    cmd_infl(i) = (act_i + infl_range_)*std_(i) + mean_(i);
    Vector<7> acc_max = dynamics_ptr_->getAccelerations(state_, cmd_infl, 0.1, wind_curl_pred_, true);

    influence_lin_.row(i) = (acc_max - acc_min).segment(0,3);
    influence_ang_.row(i) = (acc_max - acc_min).segment(3,3);
    influence_ang_.row(i) /= infl_range_;
  }

  // Normalization through each column
  for (int j=0;j<3;j++){
    influence_ang_.col(j) /= (influence_ang_.col(j).cwiseAbs()).sum();
  }

  // Propagate dynamics (only for rotor_ctrl = 0)
  if (cmd_.isRaw()){
    const Scalar max_dt = integrator_ptr_->dtMax();
    actuators_.clear();
    for (int i=1;i<states_.size();i++){ //don't update the first entry since that should be the previous state (for interpolation below)
      Scalar remain_ctl_dt = dt_hl_;
      RobotState next_state = states_[i];

      // propagate dynamics
      int j = 0;
      while (remain_ctl_dt > 0.0) {
        const Scalar sim_dt = std::min(remain_ctl_dt, max_dt);

        Vector<7> accelerations;
        if (i==1){
          accelerations = dynamics_ptr_->getAccelerations(states_[i], cmd_.getRaw(), sim_dt, wind_curl_pred_);
        }
        else{
          accelerations = dynamics_ptr_->getAccelerations(states_[i], actuators_[j], sim_dt, wind_curl_pred_, true); //use recorded actuator positions
        }

        // compute body force and torque
        states_[i].a = accelerations.segment<3>(0);
        states_[i].aa = accelerations.segment<3>(3);
        
        // actuator positions
        states_[i].x.segment(STATE::ACT, cmd_.raw_.rows()) = dynamics_ptr_->getActuator();
        if (i==1){
          actuators_.push_back(dynamics_ptr_->getActuator()); //so we don't simulate actuators again
        }
        
        // dynamics integration
        integrator_ptr_->step(states_[i].x, sim_dt, next_state.x);

        //
        states_[i].x = next_state.x;
        remain_ctl_dt -= sim_dt;

        j +=1;
      }
    }
  }
  return true;
}

Vector<Dynamic> LowLevelControllerCustom::run_ctrl(RobotState state) { 
  // Setting ground truth
  gt_vel_ = state.v;
  gt_omega_ = state.w;
  Vector<3> gt_acc = state.a;
  Vector<3> gt_aac = state.aa;
  Vector<3> wind_b = state.rotEuclWorldToFRDBodyFrame(wind_curl_pred_.segment<3>(0));

  // initialize variables
  Vector<8> proj_ref_lin = Vector<8>::Zero();
  Vector<8> proj_ref_ang = Vector<8>::Zero();
  Vector<8> proj_ref = Vector<8>::Zero();

  Vector<8> proj_state_lin = Vector<8>::Zero();
  Vector<8> proj_state_ang = Vector<8>::Zero();
  Vector<8> proj_state = Vector<8>::Zero();

  Vector<8> proj_lin_d = Vector<8>::Zero();
  Vector<8> proj_ang_d = Vector<8>::Zero();
  Vector<8> proj_d = Vector<8>::Zero();

  // find projection
  Vector<8> motor_thrust;
  if (!cmd_.isRaw()) {
    // interpolate between previous and future state (and use this as a reference)
    ref_vel_.setZero();
    ref_omega_ = cmd_.getOmega();

    // ref velocities
    proj_ref_ang = influence_ang_*ref_omega_;

    // gt velocities
    proj_state_ang = influence_ang_*gt_omega_;

    // gt derrivatives
    proj_ang_d = influence_ang_*gt_aac;

  } else {
    // interpolate between previous and future state (and use this as a reference)
    ref_vel_ = states_[0].v*(dt_hl_ - t_)/dt_hl_ + states_[1].v*(t_)/dt_hl_;
    ref_omega_ = states_[0].w*(dt_hl_ - t_)/dt_hl_ + states_[1].w*(t_)/dt_hl_;

    // ref velocities
    proj_ref_lin = influence_lin_*ref_vel_;
    proj_ref_ang = influence_ang_*ref_omega_;

    // gt velocities
    proj_state_lin = influence_lin_*gt_vel_;
    proj_state_ang = influence_ang_*gt_omega_;

    // gt derrivatives
    proj_lin_d = influence_lin_*gt_acc;
    proj_ang_d = influence_ang_*gt_aac;
  }

  proj_ref = proj_ref_lin*weigh_lin_ + proj_ref_ang*(1-weigh_lin_);
  proj_state = proj_state_lin*weigh_lin_ + proj_state_ang*(1-weigh_lin_);
  proj_err_ = proj_ref - proj_state;
  proj_d = proj_lin_d*weigh_lin_ + proj_ang_d*(1-weigh_lin_);

  // PID control
  Vector<8> act_ll;
  act_ll(0) = pid_0_.getOutput(proj_state(0),proj_ref(0),true,proj_d(0));
  act_ll(1) = pid_1_.getOutput(proj_state(1),proj_ref(1),true,proj_d(1));
  act_ll(2) = pid_2_.getOutput(proj_state(2),proj_ref(2),true,proj_d(2));
  act_ll(3) = pid_3_.getOutput(proj_state(3),proj_ref(3),true,proj_d(3));
  act_ll(4) = pid_4_.getOutput(proj_state(4),proj_ref(4),true,proj_d(4));
  act_ll(5) = pid_5_.getOutput(proj_state(5),proj_ref(5),true,proj_d(5));
  act_ll(6) = pid_6_.getOutput(proj_state(6),proj_ref(6),true,proj_d(6));
  act_ll(7) = pid_7_.getOutput(proj_state(7),proj_ref(7),true,proj_d(7));

  // Adjust gains for speed (except for motor)
  Scalar V = (gt_vel_ - wind_b).norm();
  act_ll.segment<7>(1) *= std::pow(8.0/V,2); //was tuned at 8 m/s

  // Add thrust to act
  if (!cmd_.isRaw()) {
    act_ll(0) = (cmd_.getCollectiveThrust() - mean_(0))/std_(0);
  }
  
  if (!cmd_.isRaw()) {
    motor_thrust = mean_ + act_ll.cwiseProduct(std_);
  } else {
    motor_thrust = cmd_.getRaw() + act_ll.cwiseProduct(std_);
  }

  motor_thrust_ = dynamics_ptr_->clampRaw(motor_thrust);
  if (!cmd_.isRaw()) {
    dynamics_ptr_->getAccelerations(state, motor_thrust_, dt_ll_, wind_curl_pred_);
  }
  t_ += dt_ll_;

  return motor_thrust_;
}


bool LowLevelControllerCustom::setGains(Vector<4> gains, const int idx){
  if (idx == -1){
    pid_0_.setPID(gains(0), gains(1), gains(2), gains(3));
    pid_1_.setPID(gains(0), gains(1), gains(2), gains(3));
    pid_2_.setPID(gains(0), gains(1), gains(2), gains(3));
    pid_3_.setPID(gains(0), gains(1), gains(2), gains(3));
    pid_4_.setPID(gains(0), gains(1), gains(2), gains(3));
    pid_5_.setPID(gains(0), gains(1), gains(2), gains(3));
    pid_6_.setPID(gains(0), gains(1), gains(2), gains(3));
    pid_7_.setPID(gains(0), gains(1), gains(2), gains(3));
  }
  else if (idx == 0){
    pid_0_.setPID(gains(0), gains(1), gains(2), gains(3));
  }
  else if (idx == 1){
    pid_1_.setPID(gains(0), gains(1), gains(2), gains(3));
  }
  else if (idx == 2){
    pid_2_.setPID(gains(0), gains(1), gains(2), gains(3));
  }
  else if (idx == 3){
    pid_3_.setPID(gains(0), gains(1), gains(2), gains(3));
  }
  else if (idx == 4){
    pid_4_.setPID(gains(0), gains(1), gains(2), gains(3));
  }
  else if (idx == 5){
    pid_5_.setPID(gains(0), gains(1), gains(2), gains(3));
  }
  else if (idx == 6){
    pid_6_.setPID(gains(0), gains(1), gains(2), gains(3));
  }
  else if (idx == 7){
    pid_7_.setPID(gains(0), gains(1), gains(2), gains(3));
  }

  return true;
}

bool LowLevelControllerCustom::getInfo(std::map<std::string, Eigen::VectorXd> &info){
  info["gt_omega"] = gt_omega_;
  info["gt_vel"] = gt_vel_;
  info["ref_omega"] = ref_omega_;
  info["ref_vel"] = ref_vel_;
  Vector<24> infl;
  infl.segment<8>(0) = influence_ang_.block<8,1>(0,0);
  infl.segment<8>(8) = influence_ang_.block<8,1>(0,1);
  infl.segment<8>(16) = influence_ang_.block<8,1>(0,2);
  info["proj_matrix"] = infl;

  return true;
}

bool LowLevelControllerCustom::storeReservoir(){
  pid_0_.storeReservoir();
  pid_1_.storeReservoir();
  pid_2_.storeReservoir();
  pid_3_.storeReservoir();
  pid_4_.storeReservoir();
  pid_5_.storeReservoir();
  pid_6_.storeReservoir();
  pid_7_.storeReservoir();
  
  return true;
}

bool LowLevelControllerCustom::restoreReservoir(){
  pid_0_.restoreReservoir();
  pid_1_.restoreReservoir();
  pid_2_.restoreReservoir();
  pid_3_.restoreReservoir();
  pid_4_.restoreReservoir();
  pid_5_.restoreReservoir();
  pid_6_.restoreReservoir();
  pid_7_.restoreReservoir();

  return true;
}

}  // namespace flightlib