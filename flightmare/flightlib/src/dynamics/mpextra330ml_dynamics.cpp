#include "flightlib/dynamics/mpextra330ml_dynamics.hpp"

namespace flightlib {

mpextra330mlDynamics::mpextra330mlDynamics(const YAML::Node &cfg_node):cfg_(cfg_node) {
  
  // std::string path = getenv("FLIGHTMARE_PATH") +
  //                    std::string("/flightlib/src/dynamics/mpextra330ml_json/");
  // for (const auto & entry : std::filesystem::directory_iterator(path)){
  //     const fdeep::model model = fdeep::load_model(entry.path());
  //     models_.push_back(model);
  // }

  // translational matrix. from motor to body [x0, x1, x2, x3, y0, y1, y2, y3,
  // z0, z1, z2, z3]
  t_BM_ << 0.075, -0.075, -0.075, 0.075, -0.10, 0.10, -0.10, 0.10, 0.0, 0.0,
    0.0, 0.0;

  // motor speed limits
  motor_omega_min_ = 0.0;
  motor_omega_max_ = 2000.0;

  // motor dynamics (how quickly do they react)
  motor_tau_inv_ = 1.0 / 0.033;
  Scalar motor_delay = 0.33390702538338135;

  // thrust mapping coefficients;
  thrust_map_ << 1.562522e-6, 0.0, 0.0;

  // thrust limit
  thrust_min_ = 0.0;
  thrust_max_ = 7.0;
  ail_min_ = -20.0;
  ail_max_ = 20.0;
  ele_min_ = -20.0;
  ele_max_ = 20.0;
  rud_min_ = -20.0;
  rud_max_ = 20.0;

  //
  collective_thrust_min_ = thrust_min_ / mass_;
  collective_thrust_max_ = thrust_max_ / mass_;
  //
  servo_c_ail_ = 1.0;
  servo_c_ele_ = 1.0;
  servo_c_rud_ = 1.0;

  // body rate max
  omega_max_ << 6.0, 6.0, 2.0;
  velu_max_ = 10;

  // allocation matrix (mapping from motor speeds to body torque)
  B_allocation_ =
    (Matrix<4, 4>() << Vector<4>::Ones().transpose(), t_BM_.row(1),
     -t_BM_.row(0), kappa_ * Vector<4>(-1.0, -1.0, 1.0, 1.0).transpose())
      .finished();

  // actuators
  motor_ptr_ = std::make_shared<Motor>(thrust_map_r_, motor_o_r_, motor_tau_inv_, motor_delay, thrust_min_, thrust_max_, motor_omega_min_, motor_omega_max_);
  servo_ptr_ail_ = std::make_shared<Servo>(servo_c_ail_r_, servo_o_ail_r_, ail_min_, ail_max_);
  servo_ptr_ele_ = std::make_shared<Servo>(servo_c_ele_r_, servo_o_ele_r_, ele_min_, ele_max_);
  servo_ptr_rud_ = std::make_shared<Servo>(servo_c_rud_r_, servo_o_rud_r_, rud_min_, rud_max_);

  // Update the *_r parameters and reset actuators
  reset();
}

mpextra330mlDynamics::~mpextra330mlDynamics() {}

bool mpextra330mlDynamics::dState(const Ref<const Vector<RobotState::SIZE>> state,
                            Ref<Vector<RobotState::SIZE>> dstate) const {
  if (!state.segment<STATE::NDYM>(0).allFinite()) return false;

  dstate.setZero();
  //
  const Vector<3> omega(state(STATE::OMEX), state(STATE::OMEY),
                        state(STATE::OMEZ));
  const Quaternion q_omega(0, omega.x(), omega.y(), omega.z());
  const Quaternion qx(state(STATE::ATTW), state(STATE::ATTX),
                      state(STATE::ATTY), state(STATE::ATTZ));

  // linear velocity = dx / dt
  dstate.segment<STATE::NPOS>(STATE::POS) =
    qx.toRotationMatrix() * state.segment<STATE::NVEL>(STATE::VEL);

  // differentiate quaternion = dq / dt
  dstate.segment<STATE::NATT>(STATE::ATT) =
    0.5 * Q_right(q_omega) * state.segment<STATE::NATT>(STATE::ATT);

  // linear acceleration = dv / dt
  dstate.segment<STATE::NVEL>(STATE::VEL) =
    state.segment<STATE::NACC>(STATE::ACC);

  // angular accleration = domega / dt
  dstate.segment<STATE::NOME>(STATE::OME) =
    state.segment<STATE::NAAC>(STATE::AAC);
  //
  return true;
}

mpextra330mlDynamics::DynamicsFunction mpextra330mlDynamics::getDynamicsFunction() const {
  return std::bind(
    static_cast<bool (mpextra330mlDynamics::*)(const Ref<const Vector<STATE::SIZE>>,
                                         Ref<Vector<STATE::SIZE>>) const>(
      &mpextra330mlDynamics::dState),
    this, std::placeholders::_1, std::placeholders::_2);
}

Vector<7> mpextra330mlDynamics::getAccelerations(RobotState state,
                                           const Vector<Dynamic>& cmd_ll,
                                           const Scalar sim_dt,
                                           Vector<6> wind_curl, bool direct) {
  Vector<3> wind = wind_curl.segment<3>(0);  // wind in world frame
  Vector<3> curl = wind_curl.segment<3>(3);  // curl in world frame
  
  Vector<3> vel = state.v - state.rotEuclWorldToEuclBodyFrame(wind);  // velocity in body frame
  Vector<3> omega = state.w - state.rotEuclWorldToFRDBodyFrame(curl);

  // update states matrix
  for (int i=1;i<states_.rows();i++){
    states_(i) = states_(i-1);
  }
  states_.block<1,4>(0,0) = cmd_ll;
  states_.block<1,3>(0,3) = vel;
  states_.block<1,3>(0,6) = omega;

  Scalar u = vel(0);
  
  // input
  Scalar thrust;
  Scalar aileron;
  Scalar elevator;
  Scalar rudder;
  if (direct) {
    thrust = cmd_ll(0);
    aileron = deg2rad(cmd_ll(1));
    elevator = deg2rad(cmd_ll(2));
    rudder = deg2rad(cmd_ll(3));
  }
  else{
    thrust = motor_ptr_->run(sim_dt,cmd_ll(0),u);
    aileron = deg2rad(servo_ptr_ail_->run(sim_dt,cmd_ll(1)));
    elevator = deg2rad(servo_ptr_ele_->run(sim_dt,cmd_ll(2)));
    rudder = deg2rad(servo_ptr_rud_->run(sim_dt,cmd_ll(3)));
  }

  // predict next state
  Eigen::ArrayXXd preds(10,6);
  int i = 0;
  for (fdeep::model const& model : models_) {
    auto pred = model.predict({fdeep::tensor(fdeep::tensor_shape(static_cast<std::size_t>(4)),std::vector<float>{1, 2, 3, 4})});
    std::vector<float> pred_vec = pred.at(0).to_vector();
    std::vector<double> pred_vec_d(pred_vec.begin(), pred_vec.end());
    preds.block<1,6>(i,0) = Eigen::ArrayXd::Map(&pred_vec_d[0], 1, 6);
    i += 1;
  }

  // find mean and std prediction
  Vector<6> pred_mean;
  pred_std_ = 0;
  for (int i=0;i<preds.rows();i++){
    pred_std_ += std::sqrt((preds - preds.mean()).square().sum()/(preds.size()-1));
    pred_mean(i) = preds.mean();
  }

  // calculate accelerations
  Vector<7> accelerations = Vector<7>::Zero();
  accelerations.block<1,6>(0,0) = (pred_mean - states_.block<1,6>(0,0).transpose())/sim_dt;

  // Vector<6> pred_mean = preds.colwise().mean();
  // Vector<6> diff = preds.rowwise() - pred_mean;
  //Vector<6> pred_std = 1/preds.rows()*diff.square();
  
  Vector<3> F;
  Vector<3> M;
  accelerations.segment<3>(0) = state.rotEuclWorldToEuclBodyFrame(F/mass_r_);
  accelerations.segment<3>(3) = state.rotEuclWorldToEuclBodyFrame(J_inv_r_*M);
  // accelerations.segment<3>(0) = uvw_dot;
  // accelerations.segment<3>(3) = omega_dot;

  

  // check if still within valid range of model
  Scalar limit = 1000.0;

  if (!accelerations.allFinite() || accelerations.minCoeff() < -limit || accelerations.maxCoeff() > limit){
    accelerations(6) = 1;
  }
  else{
    accelerations(6) = 0;
  }
  
  return accelerations;
}

bool mpextra330mlDynamics::valid() const {
  bool check = true;

  check &= t_BM_.allFinite();

  check &= motor_omega_min_ >= 0.0;
  check &= (motor_omega_max_ > motor_omega_min_);
  check &= motor_tau_inv_ > 0.0;

  check &= thrust_map_r_.allFinite();
  check &= kappa_ > 0.0;
  check &= thrust_min_ >= 0.0;
  check &= (thrust_max_ > thrust_min_);

  check &= (omega_max_.array() > 0).all();

  return check;
}

Vector<3> mpextra330mlDynamics::clampTorque(const Vector<3>& torque) const {
  return torque.cwiseMax(force_torque_min_.segment<3>(1))
    .cwiseMin(force_torque_max_.segment<3>(1));
}

Scalar mpextra330mlDynamics::clampCollectiveThrust(const Scalar thrust) const {
  return std::clamp(thrust, collective_thrust_min_, collective_thrust_max_);
}

Vector<Dynamic> mpextra330mlDynamics::clampRaw(const Vector<Dynamic> raw) const {
  Vector<4> thrust_c;
  thrust_c(0) = std::clamp(raw(0), thrust_min_, thrust_max_);
  thrust_c(1) = std::clamp(raw(1), ail_min_, ail_max_);
  thrust_c(2) = std::clamp(raw(2), ele_min_, ele_max_);
  thrust_c(3) = std::clamp(raw(3), rud_min_, rud_max_);
  return thrust_c;
}

Vector<Dynamic> mpextra330mlDynamics::clampMotorOmega(
  const Vector<Dynamic>& omega) const {
  return omega.cwiseMax(motor_omega_min_).cwiseMin(motor_omega_max_);
}

Vector<3> mpextra330mlDynamics::clampBodyrates(const Vector<3>& omega) const {
  return omega.cwiseMax(-omega_max_).cwiseMin(omega_max_);
}

Scalar mpextra330mlDynamics::clampVelu(const Scalar velu) const {
    return std::clamp(velu,0.0, velu_max_);
}

Matrix<4, 4> mpextra330mlDynamics::getAllocationMatrix(void) const {
  // compute column-wise cross product
  // tau_i = t_BM_i x F_i
  return B_allocation_;
}

Vector<Dynamic> mpextra330mlDynamics::getRawMean(void) const {
  Vector<4> mean;
  mean(0) = (thrust_max_ + thrust_min_) / 2;
  mean(1) = (ail_max_ + ail_min_) / 2;
  mean(2) = (ele_max_ + ele_min_) / 2;
  mean(3) = (rud_max_ + rud_min_) / 2;
  return mean;
}

Vector<Dynamic> mpextra330mlDynamics::getRawStd(void) const {
  Vector<4> std;
  std(0) = (thrust_max_ - thrust_min_) / 2;
  std(1) = (ail_max_ - ail_min_) / 2;
  std(2) = (ele_max_ - ele_min_) / 2;
  std(3) = (rud_max_ - rud_min_) / 2;
  return std;
}

Matrix<3, 3> mpextra330mlDynamics::R_body_wind(const Scalar alpha,
                                         const Scalar beta) {
  // This function calculates the rotation matrix from the wind to the body
  // frame (Brockhaus, 2009, p.62)
  Scalar sa = sin(alpha);
  Scalar sb = sin(beta);
  Scalar ca = cos(alpha);
  Scalar cb = cos(beta);

  Matrix<3, 3> R_bw;
  R_bw << ca * cb, -ca * sb, -sa, sb, cb, 0.0, sa * cb, -sa * sb, ca;
  return R_bw;
}

Matrix<3, 3> mpextra330mlDynamics::R_inertial_body(const Scalar phi,
                                             const Scalar theta,
                                             const Scalar psi) {
  // returns rotation matrix to rotate Vector from body to inertial frame (ZYX
  // convention) i.e. vec_inertial = R_ib * vec_body
  Scalar sph = sin(phi);
  Scalar cph = cos(phi);
  Scalar sth = sin(theta);
  Scalar cth = cos(theta);
  Scalar sps = sin(psi);
  Scalar cps = cos(psi);

  Matrix<3, 3> Rph;
  Matrix<3, 3> Rth;
  Matrix<3, 3> Rps;
  Matrix<3, 3> R_ib;

  Rph << 1.0, 0.0, 0.0, 0.0, cph, sph, 0.0, -sph, cph;
  Rth << cth, 0.0, -sth, 0.0, 1.0, 0.0, sth, 0.0, cth;
  Rps << cps, sps, 0.0, -sps, cps, 0.0, 0.0, 0.0, 1.0;

  R_ib = (Rph * (Rth * Rps)).transpose();
  return R_ib;
}

Scalar mpextra330mlDynamics::getConsumption(void) const{
  Scalar consumption = 0;
  consumption += motor_ptr_->getConsumption();
  consumption += servo_ptr_ail_->getConsumption();
  consumption += servo_ptr_ele_->getConsumption();
  consumption += servo_ptr_rud_->getConsumption();
  return consumption;
}

Vector<Dynamic> mpextra330mlDynamics::getActuator(void) const{
  // unity works in degrees
  Vector<4> actuators;
  actuators(0) = motor_ptr_->getState();
  actuators(1) = servo_ptr_ail_->getState();
  actuators(2) = servo_ptr_ele_->getState();
  actuators(3) = servo_ptr_rud_->getState();
  return actuators;
}

bool mpextra330mlDynamics::setActuator(const Vector<Dynamic>& actuator){
  // getActuator works in degrees
  motor_ptr_->setState(actuator(0));
  servo_ptr_ail_->setState(actuator(1));
  servo_ptr_ele_->setState(actuator(2));
  servo_ptr_rud_->setState(actuator(3));
  return true;
}

bool mpextra330mlDynamics::reset() {
  randomizeParams();
  states_.setZero();

  motor_ptr_->reset(thrust_map_r_, motor_o_r_);
  servo_ptr_ail_->reset(servo_c_ail_r_, servo_o_ail_r_);
  servo_ptr_ele_->reset(servo_c_ele_r_, servo_o_ele_r_);
  servo_ptr_rud_->reset(servo_c_rud_r_, servo_o_rud_r_);

  //If start_actuator is defined in the config file, set it accordingly
  try {motor_ptr_->setState(cfg_["start_actuator"]["actu_0"].as<Scalar>());}
  catch (...) {}
  try {servo_ptr_ail_->setState(cfg_["start_actuator"]["actu_1"].as<Scalar>());}
  catch (...) {}
  try {servo_ptr_ele_->setState(cfg_["start_actuator"]["actu_2"].as<Scalar>());}
  catch (...) {}
  try {servo_ptr_rud_->setState(cfg_["start_actuator"]["actu_3"].as<Scalar>());}
  catch (...) {}
  
  return true;
}

bool mpextra330mlDynamics::randomizeParams(){
  thrust_map_r_ = randomize(thrust_map_);
  servo_c_ail_r_ = randomize(servo_c_ail_);
  servo_c_ele_r_ = randomize(servo_c_ele_);
  servo_c_rud_r_ = randomize(servo_c_rud_);
  motor_o_r_ = randomize();
  servo_o_ail_r_ = randomize();
  servo_o_ele_r_ = randomize();
  servo_o_rud_r_ = randomize();
  return true;
}

bool mpextra330mlDynamics::setModelParams(const Vector<Dynamic>& params){
  // not implemented yet
  
  return true;
}

bool mpextra330mlDynamics::loadModelParams(const Vector<Dynamic>& params){
  // not implemented yet
  
  return true;
}

Scalar mpextra330mlDynamics::randomize(){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  return MatrixXd::Random(1,1)(0,0)*var;
}

Scalar mpextra330mlDynamics::randomize(Scalar x){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  return x + x*MatrixXd::Random(1,1)(0,0)*var;
}

Matrix<Dynamic,Dynamic> mpextra330mlDynamics::randomize(Matrix<Dynamic,Dynamic> x){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  return x + x.cwiseProduct(MatrixXd::Random(x.rows(),x.cols()))*var;
}

}  // namespace flightlib