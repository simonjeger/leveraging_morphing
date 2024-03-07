#include "flightlib/dynamics/mpextra330_dynamics.hpp"

namespace flightlib {

mpextra330Dynamics::mpextra330Dynamics(const YAML::Node &cfg_node):cfg_(cfg_node) {
  // all the parameters are hard coded according to the agilicious drone
  // motor drag coefficient
  mass_ = 0.170;
  kappa_ = 0.016;

  // flat plates
  s_topl_ = 0.054564;
  s_topr_ = 0.054564;
  s_ail_ = 0.041274;
  s_ele_ = 0.029708;
  s_side_ = 0.123344;
  s_rud_ = 0.02682;
  s_fro_ = 0.0016;  // to be corrected

  // to be corrected
  pos_topl_ << -.033, .093, .0;
  pos_topr_ << -.033, -.093, .0;
  pos_aill_ << -.0184, .230, .0;
  pos_ailr_ << -.0184, -.230, .0;
  pos_ele_ << -.477, .0, .0;
  pos_side_ << -.162, 0., .002;
  pos_rud_ << -.578, 0., .0478;
  pos_fro_ << 0.0, 0.0, 0.0;

  // rotation axis servos
  rot_axis_aill_ << 0.0, 1.0, 0.0;
  rot_axis_ailr_ << 0.0, 1.0, 0.0;
  rot_axis_ele_ << 0.0, 1.0, 0.0;
  rot_axis_rud_ << 0.0, 0.0, -1.0;

  // rotation point of servos
  pos_rot_point_aill_ << .0243, .230, 0.;
  pos_rot_point_ailr_ << .0243, -.230, 0.;
  pos_rot_point_ele_ << -.448, 0., 0.;
  pos_rot_point_rud_ << -.530, 0., 0.0478;

  // distance rotation axis to center of surface
  dist_rot_axis_aill_ = (pos_aill_ - pos_rot_point_aill_).norm();
  dist_rot_axis_ailr_ = (pos_ailr_ - pos_rot_point_ailr_).norm();
  dist_rot_axis_ele_ = (pos_ele_ - pos_rot_point_ele_).norm();
  dist_rot_axis_rud_ = (pos_rud_ - pos_rot_point_rud_).norm();

  // inertia matrix
  J_ << 849342, 0, 135726, 0, 4642933, 0, 135726, 0, 5349198.0; //mm^4
  J_ = J_ * 171.0 / 120.0 * (1e-9); //m^4
  J_inv_ = J_.inverse();

  // motor speed limits
  motor_omega_min_ = 0.0;
  motor_omega_max_ = 2000.0;

  // motor dynamics (how quickly do they react)
  motor_tau_inv_ = 1.0 / 0.033;
  Scalar motor_delay = 0.33390702538338135;

  // thrust mapping coefficients;
  thrust_map_ << 1.562522e-6, 0.0, 0.0;

  // thrust limit
  thrust_min_ = 0.0; //N
  thrust_max_ = 3.0; //N
  ail_min_ = -50.0; //deg
  ail_max_ = 50.0;  //deg
  ele_min_ = -50.0; //deg
  ele_max_ = 50.0;  //deg
  rud_min_ = -50.0; //deg
  rud_max_ = 50.0;  //deg

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

  // Update the *_r parameters
  randomizeParams();

  // actuators
  motor_ptr_ = std::make_shared<Motor>(thrust_map_r_, motor_o_r_, motor_tau_inv_, motor_delay, thrust_min_, thrust_max_, motor_omega_min_, motor_omega_max_);
  servo_ptr_ail_ = std::make_shared<Servo>(servo_c_ail_r_, servo_o_ail_r_, ail_min_, ail_max_);
  servo_ptr_ele_ = std::make_shared<Servo>(servo_c_ele_r_, servo_o_ele_r_, ele_min_, ele_max_);
  servo_ptr_rud_ = std::make_shared<Servo>(servo_c_rud_r_, servo_o_rud_r_, rud_min_, rud_max_);
}

mpextra330Dynamics::~mpextra330Dynamics() {}

bool mpextra330Dynamics::dState(const Ref<const Vector<RobotState::SIZE>> state,
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

mpextra330Dynamics::DynamicsFunction mpextra330Dynamics::getDynamicsFunction() const {
  return std::bind(
    static_cast<bool (mpextra330Dynamics::*)(const Ref<const Vector<STATE::SIZE>>,
                                         Ref<Vector<STATE::SIZE>>) const>(
      &mpextra330Dynamics::dState),
    this, std::placeholders::_1, std::placeholders::_2);
}

Vector<7> mpextra330Dynamics::getAccelerations(RobotState state,
                                           const Vector<Dynamic>& cmd_ll,
                                           const Scalar sim_dt,
                                           Vector<6> wind_curl, bool direct) {
  
  Vector<3> wind = wind_curl.segment<3>(0);  // wind in world frame
  Vector<3> curl = wind_curl.segment<3>(3);  // curl in world frame

  Vector<3> vel = state.v - state.rotEuclWorldToEuclBodyFrame(wind);  // velocity in body frame
  Scalar u = vel(0);
  
  Vector<3> omega = state.w - state.rotEuclWorldToFRDBodyFrame(curl);

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

  // motor generates airflow over control surfaces
  vel(0) += 1.5*thrust; //random assumption that for thrust to wind is linear with a factor
  Vector<3> flow = -vel; //the flow over the control surfaces is in the opposite direction of the velocity

  // Top left plate
  Vector<3> dir_topl = {0.0, 0.0, 1.0};
  Vector<3> pos_topl = pos_topl_r_;
  Vector<3> flow_rom_rot_topl = -omega.cross(pos_topl);
  Vector<3> F_topl = flatPlateForce(dir_topl, flow + flow_rom_rot_topl, s_topl_r_);

  Vector<3> dir_topr = {0.0, 0.0, 1.0};
  Vector<3> pos_topr = pos_topr_r_;
  Vector<3> flow_rom_rot_topr = -omega.cross(pos_topr_r_);
  Vector<3> F_topr = flatPlateForce(dir_topr, flow + flow_rom_rot_topr, s_topr_r_);

  // Ailerons
  Vector<3> dir_aill = {std::sin(-aileron), 0.0, std::cos(aileron)}; //positive: roll to the right, negative: roll to the left (used to be the other way around)
  Vector<3> pos_aill = pos_rot_point_aill_ + dist_rot_axis_aill_*dir_aill.cross(rot_axis_aill_); // TODO: add randomness
  Vector<3> flow_rom_rot_aill = -omega.cross(pos_aill);
  Vector<3> F_aill = flatPlateForce(dir_aill, flow + flow_rom_rot_aill, s_ail_r_); //(this is the left aileron)        

  Vector<3> dir_ailr = {std::sin(aileron), 0.0, std::cos(aileron)};
  Vector<3> pos_ailr = pos_rot_point_ailr_ + dist_rot_axis_ailr_*dir_ailr.cross(rot_axis_ailr_);
  Vector<3> flow_rom_rot_ailr = -omega.cross(pos_ailr);
  Vector<3> F_ailr = flatPlateForce(dir_ailr, flow + flow_rom_rot_ailr, s_ail_r_);

  // Elevator
  Vector<3> dir_ele = {std::sin(-elevator), 0.0, std::cos(-elevator)}; //negative: upward deflection, positive: downward deflection
  Vector<3> pos_ele = pos_rot_point_ele_ + dist_rot_axis_ele_*dir_ele.cross(rot_axis_ele_);
  Vector<3> flow_rom_rot_ele = -omega.cross(pos_ele);
  Vector<3> F_ele = flatPlateForce(dir_ele, flow + flow_rom_rot_ele, s_ele_r_);

  // Side plate
  Vector<3> dir_side = {0.0, 1.0, 0.0};
  Vector<3> pos_side = pos_side_r_;
  Vector<3> flow_rom_rot_side = -omega.cross(pos_side);
  Vector<3> F_side = flatPlateForce(dir_side, flow + flow_rom_rot_side, s_side_r_);

  // Rudder
  Vector<3> dir_rud = {std::sin(rudder), std::cos(rudder), 0.0}; //positive: left deflection, negative: right deflection
  Vector<3> pos_rud = pos_rot_point_rud_ + dist_rot_axis_rud_*dir_rud.cross(rot_axis_rud_);
  Vector<3> flow_rom_rot_rud = -omega.cross(pos_rud);
  Vector<3> F_rud = flatPlateForce(dir_rud, flow + flow_rom_rot_rud, s_rud_r_);

  // Front plate
  Vector<3> dir_fro = {1.0, 0.0, 0.0};
  Vector<3> pos_fro = pos_fro_r_;
  Vector<3> flow_rom_rot_fro = -omega.cross(pos_fro);
  Vector<3> F_fro = flatPlateForce(dir_fro, flow + flow_rom_rot_fro, s_fro_r_);
 
  // Thrust and gravity
  Vector<3> F_thr = {thrust, 0.0, 0.0};
  Vector<3> F_gra_world = {0.0, 0.0, mass_r_*Gz};
  Vector<3> F_gra = state.rotEuclWorldToEuclBodyFrame(F_gra_world);

  // calculate forces
  Vector<3> F = F_gra + F_thr + F_topl + F_topr + F_aill + F_ailr + F_ele + F_side + F_rud + F_fro;
  
  // calculate moments
  Vector<3> M = pos_topl.cross(F_topl) + 
                pos_topr.cross(F_topr) + 
                pos_aill.cross(F_aill) +
                pos_ailr.cross(F_ailr) + 
                pos_ele.cross(F_ele) + 
                pos_side.cross(F_side) + 
                pos_rud.cross(F_rud) + 
                pos_fro.cross(F_fro); //there are two ailerons

  // don't accelerate more if above max velocity
  Vector<3> uvw_dot = F/mass_r_ - omega.cross(state.v);
  Vector<3> omega_dot = J_inv_r_*(M - omega.cross(J_r_*omega));
  Scalar max_vel = 10.0;
  Scalar max_omega = 10.0;
  for(int i = 0; i < 3; i++){
    if (vel(i) > max_vel){
      uvw_dot(i) = std::min(0.0,uvw_dot(i));
    }
    else if (vel(i) < -max_vel){
      uvw_dot(i) = std::max(0.0,uvw_dot(i));
    }
    if (omega(i) > max_omega){
      omega_dot(i) = std::min(0.0,omega_dot(i));
    }
    else if (omega(i) < -max_omega){
      omega_dot(i) = std::max(0.0,omega_dot(i));
    }
  }

  Vector<7> accelerations = Vector<7>::Zero();
  accelerations.segment<3>(0) = uvw_dot;
  accelerations.segment<3>(3) = omega_dot;

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

Vector<3> mpextra330Dynamics::flatPlateForce(Vector<3> n_s, const Vector<3> vel, const Scalar s){
  // find normalised directions
  Vector<3> n_d = vel/vel.norm();
  if (n_s.transpose() * vel < 0){
    n_s = -n_s; // two normals describe the same plane, with this we get the right result for both
  }

  Vector<3> n_l;
  if ((n_s - n_d).norm() < 1e-10){ //for nummerical reasons it's not ==
    n_l << 1.0, 0.0, 0.0; //there is no lift in this case anyway
  }
  else{
    n_l = (n_d.cross(n_s)).cross(n_d);
  }
  n_l = n_l / n_l.norm();
  
  // find alpha, lift and drag coefficients
  Scalar clamped = std::clamp((n_s.transpose()*vel).norm()/(n_s.norm()*vel.norm()), -1.0, 1.0);
  Scalar alpha = std::abs(std::acos(clamped) - M_PI/2);
  Scalar c_l = 2*std::sin(alpha)*std::cos(alpha);
  Scalar c_d = 2*std::pow(std::sin(alpha),2.0);

  // caluclate forces
  Scalar f_l = 0.5*Rho*std::pow(vel.norm(),2.0)*s*c_l;
  Scalar f_d = 0.5*Rho*std::pow(vel.norm(),2.0)*s*c_d;
  Vector<3> F_l = f_l*n_l;
  Vector<3> F_d = f_d*n_d;

  return F_l + F_d;
}

bool mpextra330Dynamics::valid() const {
  bool check = true;

  check &= mass_r_ > 0.0;
  check &= mass_r_ < 100.0;  // limit maximum mass
  check &= t_BM_.allFinite();
  check &= J_r_.allFinite();
  check &= J_inv_r_.allFinite();

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

Vector<3> mpextra330Dynamics::clampTorque(const Vector<3>& torque) const {
  return torque.cwiseMax(force_torque_min_.segment<3>(1))
    .cwiseMin(force_torque_max_.segment<3>(1));
}

Scalar mpextra330Dynamics::clampCollectiveThrust(const Scalar thrust) const {
  return std::clamp(thrust, collective_thrust_min_, collective_thrust_max_);
}

Vector<Dynamic> mpextra330Dynamics::clampRaw(const Vector<Dynamic> raw) const {
  Vector<4> thrust_c;
  thrust_c(0) = std::clamp(raw(0), thrust_min_, thrust_max_);
  thrust_c(1) = std::clamp(raw(1), ail_min_, ail_max_);
  thrust_c(2) = std::clamp(raw(2), ele_min_, ele_max_);
  thrust_c(3) = std::clamp(raw(3), rud_min_, rud_max_);
  return thrust_c;
}

Vector<Dynamic> mpextra330Dynamics::clampMotorOmega(
  const Vector<Dynamic>& omega) const {
  return omega.cwiseMax(motor_omega_min_).cwiseMin(motor_omega_max_);
}

Vector<3> mpextra330Dynamics::clampBodyrates(const Vector<3>& omega) const {
  return omega.cwiseMax(-omega_max_).cwiseMin(omega_max_);
}

Scalar mpextra330Dynamics::clampVelu(const Scalar velu) const {
    return std::clamp(velu,0.0, velu_max_);
}

Matrix<4, 4> mpextra330Dynamics::getAllocationMatrix(void) const {
  // compute column-wise cross product
  // tau_i = t_BM_i x F_i
  return B_allocation_;
}

Vector<Dynamic> mpextra330Dynamics::getRawMean(void) const {
  Vector<4> mean;
  mean(0) = (thrust_max_ + thrust_min_) / 2;
  mean(1) = (ail_max_ + ail_min_) / 2;
  mean(2) = (ele_max_ + ele_min_) / 2;
  mean(3) = (rud_max_ + rud_min_) / 2;
  return mean;
}

Vector<Dynamic> mpextra330Dynamics::getRawStd(void) const {
  Vector<4> std;
  std(0) = (thrust_max_ - thrust_min_) / 2;
  std(1) = (ail_max_ - ail_min_) / 2;
  std(2) = (ele_max_ - ele_min_) / 2;
  std(3) = (rud_max_ - rud_min_) / 2;
  return std;
}

Matrix<3, 3> mpextra330Dynamics::R_body_wind(const Scalar alpha,
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

Matrix<3, 3> mpextra330Dynamics::R_inertial_body(const Scalar phi,
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

Scalar mpextra330Dynamics::getConsumption(void) const{
  Scalar consumption = 0;
  consumption += motor_ptr_->getConsumption();
  consumption += servo_ptr_ail_->getConsumption();
  consumption += servo_ptr_ele_->getConsumption();
  consumption += servo_ptr_rud_->getConsumption();
  return consumption;
}

Vector<Dynamic> mpextra330Dynamics::getActuator(void) const{
  // unity works in degrees
  Vector<4> actuators;
  actuators(0) = motor_ptr_->getState();
  actuators(1) = servo_ptr_ail_->getState();
  actuators(2) = servo_ptr_ele_->getState();
  actuators(3) = servo_ptr_rud_->getState();
  return actuators;
}

bool mpextra330Dynamics::setActuator(const Vector<Dynamic>& actuator){
  // getActuator works in degrees
  motor_ptr_->setState(actuator(0));
  servo_ptr_ail_->setState(actuator(1));
  servo_ptr_ele_->setState(actuator(2));
  servo_ptr_rud_->setState(actuator(3));
  return true;
}

bool mpextra330Dynamics::reset() {
  randomizeParams();

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

bool mpextra330Dynamics::randomizeParams(){
  thrust_map_r_ = randomize(thrust_map_);
  J_r_ = randomize(J_);
  J_inv_r_ = J_r_.inverse();
  mass_r_ = randomize(mass_);
  servo_c_ail_r_ = randomize(servo_c_ail_);
  servo_c_ele_r_ = randomize(servo_c_ele_);
  servo_c_rud_r_ = randomize(servo_c_rud_);
  motor_o_r_ = randomize();
  servo_o_ail_r_ = randomize();
  servo_o_ele_r_ = randomize();
  servo_o_rud_r_ = randomize();

  s_topl_r_ = randomize(s_topl_);
  s_topr_r_ = randomize(s_topr_);
  s_ail_r_ = randomize(s_ail_);
  s_ele_r_ = randomize(s_ele_);
  s_side_r_ = randomize(s_side_);
  s_rud_r_ = randomize(s_rud_);
  s_fro_r_ = randomize(s_fro_);

  pos_topl_r_ = randomize(pos_topl_);
  pos_topr_r_ = randomize(pos_topr_);
  pos_aill_r_ = randomize(pos_aill_);
  pos_ailr_r_ = randomize(pos_ailr_);
  pos_ele_r_ = randomize(pos_ele_);
  pos_side_r_ = randomize(pos_side_);
  pos_rud_r_ = randomize(pos_rud_);
  pos_fro_r_ = randomize(pos_fro_);
  return true;
}

bool mpextra330Dynamics::setModelParams(const Vector<Dynamic>& params){
  // not implemented yet
  
  return true;
}

bool mpextra330Dynamics::loadModelParams(const Vector<Dynamic>& params){
  // not implemented yet
  
  return true;
}

Scalar mpextra330Dynamics::randomize(){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  return MatrixXd::Random(1,1)(0,0)*var;
}

Scalar mpextra330Dynamics::randomize(Scalar x){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  return x + x*MatrixXd::Random(1,1)(0,0)*var;
}

Matrix<Dynamic,Dynamic> mpextra330Dynamics::randomize(Matrix<Dynamic,Dynamic> x){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  return x + x.cwiseProduct(MatrixXd::Random(x.rows(),x.cols()))*var;
}

}  // namespace flightlib