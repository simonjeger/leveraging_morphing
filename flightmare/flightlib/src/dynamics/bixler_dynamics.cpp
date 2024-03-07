#include "flightlib/dynamics/bixler_dynamics.hpp"

namespace flightlib {

BixlerDynamics::BixlerDynamics(const YAML::Node &cfg_node):cfg_(cfg_node) {
  // all the parameters are hard coded according to the agilicious drone
  // motor drag coefficient
  mass_ = 1.01;
  S_ = 0.276;
  c_ = 0.185;
  b_ = 1.54;
  kappa_ = 0.016;

  // inertia matrix
  J_ << 0.04766, 0.0, 0.00105, 0.0, 0.05005, 0.0, 0.00105, 0.0, 0.09558;
  J_inv_ = J_.inverse();

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

  // aerodynamic constants
  CL0_ = 0.3900;
  CL_alpha_ = 4.5321;
  CL_q_ = 0.3180;
  CL_del_e_ = 0.527;
  CD0_ = 0.0765;
  CD_alpha_ = 0.3346;
  CD_q_ = 0.354;
  CD_del_e_ = 0.004;
  CY0_ = 0.0;
  CY_beta_ = -0.033;
  CY_p_ = -0.100;
  CY_r_ = 0.039;
  CY_del_a_ = 0.000;
  CY_del_r_ = 0.225;
  Cl0_ = 0.0;
  Cl_beta_ = -0.081;
  Cl_p_ = -0.529;
  Cl_r_ = 0.159;
  Cl_del_a_ = 0.453;
  Cl_del_r_ = 0.005;
  Cm0_ = 0.0200;
  Cm_alpha_ = -1.4037;
  Cm_q_ = -0.1324;
  Cm_del_e_ = -0.4236;
  Cn0_ = 0.0;
  Cn_beta_ = 0.189;
  Cn_p_ = -0.083;
  Cn_r_ = -0.948;
  Cn_del_a_ = 0.041;
  Cn_del_r_ = -0.077;
  epsilon_ = 10.0 / 180.0 * M_PI;  // [rad]

  // actuators
  motor_ptr_ = std::make_shared<Motor>(thrust_map_r_, motor_o_r_, motor_tau_inv_, motor_delay, thrust_min_, thrust_max_, motor_omega_min_, motor_omega_max_);
  servo_ptr_ail_ = std::make_shared<Servo>(servo_c_ail_r_, servo_o_ail_r_,ail_min_, ail_max_);
  servo_ptr_ele_ = std::make_shared<Servo>(servo_c_ele_r_, servo_o_ele_r_,ele_min_, ele_max_);
  servo_ptr_rud_ = std::make_shared<Servo>(servo_c_rud_r_, servo_o_rud_r_,rud_min_, rud_max_);

  // Update the *_r parameters and reset actuators
  reset();
}

BixlerDynamics::~BixlerDynamics() {}

bool BixlerDynamics::dState(const Ref<const Vector<RobotState::SIZE>> state,
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

BixlerDynamics::DynamicsFunction BixlerDynamics::getDynamicsFunction() const {
  return std::bind(
    static_cast<bool (BixlerDynamics::*)(const Ref<const Vector<STATE::SIZE>>,
                                         Ref<Vector<STATE::SIZE>>) const>(
      &BixlerDynamics::dState),
    this, std::placeholders::_1, std::placeholders::_2);
}

Vector<7> BixlerDynamics::getAccelerations(RobotState state,
                                           const Vector<Dynamic>& cmd_ll,
                                           const Scalar sim_dt,
                                           Vector<6> wind_curl, bool direct) {
  // Vector<3> pos = state.rotEuclWorldToNEDWorldFrame(
  //   state.p);  // position in inertial frame North East Down (NED)

  Vector<3> wind = wind_curl.segment<3>(0);  // wind in world frame
  Vector<3> curl = wind_curl.segment<3>(3);  // curl in world frame

  Vector<3> vel =
    state.rotEuclBodyToFRDBodyFrame(state.v) -
    state.rotEuclWorldToFRDBodyFrame(wind);  // velocity in body frame
  Scalar u = vel(0);                         // forward
  Scalar v = vel(1);                         // right
  Scalar w = vel(2);                         // down

  Vector<3> eul = state.rotEuclWorldToNEDWorldFrame(
    state.e());           // euler angles (Tait-Bryan ZYX convention)
  Scalar phi = eul(0);    // roll
  Scalar theta = eul(1);  // pitch
  //Scalar psi = eul(2);    // yaw

  Vector<3> omega = state.rotEuclBodyToFRDBodyFrame(
    state.w) - state.rotEuclWorldToFRDBodyFrame(curl);  // angular velocity in body frame (FRD)
  Scalar p = omega(0);  // around x
  Scalar q = omega(1);  // around y
  Scalar r = omega(2);  // around z

  // Aerodynamic forces calculations (beard et al., 2012, p. 44 ff)
  Scalar V = sqrt(pow(u, 2) + pow(v, 2) + pow(w, 2));  // velocity norm
  Scalar alpha = (u != 0.0) ? atan2(w, u) : 0.0;       // angle of attack
  Scalar beta = (V != 0.0) ? atan2(v, V) : 0.0;        // angle of sideslip

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

  // NOTE: usually all of CD, CL, Cm, ... depend on alpha, q, delta_e
  // lift coefficient
  Scalar CL = (V != 0.0) ? CL0_r_ + CL_alpha_r_ * alpha + CL_q_r_ * c_r_ / (2 * V) * q +
                             CL_del_e_r_ * elevator
                         : 0.0;
  // drag coefficient
  Scalar CD = (V != 0.0) ? CD0_r_ + CD_alpha_r_ * alpha + CD_q_r_ * c_r_ / (2 * V) * q +
                             CD_del_e_r_ * elevator
                         : 0.0;
  // lateral force coefficient
  Scalar CY = (V != 0.0) ? CY0_r_ + CY_beta_r_ * beta + CY_p_r_ * b_r_ / (2 * V) * p +
                             CY_r_r_ * b_r_ / (2 * V) * r + CY_del_a_r_ * aileron +
                             CY_del_r_r_ * rudder
                         : 0.0;
  // roll moment coefficient
  Scalar Cl = (V != 0.0) ? Cl0_r_ + Cl_beta_r_ * beta + Cl_p_r_ * b_r_ / (2 * V) * p +
                             Cl_r_r_ * b_r_ / (2 * V) * r + Cl_del_a_r_ * aileron +
                             Cl_del_r_r_ * rudder
                         : 0.0;
  // pitch moment coefficient
  Scalar Cm = (V != 0.0) ? Cm0_r_ + Cm_alpha_r_ * alpha + Cm_q_r_ * c_r_ / (2 * V) * q +
                             Cm_del_e_r_ * elevator
                         : 0.0;
  // yaw moment coefficient
  Scalar Cn = (V != 0.0) ? Cn0_r_ + Cn_beta_r_ * beta + Cn_p_r_ * b_r_ / (2 * V) * p +
                             Cn_r_r_ * b_r_ / (2 * V) * r + Cn_del_a_r_ * aileron +
                             Cn_del_r_r_ * rudder
                         : 0.0;

  // resulting forces and moment
  Scalar L = 0.5 * Rho * pow(V, 2) * S_r_ * CL;       // lift
  Scalar D = 0.5 * Rho * pow(V, 2) * S_r_ * CD;       // drag
  Scalar Y = 0.5 * Rho * pow(V, 2) * S_r_ * CY;       // lateral force
  Scalar l = 0.5 * Rho * pow(V, 2) * S_r_ * c_r_ * Cl;  // roll moment
  Scalar m = 0.5 * Rho * pow(V, 2) * S_r_ * c_r_ * Cm;  // pitch moment
  Scalar n = 0.5 * Rho * pow(V, 2) * S_r_ * c_r_ * Cn;  // yaw moment

  // resulting forces (aerodynamic, weight, and propulsive)
  // - aerodynamic forces are generally in the wind frame and must be
  //   translated into forces in the body frame by the wind-to-body rotation
  //   matrix (f_body = R_wind_to_body *[-D;Y;-L]) (Beard et al., 2012, p. 18).
  // - the weight force is given in the inertial frame and must be translated
  //   into the body fixed frame by the R_inertia_to_body matrix.
  // - thrust acts in the body fixed x-z-plane at a downward facing angle of 10°
  // forces in the body frame
  Vector<3> forces_vec = {-D, Y, -L};
  Vector<3> gravity_vec = {0.0, 0.0,
                           mass_r_ * Gz * -1.0};  // because this is FRD frame
  Vector<3> thrust_vec = {thrust * cos(epsilon_r_), 0.0, thrust * sin(epsilon_r_)};

  Vector<3> f_xyz = R_body_wind(alpha, beta) * forces_vec +
                    R_inertial_body(phi, theta, 0.0).transpose() * gravity_vec +
                    thrust_vec;

  // moment vector in the body frame
  Vector<3> M = {l, m, n};

  //// Global displacement
  // displacement of the drone in the inertial frame
  // (Beard et al., 2012, p.36)
  // position change in inertial coordinates
  //Vector<3> pos_dot = R_inertial_body(phi, theta, psi) * vel;

  //// Body fixed accelerations
  // see Small Unmanned Aircraft, Beard et al., 2012, p.36
  Vector<3> uvw_dot = (1 / mass_r_) * f_xyz - omega.cross(vel);

  //// Change in pitch attitude (change in euler angles)
  // see Small Unmanned Aircraft, Beard et al., 2012, p.36
  Matrix<3, 3> angles;
  angles << 1, sin(phi) * tan(theta), cos(phi) * tan(theta), 0.0, cos(phi),
    -sin(phi), 0.0, sin(phi) / cos(theta), cos(phi) / cos(theta);
  //Vector<3> eul_angle_dot = angles * omega;

  //// Pitch acceleration
  // Eulers equation (rigid body dynamics)
  // https://en.wikipedia.org/wiki/Euler%27s_equations_(rigid_body_dynamics)

  Vector<3> omega_dot =
    J_r_.colPivHouseholderQr().solve(M - omega.cross(J_r_ * omega));

  Vector<7> accelerations = Vector<7>::Zero();
  accelerations.segment<3>(0) = state.rotFRDBodyToEuclBodyFrame(uvw_dot);
  accelerations.segment<3>(3) = state.rotFRDBodyToEuclBodyFrame(omega_dot);

  // check if still within valid range of model
  Scalar limit = 1000.0;
  //Reaslistic values would be abs(alpha) > 5, abs(beta) > 10, V < 8.0, V > 12.0, but this allows reasonable traning
  if (abs(alpha) > 30.0 || abs(beta) > 50.0 || V < 6.0 || V > 20.0 || !accelerations.allFinite() || accelerations.minCoeff() < -limit || accelerations.maxCoeff() > limit){
    accelerations(6) = 1;  
  }
  else{
    accelerations(6) = 0;
  }
  
  return accelerations;
}

bool BixlerDynamics::valid() const {
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

Vector<3> BixlerDynamics::clampTorque(const Vector<3>& torque) const {
  return torque.cwiseMax(force_torque_min_.segment<3>(1))
    .cwiseMin(force_torque_max_.segment<3>(1));
}

Scalar BixlerDynamics::clampCollectiveThrust(const Scalar thrust) const {
  return std::clamp(thrust, collective_thrust_min_, collective_thrust_max_);
}

Vector<Dynamic> BixlerDynamics::clampRaw(const Vector<Dynamic> raw) const {
  Vector<4> thrust_c;
  thrust_c(0) = std::clamp(raw(0), thrust_min_, thrust_max_);
  thrust_c(1) = std::clamp(raw(1), ail_min_, ail_max_);
  thrust_c(2) = std::clamp(raw(2), ele_min_, ele_max_);
  thrust_c(3) = std::clamp(raw(3), rud_min_, rud_max_);
  return thrust_c;
}

Vector<Dynamic> BixlerDynamics::clampMotorOmega(
  const Vector<Dynamic>& omega) const {
  return omega.cwiseMax(motor_omega_min_).cwiseMin(motor_omega_max_);
}

Vector<3> BixlerDynamics::clampBodyrates(const Vector<3>& omega) const {
  return omega.cwiseMax(-omega_max_).cwiseMin(omega_max_);
}

Scalar BixlerDynamics::clampVelu(const Scalar velu) const {
    return std::clamp(velu,0.0, velu_max_);
}

Matrix<4, 4> BixlerDynamics::getAllocationMatrix(void) const {
  // compute column-wise cross product
  // tau_i = t_BM_i x F_i
  return B_allocation_;
}

Vector<Dynamic> BixlerDynamics::getRawMean(void) const {
  Vector<4> mean;
  mean(0) = (thrust_max_ + thrust_min_) / 2;
  mean(1) = (ail_max_ + ail_min_) / 2;
  mean(2) = (ele_max_ + ele_min_) / 2;
  mean(3) = (rud_max_ + rud_min_) / 2;
  return mean;
}

Vector<Dynamic> BixlerDynamics::getRawStd(void) const {
  Vector<4> std;
  std(0) = (thrust_max_ - thrust_min_) / 2;
  std(1) = (ail_max_ - ail_min_) / 2;
  std(2) = (ele_max_ - ele_min_) / 2;
  std(3) = (rud_max_ - rud_min_) / 2;
  return std;
}

Matrix<3, 3> BixlerDynamics::R_body_wind(const Scalar alpha,
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

Matrix<3, 3> BixlerDynamics::R_inertial_body(const Scalar phi,
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

Scalar BixlerDynamics::getConsumption(void) const{
  Scalar consumption = 0;
  consumption += motor_ptr_->getConsumption();
  consumption += servo_ptr_ail_->getConsumption();
  consumption += servo_ptr_ele_->getConsumption();
  consumption += servo_ptr_rud_->getConsumption();
  return consumption;
}

Vector<Dynamic> BixlerDynamics::getActuator(void) const{
  // unity works in degrees
  Vector<4> actuators;
  actuators(0) = motor_ptr_->getState();
  actuators(1) = rad2deg(servo_ptr_ail_->getState());
  actuators(2) = rad2deg(servo_ptr_ele_->getState());
  actuators(3) = rad2deg(servo_ptr_rud_->getState());
  return actuators;
}

bool BixlerDynamics::setActuator(const Vector<Dynamic>& actuator){
  // getActuator works in degrees
  motor_ptr_->setState(actuator(0));
  servo_ptr_ail_->setState(actuator(1));
  servo_ptr_ele_->setState(actuator(2));
  servo_ptr_rud_->setState(actuator(3));
  return true;
}

bool BixlerDynamics::reset() {
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

bool BixlerDynamics::randomizeParams(){
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
  S_r_ = randomize(S_);
  c_r_ = randomize(c_);
  b_r_ = randomize(b_);
  CL0_r_ = randomize(CL0_);
  CL_alpha_r_ = randomize(CL_alpha_);
  CL_q_r_ = randomize(CL_q_);
  CL_del_e_r_ = randomize(CL_del_e_);
  CD0_r_ = randomize(CD0_);
  CD_alpha_r_ = randomize(CD_alpha_);
  CD_q_r_ = randomize(CD_q_);
  CD_del_e_r_ = randomize(CD_del_e_);
  CY0_r_ = randomize(CY0_);
  CY_beta_r_ = randomize(CY_beta_);
  CY_p_r_ = randomize(CY_p_);
  CY_r_r_ = randomize(CY_r_);
  CY_del_a_r_ = randomize(CY_del_a_);
  CY_del_r_r_ = randomize(CY_del_r_);
  Cl0_r_ = randomize(Cl0_);
  Cl_beta_r_ = randomize(Cl_beta_);
  Cl_p_r_ = randomize(Cl_p_);
  Cl_r_r_ = randomize(Cl_r_);
  Cl_del_a_r_ = randomize(Cl_del_a_);
  Cl_del_r_r_ = randomize(Cl_del_r_);
  Cm0_r_ = randomize(Cm0_);
  Cm_alpha_r_ = randomize(Cm_alpha_);
  Cm_q_r_ = randomize(Cm_q_);
  Cm_del_e_r_ = randomize(Cm_del_e_);
  Cn0_r_ = randomize(Cn0_);
  Cn_beta_r_ = randomize(Cn_beta_);
  Cn_p_r_ = randomize(Cn_p_);
  Cn_r_r_ = randomize(Cn_r_);
  Cn_del_a_r_ = randomize(Cn_del_a_);
  Cn_del_r_r_ = randomize(Cn_del_r_);
  epsilon_r_ = randomize(epsilon_);

  return true;
}

bool BixlerDynamics::setModelParams(const Vector<Dynamic>& params){
  // not implemented yet
  
  return true;
}

bool BixlerDynamics::loadModelParams(const Vector<Dynamic>& params){
  // not implemented yet
  
  return true;
}

Scalar BixlerDynamics::randomize(){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  return MatrixXd::Random(1,1)(0,0)*var;
}

Scalar BixlerDynamics::randomize(Scalar x){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  return x + x*MatrixXd::Random(1,1)(0,0)*var;
}

Matrix<Dynamic,Dynamic> BixlerDynamics::randomize(Matrix<Dynamic,Dynamic> x){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  return x + x.cwiseProduct(MatrixXd::Random(x.rows(),x.cols()))*var;
}

}  // namespace flightlib