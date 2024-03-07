#include "flightlib/dynamics/quadrotor_dynamics.hpp"

namespace flightlib {

QuadrotorDynamics::QuadrotorDynamics(const YAML::Node &cfg_node, const Scalar mass) : cfg_(cfg_node), mass_(mass) {
  // all the parameters are hard coded according to the agilicious drone
  // motor drag coefficient
  kappa_ = 0.016;

  // inertia matrix
  J_ = Matrix<3, 3>(Vector<3>(0.0025, 0.0021, 0.0043).asDiagonal());
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
  Scalar motor_delay = 0.0;

  // thrust mapping coefficients;
  // thrust = t1 * motor_omega * motor_omega + t2 * motor_omega + t3
  thrust_map_ << 1.562522e-6, 0.0, 0.0;

  // thrust limit
  thrust_min_ = 0.0;
  thrust_max_ = motor_omega_max_ * motor_omega_max_ * thrust_map_(0) +
                motor_omega_max_ * thrust_map_(1) + thrust_map_(2);

  //
  collective_thrust_min_ = 4.0 * thrust_min_ / mass_;
  collective_thrust_max_ = 4.0 * thrust_max_ / mass_;

  // torque limits
  force_torque_min_(0) = 0.0;
  force_torque_max_(0) = thrust_max_ * 4;
  // torque x
  force_torque_min_(1) = B_allocation_.row(1) * (Vector<4>() << thrust_max_, 0.0, thrust_max_, 0.0).finished();
  force_torque_max_(1) = B_allocation_.row(1) * (Vector<4>() << 0.0, thrust_max_, 0.0, thrust_max_).finished();
  // torque y
  force_torque_min_(2) = B_allocation_.row(2) * (Vector<4>() << thrust_max_, 0.0, 0.0, thrust_max_).finished();
  force_torque_max_(2) = B_allocation_.row(2) * (Vector<4>() << 0.0, thrust_max_, thrust_max_, 0.0).finished();
  // torque z
  force_torque_min_(3) = B_allocation_.row(3) * (Vector<4>() << thrust_max_, thrust_max_, 0.0, 0.0).finished();
  force_torque_max_(3) = B_allocation_.row(3) * (Vector<4>() << 0.0, 0.0, thrust_max_, thrust_max_).finished();

  // body rate max
  omega_max_ << 6.0, 6.0, 2.0;
  velu_max_ = 40;

  // allocation matrix (mapping from motor speeds to body torque)
  B_allocation_ =
    (Matrix<4, 4>() << Vector<4>::Ones().transpose(), t_BM_.row(1),
     -t_BM_.row(0), kappa_ * Vector<4>(-1.0, -1.0, 1.0, 1.0).transpose())
      .finished();

  // body drag coefficients
  cd1_ << 0.0, 0.0, 0.0;
  cd3_ << 0.0, 0.0, 0.0;
  cdz_h_ = 0.0;

  // actuators
  motor_ptr_0_ = std::make_shared<Motor>(thrust_map_0_r_, motor_o_0_r_, motor_tau_inv_, motor_delay, thrust_min_, thrust_max_, motor_omega_min_, motor_omega_max_);
  motor_ptr_1_ = std::make_shared<Motor>(thrust_map_1_r_, motor_o_1_r_, motor_tau_inv_, motor_delay, thrust_min_, thrust_max_, motor_omega_min_, motor_omega_max_);
  motor_ptr_2_ = std::make_shared<Motor>(thrust_map_2_r_, motor_o_2_r_, motor_tau_inv_, motor_delay, thrust_min_, thrust_max_, motor_omega_min_, motor_omega_max_);
  motor_ptr_3_ = std::make_shared<Motor>(thrust_map_3_r_, motor_o_3_r_, motor_tau_inv_, motor_delay, thrust_min_, thrust_max_, motor_omega_min_, motor_omega_max_);

  // Update the *_r parameters and reset actuators
  reset();
}

QuadrotorDynamics::~QuadrotorDynamics() {}

bool QuadrotorDynamics::dState(const Ref<const Vector<RobotState::SIZE>> state,
                               Ref<Vector<RobotState::SIZE>> dstate) const {
  if (!state.segment<STATE::NDYM>(0).allFinite()) return false;

  dstate.setZero();
  //
  const Vector<3> omega(state(STATE::OMEX), state(STATE::OMEY),
                        state(STATE::OMEZ));
  const Quaternion q_omega(0, omega.x(), omega.y(), omega.z());

  // linear velocity = dx / dt
  dstate.segment<STATE::NPOS>(STATE::POS) =
    state.segment<STATE::NVEL>(STATE::VEL);

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

QuadrotorDynamics::DynamicsFunction QuadrotorDynamics::getDynamicsFunction()
  const {
  return std::bind(
    static_cast<bool (QuadrotorDynamics::*)(
      const Ref<const Vector<STATE::SIZE>>, Ref<Vector<STATE::SIZE>>) const>(
      &QuadrotorDynamics::dState),
    this, std::placeholders::_1, std::placeholders::_2);
}

Vector<7> QuadrotorDynamics::getAccelerations(RobotState state,
                                              const Vector<Dynamic>& cmd_ll,
                                              const Scalar sim_dt,
                                              Vector<6> wind_curl, bool direct) {
  Vector<3> wind = wind_curl.segment<3>(0);  // wind in world frame
  Vector<3> curl = wind_curl.segment<3>(3);  // curl in world frame (use of curl here not implemented)
  
  Vector<3> force;
  Vector<3> torque;

  // compute the body drag
  const Vector<3> body_vel_rel =
    state.q().toRotationMatrix().transpose() * (state.v - wind);

  // input
  Vector<4> motor_thrusts;
  if (direct) {
    motor_thrusts(0) = cmd_ll(0)*(thrust_max_ - thrust_min_)/2.0 + (thrust_max_ + thrust_min_)/2.0;
    motor_thrusts(1) = cmd_ll(1)*(thrust_max_ - thrust_min_)/2.0 + (thrust_max_ + thrust_min_)/2.0;
    motor_thrusts(2) = cmd_ll(2)*(thrust_max_ - thrust_min_)/2.0 + (thrust_max_ + thrust_min_)/2.0;
    motor_thrusts(3) = cmd_ll(3)*(thrust_max_ - thrust_min_)/2.0 + (thrust_max_ + thrust_min_)/2.0;
  }
  else{
    motor_thrusts(0) = motor_ptr_0_->run(sim_dt,cmd_ll(0),body_vel_rel(2));
    motor_thrusts(1) = motor_ptr_1_->run(sim_dt,cmd_ll(1),body_vel_rel(2));
    motor_thrusts(2) = motor_ptr_2_->run(sim_dt,cmd_ll(2),body_vel_rel(2));
    motor_thrusts(3) = motor_ptr_3_->run(sim_dt,cmd_ll(3),body_vel_rel(2));
  }

  Vector<4> force_torques = B_allocation_r_ * motor_thrusts;

  // Compute linear acceleration and body torque
  force << 0.0, 0.0, force_torques[0];

  // compute body torque
  torque = force_torques.segment<3>(1);

  Vector<3> force_bodydrag = getBodyDrag(body_vel_rel);

  // compute accleration
  Vector<3> acc = state.q() * (force - force_bodydrag) * 1.0 / mass_r_ + GVEC;

  Vector<3> aac = J_inv_r_ * (torque - state.w.cross(J_r_ * state.w));

  Vector<7> accelerations = Vector<7>::Zero();
  accelerations.segment<3>(0) = acc;
  accelerations.segment<3>(3) = aac;
  
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

bool QuadrotorDynamics::valid() const {
  bool check = true;

  check &= mass_r_ > 0.0;
  check &= mass_r_ < 100.0;  // limit maximum mass
  check &= t_BM_.allFinite();
  check &= J_r_.allFinite();
  check &= J_inv_r_.allFinite();

  check &= motor_omega_min_ >= 0.0;
  check &= (motor_omega_max_ > motor_omega_min_);
  check &= motor_tau_inv_ > 0.0;

  check &= thrust_map_0_r_.allFinite();
  check &= thrust_map_1_r_.allFinite();
  check &= thrust_map_2_r_.allFinite();
  check &= thrust_map_3_r_.allFinite();
  check &= kappa_ > 0.0;
  check &= thrust_min_ >= 0.0;
  check &= (thrust_max_ > thrust_min_);

  check &= (omega_max_.array() > 0).all();

  return check;
}

Vector<3> QuadrotorDynamics::clampTorque(const Vector<3>& torque) const {
  return torque.cwiseMax(force_torque_min_.segment<3>(1))
    .cwiseMin(force_torque_max_.segment<3>(1));
}

Scalar QuadrotorDynamics::clampCollectiveThrust(const Scalar raw) const {
  return std::clamp(raw, collective_thrust_min_, collective_thrust_max_);
}

Vector<Dynamic> QuadrotorDynamics::clampRaw(
  const Vector<Dynamic> thrust) const {
  return thrust.cwiseMax(thrust_min_).cwiseMin(thrust_max_);
}

Vector<Dynamic> QuadrotorDynamics::clampMotorOmega(
  const Vector<Dynamic>& omega) const {
  return omega.cwiseMax(motor_omega_min_).cwiseMin(motor_omega_max_);
}

Vector<3> QuadrotorDynamics::clampBodyrates(const Vector<3>& omega) const {
  return omega.cwiseMax(-omega_max_).cwiseMin(omega_max_);
}

Scalar QuadrotorDynamics::clampVelu(const Scalar velu) const {
  return std::clamp(velu, -velu_max_, velu_max_);
}

Matrix<4, 4> QuadrotorDynamics::getAllocationMatrix(void) const {
  // compute column-wise cross product
  // tau_i = t_BM_i x F_i
  return B_allocation_;
}

Vector<Dynamic> QuadrotorDynamics::getRawMean(void) const {
  Vector<4> mean;
  mean(0) = (thrust_max_ - thrust_min_) / 2;
  mean(1) = (thrust_max_ - thrust_min_) / 2;
  mean(2) = (thrust_max_ - thrust_min_) / 2;
  mean(3) = (thrust_max_ - thrust_min_) / 2;
  return mean;
}

Vector<Dynamic> QuadrotorDynamics::getRawStd(void) const {
  Vector<4> std;
  std(0) = (thrust_max_ - thrust_min_) / 2;
  std(1) = (thrust_max_ - thrust_min_) / 2;
  std(2) = (thrust_max_ - thrust_min_) / 2;
  std(3) = (thrust_max_ - thrust_min_) / 2;
  return std;
}

Vector<3> QuadrotorDynamics::getBodyDrag(const Vector<3>& body_vel) {
  return cd1_r_.cwiseProduct(body_vel) +
         cd3_r_.cwiseProduct(body_vel.array().pow(3.0).matrix()) -
         (Vector<3>() << 0.0, 0.0,
          cdz_h_r_ * (body_vel.x() * body_vel.x() + body_vel.y() * body_vel.y()))
           .finished();
}

Scalar QuadrotorDynamics::getConsumption(void) const{
  Scalar consumption = 0;
  consumption += motor_ptr_0_->getConsumption();
  consumption += motor_ptr_1_->getConsumption();
  consumption += motor_ptr_2_->getConsumption();
  consumption += motor_ptr_3_->getConsumption();
  return consumption;
}

Vector<Dynamic> QuadrotorDynamics::getActuator(void) const{
  // everything that leaves dynamics is in rad
  Vector<4> actuators;
  actuators(0) = motor_ptr_0_->getState();
  actuators(1) = motor_ptr_1_->getState();
  actuators(2) = motor_ptr_2_->getState();
  actuators(3) = motor_ptr_3_->getState();

  return actuators;
}

bool QuadrotorDynamics::setActuator(const Vector<Dynamic>& actuator){
  // getActuator works in degrees
  motor_ptr_0_->setState(actuator(0));
  motor_ptr_1_->setState(actuator(1));
  motor_ptr_2_->setState(actuator(2));
  motor_ptr_3_->setState(actuator(3));

  return true;
}

bool QuadrotorDynamics::reset() {
  randomizeParams();

  motor_ptr_0_->reset(thrust_map_0_r_, motor_o_0_r_);
  motor_ptr_1_->reset(thrust_map_1_r_, motor_o_1_r_);
  motor_ptr_2_->reset(thrust_map_2_r_, motor_o_2_r_);
  motor_ptr_3_->reset(thrust_map_3_r_, motor_o_3_r_);

  return true;
}

bool QuadrotorDynamics::randomizeParams(){
  mass_r_ = randomize(mass_);
  B_allocation_r_ = randomize(B_allocation_);
  J_r_ = randomize(J_);
  J_inv_r_ = J_r_.inverse();
  thrust_map_0_r_ = randomize(thrust_map_);
  thrust_map_1_r_ = randomize(thrust_map_);
  thrust_map_2_r_ = randomize(thrust_map_);
  thrust_map_3_r_ = randomize(thrust_map_);
  motor_o_0_r_ = randomize();
  motor_o_1_r_ = randomize();
  motor_o_2_r_ = randomize();
  motor_o_3_r_ = randomize();
  cd1_r_ = randomize(cd1_);
  cd3_r_ = randomize(cd3_);
  cdz_h_r_ = randomize(cdz_h_);

  return true;
}

bool QuadrotorDynamics::setModelParams(const Vector<Dynamic>& params){
  // not implemented yet
  
  return true;
}

bool QuadrotorDynamics::loadModelParams(const Vector<Dynamic>& params){
  // not implemented yet
  
  return true;
}

Scalar QuadrotorDynamics::randomize(){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  return MatrixXd::Random(1,1)(0,0)*var;
}

Scalar QuadrotorDynamics::randomize(Scalar x){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  return x + x*MatrixXd::Random(1,1)(0,0)*var;
}

Matrix<Dynamic,Dynamic> QuadrotorDynamics::randomize(Matrix<Dynamic,Dynamic> x){
  Scalar var = cfg_["training"]["param_var"].as<Scalar>();
  return x + x.cwiseProduct(MatrixXd::Random(x.rows(),x.cols()))*var;
}

}  // namespace flightlib