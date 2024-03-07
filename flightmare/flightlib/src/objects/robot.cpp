#include "flightlib/objects/robot.hpp"

namespace flightlib {

Robot::Robot(const YAML::Node &cfg_node){
	cfg_ = cfg_node;
	setDetCfg();
	init();
}  // namespace flightlib

Robot::Robot(const std::string &cfg_path){
	// check if configuration file exist
	if (!(file_exists(cfg_path))) {
		logger_.error("Configuration file %s does not exists.", cfg_path);
	}

	cfg_ = YAML::LoadFile(cfg_path);
	setDetCfg();

	// create robot dynamics and update the parameters

	init();
}

Robot::~Robot() {}

void Robot::init() {
	// robot type
	if (cfg_["robot"]["type"].as<std::string>() == "quadrotor") {
		dynamics_ptr_ = std::make_shared<QuadrotorDynamics>(cfg_);
		dynamics_det_ptr_ = std::make_shared<QuadrotorDynamics>(cfg_det_);
		size_ << 0.2, 0.2, 0.2;  // scaling for unity

	} else if (cfg_["robot"]["type"].as<std::string>() == "bixler") {
		dynamics_ptr_ = std::make_shared<BixlerDynamics>(cfg_);
		dynamics_det_ptr_ = std::make_shared<BixlerDynamics>(cfg_det_);
		size_ << 2.0, 2.0, 2.0;  // scaling for unity
		if (cfg_["robot"]["rotor_ctrl"].as<int>() == 1){
			logger_.error("This type of robot does not have a body rate low level controller. Put 'rotor_ctrl' = 0 in yaml file");
		}

	} else if (cfg_["robot"]["type"].as<std::string>() == "mpextra330") {
		dynamics_ptr_ = std::make_shared<mpextra330Dynamics>(cfg_);
		dynamics_det_ptr_ = std::make_shared<mpextra330Dynamics>(cfg_det_);
		size_ << 0.88, 0.88, 0.88;  // scaling for unity
		if (cfg_["robot"]["rotor_ctrl"].as<int>() == 1){
			logger_.error("This type of robot does not have a body rate low level controller. Put 'rotor_ctrl' = 0 in yaml file");
		}

	} else if (cfg_["robot"]["type"].as<std::string>() == "mpextra330ml") {
		dynamics_ptr_ = std::make_shared<mpextra330mlDynamics>(cfg_);
		dynamics_det_ptr_ = std::make_shared<mpextra330mlDynamics>(cfg_det_);
		size_ << 0.88, 0.88, 0.88;  // scaling for unity
		if (cfg_["robot"]["rotor_ctrl"].as<int>() == 1){
			logger_.error("This type of robot does not have a body rate low level controller. Put 'rotor_ctrl' = 0 in yaml file");
		}

	} else if (cfg_["robot"]["type"].as<std::string>() == "liseagle") {
		dynamics_ptr_ = std::make_shared<LiseagleDynamics>(cfg_);
		dynamics_det_ptr_ = std::make_shared<LiseagleDynamics>(cfg_det_);
		size_ << 1.5, 1.5, 1.5;  // scaling for unity
	} else {
		logger_.error("Not Valid robot type");
	}

	// low level controller
	if (cfg_["robot"]["ll_controller"].as<std::string>() == "simple") {
		ctrl_ptr_ = std::make_shared<LowLevelControllerSimple>(cfg_);
	} else if (cfg_["robot"]["ll_controller"].as<std::string>() == "custom") {
		ctrl_ptr_ = std::make_shared<LowLevelControllerCustom>(cfg_);
	} else {
		logger_.error("Not Valid low level controller");
	}
	
	// auxilary variables
	collision_ = false;
	world_box_ << ((Matrix<3, 2>() << -1000000, 1000000, -1000000, 1000000, 0, 1000000).finished());

	updateDynamics();
	reset();
}

bool Robot::setDetCfg() {
	cfg_det_ = Clone(cfg_); // deep copy
	cfg_det_["training"]["param_var"] = 0.0;
	cfg_det_["wind"]["mag_min"] = (cfg_det_["wind"]["mag_max"].as<Scalar>() + cfg_det_["wind"]["mag_min"].as<Scalar>())/2.0;
	cfg_det_["wind"]["mag_max"] = cfg_det_["wind"]["mag_min"].as<Scalar>();
	cfg_det_["turbulence"]["var"] = 0.0;
	cfg_det_["turbulence"]["on"] = false;

	return true;
}

bool Robot::reset() {
	state_.setZero();
	dynamics_ptr_->reset();
	dynamics_det_ptr_->reset();
	ctrl_ptr_->reset(state_);
	return true;
}

bool Robot::reset(const RobotState &state) {
	if (!state.valid()) return false;
	state_ = state;
	dynamics_ptr_->reset();
	dynamics_det_ptr_->reset();
	ctrl_ptr_->reset(state_);
	return true;
}

bool Robot::run(const Command &cmd, const Scalar ctl_dt) {
	if (!setCommand(cmd)) {
		logger_.error("Cannot Set Control Command");
		return false;
	}
	return run(ctl_dt);
}

bool Robot::run(const Scalar ctl_dt) {
	if (!state_.valid()) {
		logger_.error("Not Valid states");
		return false;
	}
	if (!cmd_.valid()) {
		logger_.error("Not Valid action");
		return false;
	}

	RobotState next_state = state_;

	// this does only something for custom controller
	ctrl_ptr_->updateRobotDynamics(state_, cmd_, wind_pred_);

	// time
	const Scalar max_dt = integrator_ptr_->dtMax();
	Scalar remain_ctl_dt = ctl_dt;

	consumption_ = 0;
	
	// simulation loop
	while (remain_ctl_dt > 0.0) {
		const Scalar sim_dt = std::min(remain_ctl_dt, max_dt);
		cmd_ll_ = ctrl_ptr_->run_ctrl(state_);

		Vector<6> wind_curl = wind_->getWindCurl(state_.p,sim_dt);
		Vector<7> accelerations = dynamics_ptr_->getAccelerations(state_, cmd_ll_, sim_dt, wind_curl);

		// if invalid state, stop simulation and set flag. Otherwise add accelerations to state and continue
		if (accelerations(6)){
			state_.modelInRange = false;
			break;
		}
		else{
			// compute body force and torque
			state_.a = accelerations.segment<3>(0);
			// state_.v = Vector<3>::Zero();
			state_.aa = accelerations.segment<3>(3);
			consumption_ += dynamics_ptr_->getConsumption();
			
			// actuator positions
			state_.x.segment(STATE::ACT, cmd_.raw_.rows()) = dynamics_ptr_->getActuator();

			// dynamics integration
			integrator_ptr_->step(state_.x, sim_dt, next_state.x);

			//
			state_.x = next_state.x;
			remain_ctl_dt -= sim_dt;
		}
	}

	// update state and sim time
	state_.t += ctl_dt;

	state_.qx.normalize();

	return true;
}

bool Robot::getState(RobotState *const state) const {
	if (!state_.valid()) return false;
	*state = state_;
	return true;
}

Scalar Robot::getForceMax() const { return dynamics_ptr_->getForceMax(); }

Vector<3> Robot::getOmegaMax() const { return dynamics_ptr_->getOmegaMax(); }

Scalar Robot::getVeluMax() const { return dynamics_ptr_->getVeluMax(); }

Vector<Dynamic> Robot::getRawMean() const { return dynamics_ptr_->getRawMean(); }

Vector<Dynamic> Robot::getRawStd() const { return dynamics_ptr_->getRawStd(); }

Vector<3> Robot::getSize(void) const { return size_; }

Vector<3> Robot::getPosition(void) const { return state_.p; }

Scalar Robot::getConsumption() const {return consumption_; }

bool Robot::getCollision() const { return collision_; }

Scalar Robot::getMass(void) { return dynamics_ptr_->getMass(); }

std::vector<Scalar> Robot::getAction() {
	// unity doesn't deal with eigen vector, only with std::vector
	std::vector<double> vec;
	vec.resize(cmd_ll_.size());
	Eigen::VectorXd::Map(&vec[0], cmd_ll_.size()) = cmd_ll_;
	return vec;
}

Vector<Dynamic> Robot::getActuator() { return dynamics_ptr_->getActuator(); }

std::vector<std::shared_ptr<RGBCamera>> Robot::getCameras(void) const { return rgb_cameras_; }

bool Robot::getCamera(const size_t cam_id, std::shared_ptr<RGBCamera> camera) const {
	if (cam_id <= rgb_cameras_.size()) {
		return false;
	}

	camera = rgb_cameras_[cam_id];
	return true;
}

int Robot::getNumCamera() const { return rgb_cameras_.size(); }

std::vector<std::shared_ptr<IMU>> Robot::getImus(void) const { return imus_; }

bool Robot::getImu(const size_t imu_id, std::shared_ptr<IMU> imu) const {
	if (imu_id <= imus_.size()) {
		return false;
	}

	imu = imus_[imu_id];
	return true;
}

int Robot::getNumImu() const { return imus_.size(); }

bool Robot::getImuMeasurement(Scalar imu_id, Ref<Vector<>> accelerometer_measurement,
                               Ref<Vector<>> gyroscope_measurement) {
	if (getNumImu() < imu_id - 1) {
		logger_.error("The robot has no IMU with corresponding imu_id");
		return false;
	}

	Matrix<4, 4> T_BI = imus_[imu_id]->getRelPoseBody2Imu();

	Vector<3> gravity_W;
	gravity_W[2] = imus_[imu_id]->getGravityMagnitude();

	Vector<3> linear_accelerations =
	    T_BI.block(0, 0, 3, 3) * (state_.a + state_.rotEuclWorldToEuclBodyFrame(gravity_W));

	imus_[imu_id]->setLinearAccelerationMeasurement(linear_accelerations);
	imus_[imu_id]->setAngularVelocityMeasurement(state_.w);

	Scalar dt = state_.t - imus_[imu_id]->getLastMeasurementTime();
	imus_[imu_id]->addNoiseToImu(dt);
	imus_[imu_id]->setLastMeasurementTime(state_.t);

	accelerometer_measurement = imus_[imu_id]->getLinearAccelerationMeasurement();
	gyroscope_measurement = imus_[imu_id]->getAngularVelocityMeasurement();

	return true;
}

std::vector<std::shared_ptr<Airspeed>> Robot::getAirspeeds(void) const { return airspeeds_; }

bool Robot::getAirspeed(const size_t airspeed_id, std::shared_ptr<Airspeed> airspeed) const {
	if (airspeed_id <= airspeeds_.size()) {
		return false;
	}

	airspeed = airspeeds_[airspeed_id];
	return true;
}

int Robot::getNumAirspeed() const { return airspeeds_.size(); }

Scalar Robot::getAirspeedMeasurement(Scalar airspeed_id) {
	if (getNumAirspeed() < airspeed_id - 1) {
		logger_.error("The robot has no Airspeed Sensor with corresponding airspeed_id");
		return false;
	}

  Scalar u_rel;
  Vector<3> wind = wind_->getCurrentWind(); 
  Scalar measurement;
  if (cfg_["robot"]["type"].as<std::string>() == "quadrotor") {
		u_rel = state_.rotEuclWorldToEuclBodyFrame(state_.v - wind)(0);
    measurement = airspeeds_[airspeed_id]->getMeasurement(u_rel);  // quadrotor velocity is in global frame
	} else {
    u_rel = (state_.v - state_.rotEuclWorldToEuclBodyFrame(wind))(0);
		measurement = airspeeds_[airspeed_id]->getMeasurement(u_rel);  // fixed wing velocity is in body frame
	}

	return measurement;
}

Vector<3> Robot::getWind(void) {
	return wind_->getCurrentWind();
}

Scalar Robot::getWindVar(void) {
	return wind_->getWindVar();
}

bool Robot::setRobotState(const RobotState &state) {
	if (!state.valid()) return false;
	state_ = state;
	return true;
}

bool Robot::setCommand(const Command &cmd) {
	if (!cmd.valid()) {
		logger_.error("Not Valid action");
		return false;
	}
	cmd_ = cmd;

	if (std::isfinite(cmd_.collective_thrust_))
		cmd_.collective_thrust_ =
		dynamics_ptr_->clampCollectiveThrust(cmd_.collective_thrust_);

	if (cmd_.omega_.allFinite())
		cmd_.omega_ = dynamics_ptr_->clampBodyrates(cmd_.omega_);

	if (cmd_.raw_.allFinite()) cmd_.raw_ = dynamics_ptr_->clampRaw(cmd_.raw_);
	
	return true;
}

bool Robot::setLLDynamics(const RobotState &state, const Command &cmd) {
	if (!state.valid()) return false;
		ctrl_ptr_->updateRobotDynamics(state, cmd, wind_pred_);
	return true;
}

bool Robot::setLLOffset(const Vector<Dynamic> offset) {
	ctrl_ptr_->setOffset(offset);
	return true;
}

bool Robot::callLLRun(const RobotState &state, Ref<Vector<Dynamic>> act, std::map<std::string, Eigen::VectorXd>& info) {
	if (!state.valid()) return false;
	Vector<Dynamic> actions_actu = ctrl_ptr_->run_ctrl(state);
	Vector<Dynamic> mean = dynamics_ptr_->getRawMean();
	Vector<Dynamic> std = dynamics_ptr_->getRawStd();
	act = (actions_actu.array() - mean.array()) / std.array();
	
	return ctrl_ptr_->getInfo(info);
}

bool Robot::setLLGains(Ref<Vector<Dynamic>> gains, const int idx) {
	ctrl_ptr_->setGains(gains, idx);
	return true;
}

bool Robot::storeLLReservoir(){
	return ctrl_ptr_->storeReservoir();
}

bool Robot::restoreLLReservoir(){
	return ctrl_ptr_->restoreReservoir();
}

bool Robot::setActuator(const Vector<Dynamic>& actuator) {
	dynamics_ptr_->setActuator(actuator);
	return true;
}

void Robot::setSize(const Ref<Vector<3>> size) { size_ = size; }

void Robot::setCollision(const bool collision) { collision_ = collision; }

bool Robot::updateDynamics() {
	if (!dynamics_ptr_->valid()) {
		std::cout << "[Robot] dynamics is not valid!" << std::endl;
		return false;
	}

	if (cfg_["simulation"]["integrator"].as<std::string>() == "euler") {
		integrator_ptr_ = std::make_unique<IntegratorEuler>(dynamics_ptr_->getDynamicsFunction(), 2.5e-3);
	} else if (cfg_["simulation"]["integrator"].as<std::string>() == "rk4") {
		integrator_ptr_ = std::make_unique<IntegratorRK4>(dynamics_ptr_->getDynamicsFunction(), 2.5e-3);
	} else {
		std::cout << "[Robot] integrator is not valid!" << std::endl;
		return false;
	}
	
	ctrl_ptr_->setRobotDynamics(dynamics_det_ptr_, integrator_ptr_);
	
	// B_allocation_ = dynamics_ptr_->getAllocationMatrix();
	// B_allocation_inv_ << B_allocation_.inverse();
	return true;
}

bool Robot::setWorldBox(const Ref<Matrix<3, 2>> box) {
	if (box(0, 0) >= box(0, 1) || box(1, 0) >= box(1, 1) || box(2, 0) >= box(2, 1)) {
		return false;
	}
	world_box_ = box;
	return true;
}

bool Robot::constrainInWorldBox(const RobotState &old_state) {
	if (!old_state.valid()) return false;

	// violate world box constraint in the x-axis
	if (state_.x(STATE::POSX) <= world_box_(0, 0) || state_.x(STATE::POSX) >= world_box_(0, 1)) {
		state_.x(STATE::POSX) = old_state.x(STATE::POSX);
		state_.x(STATE::VELX) = 0.0;
	}

	// violate world box constraint in the y-axis
	if (state_.x(STATE::POSY) <= world_box_(1, 0) || state_.x(STATE::POSY) >= world_box_(1, 1)) {
		state_.x(STATE::POSY) = old_state.x(STATE::POSY);
		state_.x(STATE::VELY) = 0.0;
	}

	// violate world box constraint in the z-axis
	if (state_.x(STATE::POSZ) <= world_box_(2, 0) || state_.x(STATE::POSZ) >= world_box_(2, 1)) {
		//
		state_.x(STATE::POSZ) = old_state.x(STATE::POSZ);

		// reset velocity to zero
		state_.x(STATE::VELZ) = 0.0;

		// reset acceleration to zero
		state_.a << 0.0, 0.0, 0.0;
		
		// reset angular velocity to zero
		state_.w << 0.0, 0.0, 0.0;

		// reset valid state to true
		state_.modelInRange = true;
	}
	return true;
}

bool Robot::addWind(std::shared_ptr<Wind> wind) {
	wind_ = wind;
	wind_pred_ = wind_->getWindPred();
	return true;
}

bool Robot::addRGBCamera(std::shared_ptr<RGBCamera> camera) {
	rgb_cameras_.push_back(camera);
	return true;
}

bool Robot::addImu(std::shared_ptr<IMU> imu) {
	imus_.push_back(imu);
	return true;
}

bool Robot::addAirspeed(std::shared_ptr<Airspeed> airspeed) {
	airspeeds_.push_back(airspeed);
	return true;
}

bool Robot::setModelParams(Ref<Vector<>> params){
	return dynamics_ptr_->setModelParams(params); 
}

}  // namespace flightlib