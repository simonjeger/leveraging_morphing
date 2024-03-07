#include "flightlib/envs/robot_env/robot_env.hpp"

namespace flightlib {

RobotEnv::RobotEnv() : RobotEnv(getenv("FLIGHTMARE_PATH") + std::string("/flightpy/configs/control/config.yaml"), 0) {}

RobotEnv::RobotEnv(const std::string &cfg_path, const int env_id) : EnvBase() {
	// check if configuration file exist
	if (!(file_exists(cfg_path))) {
		logger_.error("Configuration file %s does not exists.", cfg_path);
	}
	// load configuration file
	cfg_ = YAML::LoadFile(cfg_path);
	//
	init();
	env_id_ = env_id;
}

RobotEnv::RobotEnv(const YAML::Node &cfg_node, const int env_id) : EnvBase(), cfg_(cfg_node) {
	//
	init();
	env_id_ = env_id;
}

void RobotEnv::init() {
	obs_dim_ = 0;

	// define input and output dimension for the environment
	if (cfg_["robot"]["rotor_ctrl"].as<int>() == 1){
		act_dim_ = 4;
	} 
	else if (cfg_["robot"]["type"].as<std::string>() == "quadrotor") {
		act_dim_ = 4;
	} else if (cfg_["robot"]["type"].as<std::string>() == "bixler") {
		act_dim_ = 4;
	} else if (cfg_["robot"]["type"].as<std::string>() == "mpextra330") {
		act_dim_ = 4;
	} else if (cfg_["robot"]["type"].as<std::string>() == "mpextra330ml") {
		act_dim_ = 4;
	} else if (cfg_["robot"]["type"].as<std::string>() == "liseagle") {
		act_dim_ = 8;
	} else {
		logger_.error("Not Valid robot type");
	}

	obs_dim_ += cfg_["observation"]["traj_obs_n"].as<int>() * 3;  // rel pos [3*traj_obs_n]
	
	if (cfg_["observation"]["ori"].as<bool>()) {
		obs_dim_ += 9;  // orientation. [9]
	}

	if (cfg_["observation"]["lin_vel"].as<bool>()) {
		obs_dim_ += 3;  // lin. vel. [3]
	}

	if (cfg_["observation"]["ang_vel"].as<bool>()) {
		obs_dim_ += 3;  // ang. vel. [3]
	}
	
	if (cfg_["observation"]["lin_acc"].as<bool>()) {
		obs_dim_ += 3;  // lin. acc. [3]
	}

	if (cfg_["observation"]["ang_acc"].as<bool>()) {
		obs_dim_ += 3;  // ang. acc. [3]
	}
	
	if (cfg_["observation"]["actuator"].as<bool>()) {
		obs_dim_ += act_dim_;  // act [act_dim]
	}

	if (cfg_["observation"]["wind"].as<bool>()) {
		obs_dim_ += 3;  // wind [3]
	}

	if (cfg_["observation"]["curl"].as<bool>()) {
		obs_dim_ += 3;  // curl [3]
	}

	if (cfg_["observation"]["wind_est"].as<bool>()) {
		obs_dim_ += 3;  // wind [3]
	}

	if (cfg_["observation"]["curl_est"].as<bool>()) {
		obs_dim_ += 3;  // curl [3]
	}

	if (cfg_["observation"]["airspeed"].as<bool>()) {
		obs_dim_ += 1;  // airspeed [1]
	}

	act_mean_ = Eigen::MatrixXd::Zero(act_dim_, 1);
	act_std_ = Eigen::MatrixXd::Ones(act_dim_, 1);
	obs_mean_ = Eigen::MatrixXd::Zero(obs_dim_, 1);
	obs_std_ = Eigen::MatrixXd::Ones(obs_dim_, 1);

	// check if some actuators are not active
	inactivity_ = Eigen::MatrixXd::Ones(act_dim_, 1);
	for (int i=0;i<act_dim_;i++){
		if (!cfg_["actuator"][std::to_string(i)].as<bool>()){
		inactivity_(i) = 0;
		}
	}

	//
	robot_ptr_ = std::make_shared<Robot>(cfg_);
	// update dynamics
	robot_ptr_->updateDynamics();

	// define a bounding box {xmin, xmax, ymin, ymax, zmin, zmax}
	world_box_ = Matrix<3, 2>((cfg_["environment"]["world_box"].as<std::vector<Scalar>>()).data());
	if (!robot_ptr_->setWorldBox(world_box_)) {
		logger_.error("cannot set world box");
	}

	rew_dim_ = 0;

	// load parameters and set goal_pos_ and goal_obs_
	loadParam(cfg_);
	setNextGoalPos();

	// add wind
	wind_ = std::make_shared<Wind>(cfg_);
	robot_ptr_->addWind(wind_);

	// add camera
	if (use_camera_) {
		rgb_camera_ = std::make_shared<RGBCamera>();
		if (!configCamera(cfg_, rgb_camera_)) {
			logger_.error("Cannot config RGB Camera. Something wrong with the config file");
		}
		robot_ptr_->addRGBCamera(rgb_camera_);
		//
		img_width_ = rgb_camera_->getWidth();
		img_height_ = rgb_camera_->getHeight();
		rgb_img_ = cv::Mat::zeros(img_height_, img_width_, CV_MAKETYPE(CV_8U, rgb_camera_->getChannels()));
		depth_img_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
	}

	// add IMU
	if (use_imu_) {
		imu_ = std::make_shared<IMU>();
		if (!configImu(cfg_, imu_)) {
			logger_.error("Cannot config IMU. Something wrong with the config file");
		}
		robot_ptr_->addImu(imu_);
	}

	// add Airspeed sensor
	if (use_airspeed_) {
		airspeed_ = std::make_shared<Airspeed>();
		if (!configAirspeed(cfg_, airspeed_)) {
			logger_.error("Cannot config Airspeed Sensor. Something wrong with the config file");
		}
		robot_ptr_->addAirspeed(airspeed_);
	}

	// use single rotor control or bodyrate control
	getActMean(act_mean_);
	getActStd(act_std_);

	// state estimation
	pos_std_ = cfg_["state_estimation"]["pos_std"].as<Scalar>();
    vel_std_ = cfg_["state_estimation"]["vel_std"].as<Scalar>();
    acc_std_ = cfg_["state_estimation"]["acc_std"].as<Scalar>();
    quat_std_ = cfg_["state_estimation"]["att_std"].as<Scalar>();
    omega_std_ = cfg_["state_estimation"]["omega_std"].as<Scalar>();
    aac_std_ = cfg_["state_estimation"]["aac_std"].as<Scalar>();
}

RobotEnv::~RobotEnv() {}

bool RobotEnv::reset(Ref<Vector<>> obs) {
	robot_state_.setZero();
	pi_act_.setZero();

	// reset position
	robot_state_.x(STATE::POSX) = start_pos_[0] + uniform_dist_(random_gen_) * start_pos_var_;
	robot_state_.x(STATE::POSY) = start_pos_[1] + uniform_dist_(random_gen_) * start_pos_var_;
	robot_state_.x(STATE::POSZ) = start_pos_[2] + uniform_dist_(random_gen_) * start_pos_var_;
	if (robot_state_.x(STATE::POSZ) < -0.0) robot_state_.x(STATE::POSZ) = -robot_state_.x(STATE::POSZ);
	// reset linear velocity
	robot_state_.x(STATE::VELX) = start_lin_vel_[0] + uniform_dist_(random_gen_) * start_lin_vel_var_;
	robot_state_.x(STATE::VELY) = start_lin_vel_[1] + uniform_dist_(random_gen_) * start_lin_vel_var_;
	robot_state_.x(STATE::VELZ) = start_lin_vel_[2] + uniform_dist_(random_gen_) * start_lin_vel_var_;
	// reset orientation
	robot_state_.x(STATE::ATTW) = start_rot_[0] + uniform_dist_(random_gen_) * start_rot_var_;
	robot_state_.x(STATE::ATTX) = start_rot_[1] + uniform_dist_(random_gen_) * start_rot_var_;
	robot_state_.x(STATE::ATTY) = start_rot_[2] + uniform_dist_(random_gen_) * start_rot_var_;
	robot_state_.x(STATE::ATTZ) = start_rot_[3] + uniform_dist_(random_gen_) * start_rot_var_;
	// reset angular velocity
	robot_state_.x(STATE::OMEX) = start_omega_[0] + uniform_dist_(random_gen_) * start_omega_var_;
	robot_state_.x(STATE::OMEY) = start_omega_[1] + uniform_dist_(random_gen_) * start_omega_var_;
	robot_state_.x(STATE::OMEZ) = start_omega_[2] + uniform_dist_(random_gen_) * start_omega_var_;
	robot_state_.qx /= robot_state_.qx.norm();

	// reset actuators to zero
	robot_state_.x.segment(STATE::ACT, STATE::NACT).setZero();

	// reset robot with random states
	robot_ptr_->reset(robot_state_);
	
	// reset command
	if (rotor_ctrl_ == 0) {
		cmd_.setCmdMode(0);
	} else if (rotor_ctrl_ == 1) {
		cmd_.setCmdMode(1);
		}
	cmd_.reset(robot_ptr_->getRawMean());

	// reset goal
	if (setRandomTraj_) {
		setGoalTrajectory();
	}
	goal_idx_ = 0;
	goal_idx_old_ = 0;

	// reset wind
	wind_->reset();

	// reset sensor
	//imu_->reset();
	airspeed_->reset();

	setNextGoalPos();

	// obtain observations
	getObs(obs);

	return true;
}

bool RobotEnv::getObs(Ref<Vector<>> obs) {
	if (obs.size() != obs_dim_) {
		logger_.error("Observation dimension mismatch. %d != %d", obs.size(), obs_dim_);
		return false;
	}
	
	// add noise to the observation
	robot_ptr_->getState(&robot_state_);
	RobotState est_state = robot_state_;
	est_state.p += MatrixXd::Random(3,1)*pos_std_;
    est_state.v += MatrixXd::Random(3,1)*vel_std_;
	est_state.a += MatrixXd::Random(3,1)*acc_std_;
    est_state.qx += MatrixXd::Random(4,1)*quat_std_;
    est_state.w += MatrixXd::Random(3,1)*omega_std_;
	est_state.aa += MatrixXd::Random(3,1)*aac_std_;

	// convert quaternion to euler angle
	Vector<9> ori = Map<Vector<>>(est_state.R().data(), est_state.R().size());
	
	Matrix<3, Dynamic> goal = est_state.moveEuclWorldToEuclBodyFrame(goal_obs_);

	int size_obs = goal.size();
	obs.segment(0, size_obs) << Map<Vector<>>(goal.data(), goal.size());

	if (cfg_["observation"]["ori"].as<bool>()) {
		obs.segment(size_obs, 9) = ori;
		size_obs += 9;
	}
	
	if (cfg_["observation"]["lin_vel"].as<bool>()) {
		obs.segment(size_obs, 3) = est_state.x.segment(STATE::VEL, STATE::NVEL);
		size_obs += 3;
	}
	
	if (cfg_["observation"]["ang_vel"].as<bool>()) {
		obs.segment(size_obs, 3) = est_state.x.segment(STATE::OME, STATE::NOME);
		size_obs += 3;
	}

	if (cfg_["observation"]["lin_acc"].as<bool>()) {
		obs.segment(size_obs, 3) = est_state.x.segment(STATE::ACC, STATE::NACC);
		size_obs += 3;
	}

	if (cfg_["observation"]["ang_acc"].as<bool>()) {
		obs.segment(size_obs, 3) = est_state.x.segment(STATE::AAC, STATE::NAAC);
		size_obs += 3;
	}
	
	if (cfg_["observation"]["actuator"].as<bool>()) {
		obs.segment(size_obs, act_dim_) = est_state.x.segment(STATE::ACT, act_dim_);
		size_obs += act_dim_;
	}
	
	if (cfg_["observation"]["wind"].as<bool>()) {
		obs.segment(size_obs, 3) = robot_state_.rotEuclWorldToEuclBodyFrame(wind_->getCurrentWind());
		size_obs += 3;
	}

	if (cfg_["observation"]["curl"].as<bool>()) {
		obs.segment(size_obs, 3)  = robot_state_.rotEuclWorldToEuclBodyFrame(wind_->getCurrentCurl());
		size_obs += 3;
	}

	if (cfg_["observation"]["wind_est"].as<bool>()) {
		obs.segment(size_obs, 3)  = robot_state_.rotEuclWorldToEuclBodyFrame(wind_->getWindPred());
		size_obs += 3;
	}

	if (cfg_["observation"]["curl_est"].as<bool>()) {
		obs.segment(size_obs, 3) = Vector<3>::Zero();
		size_obs += 3;
	}

	if (cfg_["observation"]["airspeed"].as<bool>()) {
		obs(size_obs) = robot_ptr_->getAirspeedMeasurement(0.0);
		size_obs += 1;
	}

	return true;
}

bool RobotEnv::step(const Ref<Vector<>> act, Scalar dt, Ref<Vector<>> obs, Ref<Vector<>> reward) {
	//takes the current action and current state and returns the next timestep and observation

	//in case the timestep is not given, use the default timestep
	if (dt != -1.0) {
		sim_dt_ = dt;
	}
	
	pos_old_ = robot_state_.p;
	goal_idx_old_ = goal_idx_;

	// act is [-1.0, 1.0]
	if (!act.allFinite() || act.rows() != act_dim_ || rew_dim_ != reward.rows())
		return false;

	if (rotor_ctrl_ == 0) {
		act_ina_ = act.cwiseProduct(inactivity_);
		pi_act_ = act_ina_.cwiseProduct(act_std_) + act_mean_;
	} else{
		pi_act_ = act.cwiseProduct(act_std_) + act_mean_;
	}

	cmd_.t_ += sim_dt_;
	robot_state_.t += sim_dt_;
	
	if (rotor_ctrl_ == 0) {
		cmd_.setRaw(pi_act_);
	} else if (rotor_ctrl_ == 1) {
		cmd_.setThrustRates(pi_act_);
	}

	// simulate robot
	robot_ptr_->run(cmd_, sim_dt_);

	// update observations
	getObs(obs);

	// get reward
	getReward(reward);
	
	// Set new goal from trajectory everytime the current one is reached
	if ((robot_state_.p - goal_pos_).norm() < goal_acc_r_) {
		setNextGoalPos();
	}

	return true;
}

bool RobotEnv::setRobotState(const Ref<Vector<>> state, Ref<Vector<>> obs, Ref<Vector<>> reward) {
	pos_old_ = robot_state_.p;
	goal_idx_old_ = goal_idx_;

	// overwrite state of robot with measurements
	robot_ptr_->getState(&robot_state_);
	robot_state_.p = state.segment<3>(0);
	robot_state_.qx = state.segment<4>(3);
	robot_state_.v = state.segment<3>(7);
	robot_state_.w = state.segment<3>(10);
	robot_state_.a = state.segment<3>(13);
	robot_state_.aa = state.segment<3>(16);
	robot_ptr_->setRobotState(robot_state_);

	// update observations
	getObs(obs);

	// get reward
	getReward(reward);

	// Set new goal from trajectory everytime the current one is reached
	if ((robot_state_.p - goal_pos_).norm() < goal_acc_r_) {
		setNextGoalPos();
	}

	return true;
}

bool RobotEnv::setModelParams(const Ref<Vector<>> params) {
	// set the state according to the data
	robot_ptr_->setModelParams(params);

	return true;
}

bool RobotEnv::setLLDynamics(const Ref<Vector<>> state, const Ref<Vector<>> cmd){
	RobotState state_temp = robot_state_;
	state_temp.p = state.segment<3>(0);
	state_temp.qx = state.segment<4>(3);
	state_temp.v = state.segment<3>(7);
	state_temp.w = state.segment<3>(10);
	state_temp.a = state.segment<3>(13);

	Command cmd_ll = cmd_;
	
	if (!cmd_ll.isRaw()){
		cmd_ll.setThrustRates(cmd.cwiseProduct(act_std_) + act_mean_);
	} else {
		cmd_ll.setRaw(cmd.cwiseProduct(act_std_) + act_mean_);
	}
	robot_ptr_->setLLDynamics(state_temp, cmd_ll);
	return true;
}

bool RobotEnv::setLLOffset(const Ref<Vector<>> lastAction){
	return robot_ptr_->setLLOffset(lastAction);
}

bool RobotEnv::callLLRun(const Ref<Vector<>> state, Ref<Vector<>> act, 
	Ref<Vector<>> gt, Ref<Vector<>> ref, Ref<Vector<>> proj_matrix){
	RobotState state_temp = robot_state_;
	state_temp.p = state.segment<3>(0);
	state_temp.qx = state.segment<4>(3);
	state_temp.v = state.segment<3>(7);
	state_temp.w = state.segment<3>(10);
	state_temp.a = state.segment<3>(13);
	state_temp.aa = state.segment<3>(16);
	
	std::map<std::string, Eigen::VectorXd> info;
	robot_ptr_->callLLRun(state_temp, act, info);
	gt.segment(0,3) = info["gt_omega"];
  	gt.segment(3,3) = info["gt_vel"];
  	ref.segment(0,3) = info["ref_omega"];
  	ref.segment(3,3) = info["ref_vel"];
  	proj_matrix = info["proj_matrix"];
	return true;
}

bool RobotEnv::setLLGains(const Ref<Vector<>> gains, const int idx){
	robot_ptr_->setLLGains(gains, idx);
	return true;
}

bool RobotEnv::storeLLReservoir(){
	return robot_ptr_->storeLLReservoir();
}

bool RobotEnv::restoreLLReservoir(){
	return robot_ptr_->storeLLReservoir();
}

bool RobotEnv::setRobotActuator(Ref<Vector<>> actuator) const {
	robot_ptr_->setActuator(actuator);
    return true;
}

bool RobotEnv::setTurbulence(Ref<Matrix<Dynamic,3>> turbulence) {
	wind_->setTurbulence(turbulence);
	return true;
}

bool RobotEnv::getReward(Ref<Vector<>> reward){
	// ---------------------- reward function design
	Scalar pos_reward = 0;
	Scalar pro_reward = 0;
	Scalar done_reward = 0;

	// - orientation tracking (linear)
	const Scalar ori_reward = ori_coeff_ * robot_state_.e().norm() * sim_dt_;
	
	// - yaw tracking (linear)
	const Scalar yaw_reward = yaw_coeff_ * std::abs(robot_state_.e()(2)) * sim_dt_;

	// - linear velocity tracking (linear)
	const Scalar lin_vel_reward = lin_vel_coeff_ * robot_state_.v.norm() * sim_dt_;

	// - angular velocity tracking (linear)
	const Scalar ang_vel_reward = ang_vel_coeff_ * robot_state_.w.norm() * sim_dt_;

	// - linear velocity tracking (linear)
	const Scalar lin_acc_reward = lin_acc_coeff_ * robot_state_.a.norm() * sim_dt_;

	// - angular velocity tracking (linear)
	const Scalar ang_acc_reward = ang_acc_coeff_ * robot_state_.aa.norm() * sim_dt_;

	// - actuator position (linear)
	const Scalar action_reward = action_coeff_*act_ina_.norm() * sim_dt_;

	// - energy cost (linear)
	const Scalar energy_reward = energy_coeff_*robot_ptr_->getConsumption() * sim_dt_;

	if ((goal_obs_.cols() == 1) || goal_idx_ == goal_traj_.cols()) {
		// - position tracking (ln)
		pos_reward = pos_coeff_*ln((robot_state_.p - goal_pos_).norm()) * sim_dt_;

		// - time spend in the done state is rewarded (const)
		done_reward = done_coeff_;

	} else {
		// - position tracking (ln)
		pos_reward = pos_coeff_ * ln(distanceToPath(robot_state_.p,
									goal_traj_.block<3, 1>(0, goal_idx_ - 1),
									goal_traj_.block<3, 1>(0, goal_idx_))) * sim_dt_;

		// - progress tracking (linear)
		Scalar s0;
		if (goal_idx_ == goal_idx_old_) {
		s0 =
			progressOnPath(pos_old_, goal_traj_.block<3, 1>(0, goal_idx_old_ - 1),
						goal_traj_.block<3, 1>(0, goal_idx_old_));
		} else {
		s0 = 0;
		}
		Scalar s1 =
		progressOnPath(robot_state_.p, goal_traj_.block<3, 1>(0, goal_idx_ - 1),
						goal_traj_.block<3, 1>(0, goal_idx_));

		pro_reward = pro_coeff_ * (s1 - s0) / sim_dt_ * sim_dt_;  // progress per time
	}

	const Scalar total_reward = pos_reward + ori_reward + yaw_reward + lin_vel_reward +
								ang_vel_reward + lin_acc_reward + ang_acc_reward + pro_reward + done_reward + 
								action_reward + energy_reward;
	reward << pos_reward, ori_reward, yaw_reward, lin_vel_reward, ang_vel_reward, lin_acc_reward, 
		ang_acc_reward, pro_reward, done_reward, action_reward, energy_reward, total_reward;
	
	return true;
}

Scalar RobotEnv::progressOnPath(Vector<3> pos, Vector<3> g0, Vector<3> g1) {
	Vector<3> a = (pos - g0);
	Vector<3> b = (g1 - g0);
	Scalar num = a.transpose() * b;
	Scalar den = b.norm();
	return std::min(num / den,
	                den);  // over-shooting the target should not be rewarded
}

Scalar RobotEnv::distanceToPath(Vector<3> pos, Vector<3> g0, Vector<3> g1) {
	Vector<3> a = (pos - g0);
	Vector<3> b = (g1 - g0);
	Scalar num = a.transpose() * b;
	Scalar den = b.norm() * b.norm();
	Vector<3> proj = num / den * b;
	Scalar dist;
	if (proj.norm() <= b.norm()) {
		dist = (a - proj).norm();
	} else {
		dist = (pos - g1).norm();
	}
	return dist;
}

Scalar RobotEnv::ln(Scalar x){
	return log(abs(x)+1);
}

bool RobotEnv::isTerminalState(Scalar &reward) {
	bool out_of_bounds = false;
	if (robot_state_.x(STATE::POSX) <= world_box_(0,0) || robot_state_.x(STATE::POSX) >= world_box_(0,1)){
		out_of_bounds = true;
	}
	if (robot_state_.x(STATE::POSY) <= world_box_(1,0) || robot_state_.x(STATE::POSY) >= world_box_(1,1)){
		out_of_bounds = true;
	}
	if (robot_state_.x(STATE::POSZ) <= world_box_(2,0) || robot_state_.x(STATE::POSZ) >= world_box_(2,1)){
		out_of_bounds = true;
	}

	if (out_of_bounds || !robot_state_.modelInRange) {
		reward = -1.0;
		return true;
	}

	if (cmd_.t_ >= max_t_ - sim_dt_) {
		reward = 0.0;
		return true;
	}

	return false;
}

bool RobotEnv::getActMean(Ref<Vector<>> act){
	if (rotor_ctrl_ == 0) {
		act = robot_ptr_->getRawMean();
	} else if (rotor_ctrl_ == 1) {
		if (cfg_["robot"]["type"].as<std::string>() == "quadrotor"){
			Scalar max_force = robot_ptr_->getForceMax();
			act<< (max_force / robot_ptr_->getMass()) / 2.0, 0.0, 0.0, 0.0;
		}
		else{
			act << robot_ptr_->getRawMean()(0), 0.0, 0.0, 0.0;
		}
	}
	return true;
}

bool RobotEnv::getActStd(Ref<Vector<>> std){
	if (rotor_ctrl_ == 0) {
		std = robot_ptr_->getRawStd();
	} else if (rotor_ctrl_ == 1) {
		if (cfg_["robot"]["type"].as<std::string>() == "quadrotor"){
			Scalar max_force = robot_ptr_->getForceMax();
			Vector<3> max_omega = robot_ptr_->getOmegaMax();
			std << (max_force / robot_ptr_->getMass()) / 2, max_omega.x(), max_omega.y(), max_omega.z();
		}
		else{
			Vector<3> max_omega = robot_ptr_->getOmegaMax();
			std << robot_ptr_->getRawStd()(0), max_omega.x(), max_omega.y(), max_omega.z();
		}
		
	}
	return true;
}

bool RobotEnv::getRobotAct(Ref<Vector<>> act) const {
	if (cmd_.t_ >= 0.0 && pi_act_.allFinite() && (act.size() == pi_act_.size())) {
    	act = pi_act_;
    return true;
  }
  return false;
}

bool RobotEnv::getRobotActuator(Ref<Vector<>> actuator) const {
	if (cmd_.t_ >= 0.0 && pi_act_.allFinite() && (actuator.size() == pi_act_.size())) {
    	actuator = robot_ptr_->getActuator();
    return true;
  }
  return false;
}

bool RobotEnv::getRobotState(Ref<Vector<>> state) const {
	if (robot_state_.t >= 0.0 && (state.rows() == RobotState::NDYM)) {
		state << robot_state_.x;
		return true;
	}
	logger_.error("Get Robot state failed.");
	return false;
}

Scalar RobotEnv::getRobotTimestamp() const {
	if (robot_state_.t >= 0.0) {
		return robot_state_.t;
	}
	logger_.error("Get Robot Timestamp failed.");
	return 0.0;
}


Scalar RobotEnv::getRobotEnergy() const {
	if (robot_state_.t >= 0.0) {
		return robot_ptr_->getConsumption();
	}
	logger_.error("Get Robot Energy failed.");
	return 0.0;
}

bool RobotEnv::getWindCurl(Ref<Vector<>> wc) const {
	if (robot_state_.t >= 0.0 && (wc.rows() == 6)) {
		wc << wind_->getCurrentWind(), wind_->getCurrentCurl();
		return true;
	}
	logger_.error("Get WindCurl failed.");
	return false;
}

bool RobotEnv::getWindCurlEst(Ref<Vector<>> wce) const {
	if (robot_state_.t >= 0.0 && (wce.rows() == 6)) {
		wce << wind_->getWindPred(), Vector<3>::Zero(); //replace with actual estimate
		return true;
	}
	logger_.error("Get WindCurl estimate failed.");
	return false;
}

bool RobotEnv::getGoalState(Ref<Vector<>> goal_state) const {
	goal_state << goal_pos_;
	return true;
}

bool RobotEnv::getDepthImage(Ref<DepthImgVector<>> depth_img) {
	if (!rgb_camera_ || !rgb_camera_->getEnabledLayers()[0]) {
		logger_.error(
		    "No RGB Camera or depth map is not enabled. Cannot retrieve depth "
		    "images.");
		return false;
	}
	rgb_camera_->getDepthMap(depth_img_);

	depth_img = Map<DepthImgVector<>>((float_t *)depth_img_.data, depth_img_.rows * depth_img_.cols);
	return true;
}

bool RobotEnv::getImage(Ref<ImgVector<>> img, const bool rgb) {
	if (!rgb_camera_) {
		logger_.error("No Camera! Cannot retrieve Images.");
		return false;
	}

	rgb_camera_->getRGBImage(rgb_img_);

	if (rgb_img_.rows != img_height_ || rgb_img_.cols != img_width_) {
		logger_.error(
		    "Image resolution mismatch. Aborting.. Image rows %d != %d, Image cols "
		    "%d != %d",
		    rgb_img_.rows, img_height_, rgb_img_.cols, img_width_);
		return false;
	}

	if (!rgb) {
		// converting rgb image to gray image
		cvtColor(rgb_img_, gray_img_, CV_RGB2GRAY);
		// map cv::Mat data to Eiegn::Vector
		img = Map<ImgVector<>>(gray_img_.data, gray_img_.rows * gray_img_.cols);
	} else {
		img = Map<ImgVector<>>(rgb_img_.data, rgb_img_.rows * rgb_img_.cols * rgb_camera_->getChannels());
	}
	return true;
}

bool RobotEnv::loadParam(const YAML::Node &cfg) {
	if (cfg["simulation"]) {
		sim_dt_ = cfg["simulation"]["ctr_dt"].as<Scalar>();
		max_t_ = cfg["simulation"]["max_t"].as<Scalar>();
		rotor_ctrl_ = cfg["robot"]["rotor_ctrl"].as<int>();
  	} else {
    	logger_.error("Cannot load [robot_env] parameters");
    	return false;
  	}

  	if (cfg["rewards"]) {
		// load reinforcement learning related parameters
		pos_coeff_ = cfg["rewards"]["pos_coeff"].as<Scalar>();
		ori_coeff_ = cfg["rewards"]["ori_coeff"].as<Scalar>();
		yaw_coeff_ = cfg["rewards"]["yaw_coeff"].as<Scalar>();
		lin_vel_coeff_ = cfg["rewards"]["lin_vel_coeff"].as<Scalar>();
		ang_vel_coeff_ = cfg["rewards"]["ang_vel_coeff"].as<Scalar>();
		lin_acc_coeff_ = cfg["rewards"]["lin_acc_coeff"].as<Scalar>();
		ang_acc_coeff_ = cfg["rewards"]["ang_acc_coeff"].as<Scalar>();
		pro_coeff_ = cfg["rewards"]["pro_coeff"].as<Scalar>();
		done_coeff_ = cfg["rewards"]["done_coeff"].as<Scalar>();
		energy_coeff_ = cfg["rewards"]["energy_coeff"].as<Scalar>();
		action_coeff_ = cfg["rewards"]["action_coeff"].as<Scalar>();
		// load reward settings
		reward_names_ = cfg["rewards"]["names"].as<std::vector<std::string>>();
		rew_dim_ = cfg["rewards"]["names"].as<std::vector<std::string>>().size();
		// load start state
		start_pos_ = Vector<3>(
		(cfg["start_state"]["pos"].as<std::vector<Scalar>>()).data());
		start_lin_vel_ = Vector<3>(
		(cfg["start_state"]["vel"].as<std::vector<Scalar>>())
			.data());
		start_rot_ = Vector<4>(
		(cfg["start_state"]["rot"].as<std::vector<Scalar>>()).data());
		start_omega_ = Vector<3>(
		(cfg["start_state"]["omega"].as<std::vector<Scalar>>())
			.data());
		start_pos_var_ = cfg["start_state"]["pos_var"].as<Scalar>();
		start_lin_vel_var_ = cfg["start_state"]["vel_var"].as<Scalar>();
		start_rot_var_ = cfg["start_state"]["rot_var"].as<Scalar>();
		start_omega_var_ = cfg["start_state"]["omega_var"].as<Scalar>();
		// load goal trajectory
		// in case the yaml entry is a vector
		try {
		std::vector<Scalar> temp = cfg["goal_state"]["trajectory"].as<std::vector<Scalar>>();
		int goal_traj_n = (int) (temp.size() / 3);
		goal_traj_ = Eigen::Map<Eigen::MatrixXd, Eigen::Unaligned>(temp.data(), 3, goal_traj_n);
		}
		// in case the yaml entry is not a vector, so we take a random trajectory
		catch (...) {
		setRandomTraj_ = true;
		traj_n_min_ = cfg["goal_state"]["traj_n_min"].as<Scalar>();
		traj_n_max_ = cfg["goal_state"]["traj_n_max"].as<Scalar>();
		traj_size_x_ = cfg["goal_state"]["traj_size_x"].as<Scalar>();
		traj_size_y_ = cfg["goal_state"]["traj_size_y"].as<Scalar>();
		traj_size_z_ = cfg["goal_state"]["traj_size_z"].as<Scalar>();
		setGoalTrajectory();
		}

		goal_n_ = cfg["observation"]["traj_obs_n"].as<int>();
		goal_acc_r_ = cfg["goal_state"]["acc_r"].as<Scalar>();
  } else {
		logger_.error("Cannot load [rewards] parameters");
		return false;
  }
  return true;
}

bool RobotEnv::setGoalTrajectory() {
	int traj_n_ = (int)(((uniform_dist_(random_gen_) + 1) / 2) * (traj_n_max_ - traj_n_min_) + traj_n_min_);
	Matrix<Dynamic, Dynamic> temp(3, traj_n_);
	for (int i = 0; i < traj_n_; i++) {
		temp(0, i) = uniform_dist_(random_gen_) * traj_size_x_ / 2 + start_pos_[0];
		temp(1, i) = uniform_dist_(random_gen_) * traj_size_y_ / 2 + start_pos_[1];
		temp(2, i) = uniform_dist_(random_gen_) * traj_size_z_ / 2 + start_pos_[2];
	}
	goal_traj_ = temp;

	return true;
}

bool RobotEnv::setNextGoalPos() {
	int n = goal_traj_.cols();
	if (goal_idx_ < n) {
		goal_idx_++;
	}

	Matrix<3, Dynamic> temp(3, goal_n_);
	for (int i = 0; i < goal_n_; i++) {
		int j = std::min(goal_idx_ + i, n - 1);
		temp.block<3, 1>(0, i) = goal_traj_.block<3, 1>(0, j);
	}
	goal_obs_ = temp;
	goal_pos_ = goal_obs_.block<3, 1>(0, 0);

	return true;
}

bool RobotEnv::configCamera(const YAML::Node &cfg, const std::shared_ptr<RGBCamera> camera_) {
	if (!cfg["rgb_camera"]) {
		logger_.error("Cannot config RGB Camera");
		return false;
	}

	if (!cfg["rgb_camera"]["on"].as<bool>()) {
		logger_.warn("Camera is off. Please turn it on.");
		return false;
	}

	if (robot_ptr_->getNumCamera() >= 1) {
		logger_.warn("Camera has been added. Skipping the camera configuration.");
		return false;
	}

	// load camera settings
	std::vector<Scalar> t_BC_vec = cfg["rgb_camera"]["t_BC"].as<std::vector<Scalar>>();
	std::vector<Scalar> r_BC_vec = cfg["rgb_camera"]["r_BC"].as<std::vector<Scalar>>();

	//
	Vector<3> t_BC(t_BC_vec.data());
	Matrix<3, 3> r_BC = (AngleAxis(r_BC_vec[2] * M_PI / 180.0, Vector<3>::UnitZ()) *
	                     AngleAxis(r_BC_vec[1] * M_PI / 180.0, Vector<3>::UnitY()) *
	                     AngleAxis(r_BC_vec[0] * M_PI / 180.0, Vector<3>::UnitX()))
	                        .toRotationMatrix();
	std::vector<bool> post_processing = {false, false, false};
	post_processing[0] = cfg["rgb_camera"]["enable_depth"].as<bool>();
	post_processing[1] = cfg["rgb_camera"]["enable_segmentation"].as<bool>();
	post_processing[2] = cfg["rgb_camera"]["enable_opticalflow"].as<bool>();

	//
	rgb_camera_->setFOV(cfg["rgb_camera"]["fov"].as<Scalar>());
	rgb_camera_->setWidth(cfg["rgb_camera"]["width"].as<int>());
	rgb_camera_->setChannels(cfg["rgb_camera"]["channels"].as<int>());
	rgb_camera_->setHeight(cfg["rgb_camera"]["height"].as<int>());
	rgb_camera_->setRelPose(t_BC, r_BC);
	rgb_camera_->setPostProcessing(post_processing);

	// adapt parameters
	img_width_ = rgb_camera_->getWidth();
	img_height_ = rgb_camera_->getHeight();
	rgb_img_ = cv::Mat::zeros(img_height_, img_width_, CV_MAKETYPE(CV_8U, rgb_camera_->getChannels()));
	depth_img_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
	return true;
}

bool RobotEnv::configImu(const YAML::Node &cfg, const std::shared_ptr<IMU> imu_) {
	if (!cfg["imu"]) {
		logger_.error("Cannot config IMU");
		return false;
	}

	if (!cfg["imu"]["on"].as<bool>()) {
		logger_.warn("IMU is off. Please turn it on.");
		return false;
	}

	if (robot_ptr_->getNumImu() >= 1) {
		logger_.warn("IMU has already been added. Skipping the imu configuration.");
		return false;
	}

	// load imu settings
	std::vector<Scalar> t_BC_vec = cfg["imu"]["t_BC"].as<std::vector<Scalar>>();
	std::vector<Scalar> r_BC_vec = cfg["imu"]["r_BC"].as<std::vector<Scalar>>();

	// Relative pose
	Vector<3> t_BC(t_BC_vec.data());
	Matrix<3, 3> r_BC = (AngleAxis(r_BC_vec[2] * M_PI / 180.0, Vector<3>::UnitZ()) *
	                     AngleAxis(r_BC_vec[1] * M_PI / 180.0, Vector<3>::UnitY()) *
	                     AngleAxis(r_BC_vec[0] * M_PI / 180.0, Vector<3>::UnitX()))
	                        .toRotationMatrix();
	imu_->setRelPose(t_BC, r_BC);

	// Noise
	if (!cfg["imu"]["use_default_noise_params"].as<bool>()) {
		imu_->setGyroscopeNoiseDensity(cfg["imu"]["noise_params"]["gyroscope_noise_density"].as<Scalar>());
		imu_->setGyroscopeRandomWalk(cfg["imu"]["noise_params"]["gyroscope_random_walk"].as<Scalar>());
		imu_->setGyroscopeBiasCorrelationTime(
		    cfg["imu"]["noise_params"]["gyroscope_bias_correlation_time"].as<Scalar>());
		imu_->setGyroscopeTurnOnBiasSigma(cfg["imu"]["noise_params"]["gyroscope_turn_on_bias_sigma"].as<Scalar>());
		imu_->setAccelerometerNoiseDensity(cfg["imu"]["noise_params"]["accelerometer_noise_density"].as<Scalar>());
		imu_->setAccelerometerRandomWalk(cfg["imu"]["noise_params"]["accelerometer_random_walk"].as<Scalar>());
		imu_->setAccelerometerBiasCorrelationTime(
		    cfg["imu"]["noise_params"]["accelerometer_bias_correlation_time"].as<Scalar>());
		imu_->setAccelerometerTurnOnBiasSigma(
		    cfg["imu"]["noise_params"]["accelerometer_turn_on_bias_sigma"].as<Scalar>());
		imu_->setGravityMagnitude(cfg["imu"]["noise_params"]["gravity_magnitude"].as<Scalar>());
	}
	imu_->setBiases();

	return true;
}

bool RobotEnv::configAirspeed(const YAML::Node &cfg, const std::shared_ptr<Airspeed> airspeed_) {
	if (!cfg["airspeed"]) {
		logger_.error("Cannot config airspeed sensor");
		return false;
	}

	if (!cfg["airspeed"]["on"].as<bool>()) {
		logger_.warn("Airspeed sensor is off. Please turn it on.");
		return false;
	}

	if (robot_ptr_->getNumAirspeed() >= 1) {
		logger_.warn("Airspeed sensor has already been added. Skipping the Airspeed sensor configuration.");
		return false;
	}

	// load airspeed settings
	Scalar offset_std = cfg["airspeed"]["offset_std"].as<Scalar>();
	Scalar noise_std = cfg["airspeed"]["noise_std"].as<Scalar>();

	airspeed_->setOffsetStd(offset_std);
	airspeed_->setNoiseStd(noise_std);

	return true;
}

bool RobotEnv::addRobotToUnity(const std::shared_ptr<UnityBridge> bridge) {
	if (!robot_ptr_) return false;
	bridge->addRobot(robot_ptr_, cfg_["robot"]["type"].as<std::string>());
	return true;
}

bool RobotEnv::addTrajectoryObjectToUnity(const std::shared_ptr<UnityBridge> bridge) {
	// Initialize spheres
	for (int i = 0; i < goal_traj_.cols(); i++) {
		// core of the target
		std::string object_id = "staticSphere" + std::to_string(i);  // Unique name
		std::string prefab_id = "sphere";  // Name of the prefab in the Assets/Resources folder
		std::shared_ptr<StaticSphere> sphere = std::make_shared<StaticSphere>(object_id, prefab_id);
		Vector<3> size(1, 1, 1);
		sphere->setSize(size);  // otherwise it has size (0,0,0) and crashes
		sphere->setScale(size * 0.1);
		sphere->setRotation(Quaternion(1.0, 0.0, 0.0, 0.0));
		Vector<3> position = goal_traj_.block<3, 1>(0, i);
		sphere->setPosition(position);
		bridge->addStaticObject(sphere);

		// acceptance radius
		if (goal_traj_.cols() > 1) {
			object_id = "staticSphere_trans" + std::to_string(i);  // Unique name
			prefab_id = "sphere_transparent";                      // Name of the prefab in the Assets/Resources folder
			std::shared_ptr<StaticSphere> sphere_trans = std::make_shared<StaticSphere>(object_id, prefab_id);
			sphere_trans->setSize(size);  // otherwise it has size (0,0,0) and crashes
			sphere_trans->setScale(size * cfg_["goal_state"]["acc_r"].as<Scalar>());
			sphere_trans->setRotation(Quaternion(1.0, 0.0, 0.0, 0.0));
			sphere_trans->setPosition(position);
			bridge->addStaticObject(sphere_trans);
		}
	}
	return true;
}

std::ostream &operator<<(std::ostream &os, const RobotEnv &robot_env) {
	os.precision(3);
	os << "Robot Environment:\n"
	   << "obs dim =            [" << robot_env.obs_dim_ << "]\n"
	   << "act dim =            [" << robot_env.act_dim_ << "]\n"
	   << "sim dt =             [" << robot_env.sim_dt_ << "]\n"
	   << "max_t =              [" << robot_env.max_t_ << "]\n"
	   << "act_mean =           [" << robot_env.act_mean_.transpose() << "]\n"
	   << "act_std =            [" << robot_env.act_std_.transpose() << "]\n"
	   << "obs_mean =           [" << robot_env.obs_mean_.transpose() << "]\n"
	   << "obs_std =            [" << robot_env.obs_std_.transpose() << "]" << std::endl;
	os.precision();
	return os;
}

}  // namespace flightlib