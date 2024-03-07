#pragma once

// std lib
#include <stdlib.h>

#include <cmath>
#include <iostream>

// yaml cpp
#include <yaml-cpp/yaml.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/command.hpp"
#include "flightlib/common/logger.hpp"
#include "flightlib/common/robot_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/common/utils.hpp"
#include "flightlib/envs/env_base.hpp"
#include "flightlib/objects/robot.hpp"
#include "flightlib/objects/static_sphere.hpp"
#include "flightlib/sensors/imu.hpp"
#include "flightlib/sensors/airspeed.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

namespace flightlib {

class RobotEnv final : public EnvBase {
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	RobotEnv();
	RobotEnv(const std::string &cfg_path, const int env_id);
	RobotEnv(const YAML::Node &cfg_node, const int env_id);
	~RobotEnv();

	// - public OpenAI-gym-style functions
	bool reset(Ref<Vector<>> obs) override;
	bool step(const Ref<Vector<>> act, const Scalar dt, Ref<Vector<>> obs, Ref<Vector<>> reward) override;
	bool setRobotState(const Ref<Vector<>> state, Ref<Vector<>> obs, Ref<Vector<>> reward);
	bool setModelParams(const Ref<Vector<>> params);
	bool setLLDynamics(const Ref<Vector<>> state, const Ref<Vector<>> cmd);
	bool setLLOffset(const Ref<Vector<>> lastAction);
	bool callLLRun(const Ref<Vector<>> state, Ref<Vector<>> act, 
		Ref<Vector<>> gt, Ref<Vector<>> ref, Ref<Vector<>> proj_matrix);
	bool setLLGains(const Ref<Vector<>> gains, const int idx);
	bool storeLLReservoir();
	bool restoreLLReservoir();
	bool setRobotActuator(Ref<Vector<>> actuator) const;
	bool setTurbulence(Ref<Matrix<Dynamic,3>> turbulence);

	// - public set functions
	bool loadParam(const YAML::Node &cfg);

	// - public get functions
	bool getObs(Ref<Vector<>> obs) override;
	bool getImage(Ref<ImgVector<>> img, const bool rgb = true) override;
	bool getDepthImage(Ref<DepthImgVector<>> img) override;

	// get robot states
	bool getActMean(Ref<Vector<>> mean);
	bool getActStd(Ref<Vector<>> std);
	bool getRobotAct(Ref<Vector<>> act) const;
	bool getRobotActuator(Ref<Vector<>> actuator) const;
	bool getRobotState(Ref<Vector<>> state) const;
	Scalar getRobotTimestamp() const;
	Scalar getRobotEnergy() const;
	bool getWindCurl(Ref<Vector<>> wc) const;
	bool getWindCurlEst(Ref<Vector<>> wce) const;

	// get goal
	bool getGoalState(Ref<Vector<>> goal_state) const;

	// - auxiliar functions
	bool isTerminalState(Scalar &reward) override;
	bool addRobotToUnity(const std::shared_ptr<UnityBridge> bridge) override;
	bool addTrajectoryObjectToUnity(const std::shared_ptr<UnityBridge> bridge) override;

	friend std::ostream &operator<<(std::ostream &os, const RobotEnv &robot_env);

	inline std::vector<std::string> getRewardNames() { return reward_names_; }

	std::unordered_map<std::string, float> extra_info_;

   private:
	void init();
	int env_id_;
	bool configCamera(const YAML::Node &cfg, const std::shared_ptr<RGBCamera>);
	bool configImu(const YAML::Node &cfg, const std::shared_ptr<IMU>);
	bool configAirspeed(const YAML::Node &cfg, const std::shared_ptr<Airspeed>);

	// Setting goal functions
	bool setGoalTrajectory();
	bool setNextGoalPos();

	// Calculate reward for trajectory case
	bool getReward(Ref<Vector<>> reward);
	Scalar progressOnPath(Vector<3> pos, Vector<3> g0, Vector<3> g1);
	Scalar distanceToPath(Vector<3> pos, Vector<3> g0, Vector<3> g1);
	Scalar ln(Scalar x);

	// robot
	std::shared_ptr<Robot> robot_ptr_;
	RobotState robot_state_;
	Command cmd_;
	Logger logger_{"RobotEnv"};

	// Define reward for training
	Scalar pos_coeff_, ori_coeff_, ori_coeff_vert_, yaw_coeff_, lin_vel_coeff_, ang_vel_coeff_, lin_acc_coeff_, ang_acc_coeff_, pro_coeff_, done_coeff_, action_coeff_, energy_coeff_;

	// observations and actions (for RL)
	Vector<Dynamic> pi_obs_;
	Vector<Dynamic> pi_act_;
	Vector<Dynamic> act_ina_;
	//int size_reg_obs_;  // number of entries in the regular observation space (without actuator positions)

	// reward function design (for model-free reinforcement learning)
	Vector<3> start_pos_;
	Vector<3> start_lin_vel_;
	Vector<4> start_rot_;
	Vector<3> start_omega_;
	Scalar start_pos_var_;
	Scalar start_lin_vel_var_;
	Scalar start_rot_var_;
	Scalar start_omega_var_;
	Matrix<3, Dynamic> goal_traj_;
	int goal_idx_ = 0;
	int goal_idx_old_ = 0;
	Vector<3> goal_pos_;
	int goal_n_;
	Matrix<3, Dynamic> goal_obs_;
	Scalar goal_acc_r_ = 0;
	Vector<3> pos_old_;

	// random trajectory generation
	bool setRandomTraj_ = false;
	Scalar traj_n_min_;
	Scalar traj_n_max_;
	Scalar traj_size_x_;
	Scalar traj_size_y_;
	Scalar traj_size_z_;

	// action and observation normalization (for learning)
	Vector<Dynamic> act_mean_;
	Vector<Dynamic> act_std_;
	Vector<Dynamic> obs_mean_;
	Vector<Dynamic> obs_std_;
	Vector<Dynamic> inactivity_;

	// wind
	std::shared_ptr<Wind> wind_;

	// robot vision
	std::shared_ptr<RGBCamera> rgb_camera_;
	cv::Mat rgb_img_, gray_img_;
	cv::Mat depth_img_;

	// robot imu
	std::shared_ptr<IMU> imu_;

	// robot airspeed
	std::shared_ptr<Airspeed> airspeed_;

	// auxiliary variables
	int rotor_ctrl_{true};
	bool use_camera_{true};
	bool use_imu_{true};
	bool use_airspeed_{true};
	YAML::Node cfg_;
	std::vector<std::string> reward_names_;
	Matrix<3, 2> world_box_;

	// state estimation
	Scalar pos_std_;
	Scalar vel_std_;
	Scalar acc_std_;
	Scalar quat_std_;
	Scalar omega_std_;
	Scalar aac_std_;
};

}  // namespace flightlib