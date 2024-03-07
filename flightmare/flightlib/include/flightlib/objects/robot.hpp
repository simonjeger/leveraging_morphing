#pragma once

#include <stdlib.h>

// flightlib
#include "flightlib/objects/object_base.hpp"
#include "flightlib/common/command.hpp"
#include "flightlib/common/integrator_euler.hpp"
#include "flightlib/common/integrator_rk4.hpp"
#include "flightlib/common/logger.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/common/utils.hpp"
#include "flightlib/controller/lowlevel_controller_betaflight.hpp"
#include "flightlib/controller/lowlevel_controller_simple.hpp"
#include "flightlib/controller/lowlevel_controller_custom.hpp"
#include "flightlib/dynamics/bixler_dynamics.hpp"
#include "flightlib/dynamics/mpextra330_dynamics.hpp"
#include "flightlib/dynamics/mpextra330ml_dynamics.hpp"
#include "flightlib/dynamics/liseagle_dynamics.hpp"
#include "flightlib/dynamics/quadrotor_dynamics.hpp"
#include "flightlib/dynamics/wind.hpp"
#include "flightlib/sensors/airspeed.hpp"
#include "flightlib/sensors/imu.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

namespace flightlib {

class Robot : public ObjectBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Robot(const YAML::Node& cfg_node);
	Robot(const std::string& cfg_path);
	~Robot();

	// reset
	void init();
	bool setDetCfg();
	bool reset();
	bool reset(const RobotState& state);

	// run the robot
	bool run(const Command& cmd, const Scalar dt);
	bool run(const Scalar dt);

	// public get functions
	bool getState(RobotState* const state) const;
	Scalar getForceMax(void) const;
	Vector<3> getOmegaMax(void) const;
	Scalar getVeluMax(void) const;
	Vector<Dynamic> getRawMean(void) const;
	Vector<Dynamic> getRawStd(void) const;
	Vector<3> getSize(void) const;
	Vector<3> getPosition(void) const;
	Scalar getConsumption() const;
	bool getCollision() const;
	Scalar getMass(void);
	std::vector<Scalar> getAction(void);
	Vector<Dynamic> getActuator();
	std::vector<std::shared_ptr<RGBCamera>> getCameras(void) const;
	bool getCamera(const size_t cam_id, std::shared_ptr<RGBCamera> camera) const;
	int getNumCamera() const;
	std::vector<std::shared_ptr<IMU>> getImus(void) const;
	bool getImu(const size_t imu_id, std::shared_ptr<IMU> imu) const;
	int getNumImu() const;
	bool getImuMeasurement(Scalar imu_id, Ref<Vector<>> accelerometer_measurement, Ref<Vector<>> gyroscope_measurement);
	std::vector<std::shared_ptr<Airspeed>> getAirspeeds(void) const;
	bool getAirspeed(const size_t airspeed_id, std::shared_ptr<Airspeed> airspeed) const;
	int getNumAirspeed() const;
	Scalar getAirspeedMeasurement(Scalar airspeed_id);
	Vector<3> getWind(void);
	Scalar getWindVar(void);

	// public set functions
	bool setRobotState(const RobotState& state);
	bool setCommand(const Command& cmd);
  	void setSize(const Ref<Vector<3>> size);
	void setCollision(const bool collision);
	bool updateDynamics();
  	bool setWorldBox(const Ref<Matrix<3, 2>> box);
	bool constrainInWorldBox(const RobotState& old_state);
	bool addWind(std::shared_ptr<Wind> wind);
	bool addRGBCamera(std::shared_ptr<RGBCamera> camera);
	bool addImu(std::shared_ptr<IMU> imu);
	bool addAirspeed(std::shared_ptr<Airspeed> airspeed);
	bool setModelParams(Ref<Vector<>> params);
	bool setLLDynamics(const RobotState& state, const Command& cmd);
	bool setLLOffset(const Vector<Dynamic> offset);
	bool callLLRun(const RobotState& state, Ref<Vector<Dynamic>> act, 
		std::map<std::string, Eigen::VectorXd>& info);
	bool setLLGains(Ref<Vector<Dynamic>> gains, const int idx);
	bool storeLLReservoir();
	bool restoreLLReservoir();
  	bool setActuator(const Vector<Dynamic>& actuator);

private:
	// sensors
	std::shared_ptr<DynamicsBase> dynamics_ptr_;
	std::shared_ptr<DynamicsBase> dynamics_det_ptr_;
	std::shared_ptr<LowLevelControllerBase> ctrl_ptr_;
	std::shared_ptr<IntegratorBase> integrator_ptr_;
	std::shared_ptr<Wind> wind_;
	std::vector<std::shared_ptr<RGBCamera>> rgb_cameras_;
	std::vector<std::shared_ptr<IMU>> imus_;
	std::vector<std::shared_ptr<Airspeed>> airspeeds_;

	// logger
	Logger logger_{"Robot"};

	// robot control command
	Command cmd_;
  	Vector<Dynamic> cmd_ll_; // is only global so unity bridge can access it through getAction

	// robot state
	RobotState state_;
	Vector<3> size_;
	bool collision_;

	// auxiliary variables
	YAML::Node cfg_;
	YAML::Node cfg_det_;
	Scalar consumption_;
	Matrix<3, 2> world_box_;
	Vector<3> wind_pred_;
};

}  // namespace flightlib