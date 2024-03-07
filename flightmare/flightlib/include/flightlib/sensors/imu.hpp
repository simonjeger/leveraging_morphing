#pragma once

#include "flightlib/common/logger.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/sensors/sensor_base.hpp"

namespace flightlib {

// Default values for use with ADIS16448 IMU
static constexpr double kDefaultAdisGyroscopeNoiseDensity = 2.0 * 35.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeRandomWalk = 2.0 * 4.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeBiasCorrelationTime = 1.0e+3;
static constexpr double kDefaultAdisGyroscopeTurnOnBiasSigma = 0.5 / 180.0 * M_PI;
static constexpr double kDefaultAdisAccelerometerNoiseDensity = 2.0 * 2.0e-3;
static constexpr double kDefaultAdisAccelerometerRandomWalk = 2.0 * 3.0e-3;
static constexpr double kDefaultAdisAccelerometerBiasCorrelationTime = 300.0;
static constexpr double kDefaultAdisAccelerometerTurnOnBiasSigma = 20.0e-3 * 9.8;
static constexpr double kDefaultGravityMagnitude = -Gz;

class IMU : SensorBase {
   public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	IMU();
	~IMU();

	// public set functions
	bool setRelPose(const Ref<Vector<3>> B_r_BI, const Ref<Matrix<3, 3>> R_BI);
	bool setBiases(void);

	bool setLinearAccelerationMeasurement(Vector<3> linear_acceleration);
	bool setAngularVelocityMeasurement(Vector<3> angular_velocity);

	bool setGyroscopeNoiseDensity(Scalar gyroscope_noise_density);
	bool setGyroscopeRandomWalk(Scalar gyroscope_random_walk);
	bool setGyroscopeBiasCorrelationTime(Scalar gyroscope_bias_correlation_time);
	bool setGyroscopeTurnOnBiasSigma(Scalar gyroscope_turn_on_bias_sigma);
	bool setAccelerometerNoiseDensity(Scalar accelerometer_noise_density);
	bool setAccelerometerRandomWalk(Scalar accelerometer_random_walk);
	bool setAccelerometerBiasCorrelationTime(Scalar accelerometer_bias_correlation_time);
	bool setAccelerometerTurnOnBiasSigma(Scalar accelerometer_turn_on_bias_sigma);
	bool setGravityMagnitude(Scalar gravity_magnitude);

	bool setLastMeasurementTime(Scalar last_measurement_time);

	// public get functions
	Matrix<4, 4> getRelPoseBody2Imu(void) const;
	Matrix<4, 4> getRelPoseImu2Body(void) const;
	Vector<3> getGyroscopeBias(void) const;
	Vector<3> getAccelerometerBias(void) const;
	Vector<3> getGyroscopeTurnOnBias(void) const;
	Vector<3> getAccelerometerTurnOnBias(void) const;

	Vector<3> getLinearAccelerationMeasurement(void) const;
	Vector<3> getAngularVelocityMeasurement(void) const;

	Scalar getGyroscopeNoiseDensity(void) const;
	Scalar getGyroscopeRandomWalk(void) const;
	Scalar getGyroscopeBiasCorrelationTime(void) const;
	Scalar getGyroscopeTurnOnBiasSigma(void) const;
	Scalar getAccelerometerNoiseDensity(void) const;
	Scalar getAccelerometerRandomWalk(void) const;
	Scalar getAccelerometerBiasCorrelationTime(void) const;
	Scalar getAccelerometerTurnOnBiasSigma(void) const;
	Scalar getGravityMagnitude(void) const;
	Scalar getLastMeasurementTime(void) const;

	// auxiliary functions
	void addNoiseToImu(const Scalar dt);

   private:
	Logger logger_{"IMU"};
	Vector<3> B_r_BI_;   // Translation body to IMU
	Matrix<4, 4> T_BI_;  // relative pose from body to IMU frame
	Matrix<4, 4> T_IB_;  // relative pose from IMU to body frame
	Vector<3> gyroscope_bias_;
	Vector<3> accelerometer_bias_;
	Vector<3> gyroscope_turn_on_bias_;
	Vector<3> accelerometer_turn_on_bias_;

	Vector<3> linear_acceleration_measurement_;
	Vector<3> angular_velocity_measurement_;
	Scalar last_measurement_time_;

	// IMU noise parameters
	Scalar gyroscope_noise_density_;              // [rad/s/sqrt(Hz)]
	Scalar gyroscope_random_walk_;                // [rad/s/s/sqrt(Hz)]
	Scalar gyroscope_bias_correlation_time_;      // [s]
	Scalar gyroscope_turn_on_bias_sigma_;         // [rad/s]
	Scalar accelerometer_noise_density_;          // [m/s^2/sqrt(Hz)]
	Scalar accelerometer_random_walk_;            // [m/s^2/s/sqrt(Hz)]
	Scalar accelerometer_bias_correlation_time_;  // [s]
	Scalar accelerometer_turn_on_bias_sigma_;     // [m/s^2]
	Scalar gravity_magnitude_;                    // [m/s^2]

	std::default_random_engine random_generator_;
	std::normal_distribution<double> standard_normal_distribution_;
};
}  // namespace flightlib
