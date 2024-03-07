#include "flightlib/sensors/imu.hpp"

namespace flightlib {

IMU::IMU()
    : gyroscope_noise_density_(kDefaultAdisGyroscopeNoiseDensity),
      gyroscope_random_walk_(kDefaultAdisGyroscopeRandomWalk),
      gyroscope_bias_correlation_time_(kDefaultAdisGyroscopeBiasCorrelationTime),
      gyroscope_turn_on_bias_sigma_(kDefaultAdisGyroscopeTurnOnBiasSigma),
      accelerometer_noise_density_(kDefaultAdisAccelerometerNoiseDensity),
      accelerometer_random_walk_(kDefaultAdisAccelerometerRandomWalk),
      accelerometer_bias_correlation_time_(kDefaultAdisAccelerometerBiasCorrelationTime),
      accelerometer_turn_on_bias_sigma_(kDefaultAdisAccelerometerTurnOnBiasSigma),
      gravity_magnitude_(kDefaultGravityMagnitude),
      last_measurement_time_(0.0){}

IMU::~IMU() {}

bool IMU::setRelPose(const Ref<Vector<3>> B_r_BI, const Ref<Matrix<3, 3>> R_BI) {
	if (!B_r_BI.allFinite() || !R_BI.allFinite()) {
		logger_.error("The setting value for IMU Relative Pose Matrix is not valid, discard the setting.");
		return false;
	}

	// Transformation from body to IMU frame
	B_r_BI_ = B_r_BI;
	T_BI_.block<3, 3>(0, 0) = R_BI;
	T_BI_.block<3, 1>(0, 3) = B_r_BI;
	T_BI_.row(3) << 0.0, 0.0, 0.0, 1.0;

	// Transformation from IMU to body frame
	T_IB_.block<3, 3>(0, 0) = R_BI.transpose();
	T_BI_.block<3, 1>(0, 3) = -R_BI * B_r_BI;
	T_BI_.row(3) << 0.0, 0.0, 0.0, 1.0;

	return true;
}

bool IMU::setBiases() {
	Scalar gyro_turn_on_sigma = getGyroscopeTurnOnBiasSigma();
	Scalar acc_turn_on_sigma = getAccelerometerTurnOnBiasSigma();
	accelerometer_bias_.setZero();
	gyroscope_bias_.setZero();
	for (int i = 0; i < 3; ++i) {
		gyroscope_turn_on_bias_[i] = gyro_turn_on_sigma * norm_dist_(rd_);
		accelerometer_turn_on_bias_[i] = acc_turn_on_sigma * norm_dist_(rd_);
	}
	return true;
}

bool IMU::setLinearAccelerationMeasurement(Vector<3> linear_acceleration) {
	linear_acceleration_measurement_ = linear_acceleration;
	return true;
}

bool IMU::setAngularVelocityMeasurement(Vector<3> angular_velocity) {
	angular_velocity_measurement_ = angular_velocity;
	return true;
}

bool IMU::setGyroscopeNoiseDensity(Scalar gyroscope_noise_density) {
	if (gyroscope_noise_density < 0) {
		logger_.warn("setGyroscopeNoiseDensity must be provided a postive value. Using default value instead");
		accelerometer_random_walk_ = kDefaultAdisGyroscopeNoiseDensity;
		return false;
	}
	gyroscope_noise_density_ = gyroscope_noise_density;
	return true;
}

bool IMU::setGyroscopeRandomWalk(Scalar gyroscope_random_walk) {
	if (gyroscope_random_walk < 0) {
		logger_.warn("setGyroscopeRandomWalk must be provided a postive value. Using default value instead");
		accelerometer_random_walk_ = kDefaultAdisGyroscopeRandomWalk;
		return false;
	}
	gyroscope_random_walk_ = gyroscope_random_walk;
	return true;
}

bool IMU::setGyroscopeBiasCorrelationTime(Scalar gyroscope_bias_correlation_time) {
	if (gyroscope_bias_correlation_time < 0) {
		logger_.warn("setGyroscopeBiasCorrelationTime must be provided a postive value. Using default value instead");
		gyroscope_bias_correlation_time_ = kDefaultAdisGyroscopeBiasCorrelationTime;
		return false;
	}
	gyroscope_bias_correlation_time_ = gyroscope_bias_correlation_time;
	return true;
}

bool IMU::setGyroscopeTurnOnBiasSigma(Scalar gyroscope_turn_on_bias_sigma) {
	if (gyroscope_turn_on_bias_sigma < 0) {
		logger_.warn("setGyroscopeTurnOnBiasSigma must be provided a postive value. Using default value instead");
		gyroscope_turn_on_bias_sigma_ = kDefaultAdisGyroscopeTurnOnBiasSigma;
		return false;
	}
	gyroscope_turn_on_bias_sigma_ = gyroscope_turn_on_bias_sigma;
	return true;
}

bool IMU::setAccelerometerNoiseDensity(Scalar accelerometer_noise_density) {
	if (accelerometer_noise_density < 0) {
		logger_.warn("setAccelerometerNoiseDensity must be provided a postive value. Using default value instead");
		accelerometer_random_walk_ = kDefaultAdisAccelerometerNoiseDensity;
		return false;
	}
	accelerometer_noise_density_ = accelerometer_noise_density;
	return true;
}

bool IMU::setAccelerometerRandomWalk(Scalar accelerometer_random_walk) {
	if (accelerometer_random_walk < 0) {
		logger_.warn("setAccelerometerRandomWalk must be provided a postive value. Using default value instead");
		accelerometer_random_walk_ = kDefaultAdisAccelerometerRandomWalk;
		return false;
	}
	accelerometer_random_walk_ = accelerometer_random_walk;
	return true;
}

bool IMU::setAccelerometerBiasCorrelationTime(Scalar accelerometer_bias_correlation_time) {
	if (accelerometer_bias_correlation_time < 0) {
		logger_.warn(
		    "setAccelerometerBiasCorrelationTime must be provided a postive value. Using default value instead");
		accelerometer_bias_correlation_time_ = kDefaultAdisAccelerometerBiasCorrelationTime;
		return false;
	}
	accelerometer_bias_correlation_time_ = accelerometer_bias_correlation_time;
	return true;
}

bool IMU::setAccelerometerTurnOnBiasSigma(Scalar accelerometer_turn_on_bias_sigma) {
	if (accelerometer_turn_on_bias_sigma < 0) {
		logger_.warn("setAccelerometerTurnOnBiaSigma must be provided a postive value. Using default value instead");
		accelerometer_turn_on_bias_sigma_ = kDefaultAdisAccelerometerTurnOnBiasSigma;
		return false;
	}
	accelerometer_turn_on_bias_sigma_ = accelerometer_turn_on_bias_sigma;
	return true;
}

bool IMU::setGravityMagnitude(Scalar gravity_magnitude) {
	if (gravity_magnitude < 0) {
		logger_.warn("setGravityMagnitude must be provided a postive value. Using default value instead");
		gravity_magnitude_ = kDefaultGravityMagnitude;
		return false;
	}
	gravity_magnitude_ = gravity_magnitude;
	return true;
}

bool IMU::setLastMeasurementTime(Scalar last_measurement_time) {
	if (last_measurement_time < 0) {
		logger_.warn("setLastMeasurementTime must be provided a postive value. Keeping previous value instead");
		return false;
	}
	last_measurement_time_ = last_measurement_time;
	return true;
}

Matrix<4, 4> IMU::getRelPoseBody2Imu(void) const { return T_BI_; }
Matrix<4, 4> IMU::getRelPoseImu2Body(void) const { return T_IB_; }
Vector<3> IMU::getGyroscopeBias(void) const { return gyroscope_bias_; }
Vector<3> IMU::getAccelerometerBias(void) const { return accelerometer_bias_; }
Vector<3> IMU::getGyroscopeTurnOnBias(void) const { return gyroscope_turn_on_bias_; }
Vector<3> IMU::getAccelerometerTurnOnBias(void) const { return accelerometer_turn_on_bias_; }
Vector<3> IMU::getLinearAccelerationMeasurement(void) const { return linear_acceleration_measurement_; }
Vector<3> IMU::getAngularVelocityMeasurement(void) const { return angular_velocity_measurement_; }

Scalar IMU::getGyroscopeNoiseDensity(void) const { return gyroscope_noise_density_; }
Scalar IMU::getGyroscopeRandomWalk(void) const { return gyroscope_random_walk_; }
Scalar IMU::getGyroscopeBiasCorrelationTime(void) const { return gyroscope_bias_correlation_time_; }
Scalar IMU::getGyroscopeTurnOnBiasSigma(void) const { return gyroscope_turn_on_bias_sigma_; }
Scalar IMU::getAccelerometerNoiseDensity(void) const { return accelerometer_noise_density_; }
Scalar IMU::getAccelerometerRandomWalk(void) const { return accelerometer_random_walk_; }
Scalar IMU::getAccelerometerBiasCorrelationTime(void) const { return accelerometer_bias_correlation_time_; }
Scalar IMU::getAccelerometerTurnOnBiasSigma(void) const { return accelerometer_turn_on_bias_sigma_; }
Scalar IMU::getGravityMagnitude(void) const { return gravity_magnitude_; }
Scalar IMU::getLastMeasurementTime(void) const { return last_measurement_time_; }

void IMU::addNoiseToImu(const Scalar dt) {
	if (dt <= 0.0) {
		logger_.error("Change in time must be greater than 0.");
	}

	// Accelerometer
	Scalar tau_a = accelerometer_bias_correlation_time_;
	Scalar sigma_a_d = 1 / sqrt(dt) * accelerometer_noise_density_;
	Scalar sigma_b_a = accelerometer_random_walk_;
	Scalar sigma_b_a_d = sqrt(-sigma_b_a * sigma_b_a * tau_a / 2.0 * (exp(-2.0 * dt / tau_a) - 1.0));
	Scalar phi_a_d = exp(-1.0 / tau_a * dt);
	for (int i = 0; i < 3; ++i) {
		accelerometer_bias_[i] = phi_a_d * accelerometer_bias_[i] + sigma_b_a_d * norm_dist_(rd_);
		(linear_acceleration_measurement_)[i] = (linear_acceleration_measurement_)[i] + accelerometer_bias_[i] +
		                                        sigma_a_d * norm_dist_(rd_) + accelerometer_turn_on_bias_[i];
	}

	// Gyroscope
	Scalar tau_g = gyroscope_bias_correlation_time_;
	Scalar sigma_g_d = 1 / sqrt(dt) * gyroscope_noise_density_;
	Scalar sigma_b_g = gyroscope_random_walk_;
	Scalar sigma_b_g_d = sqrt(-sigma_b_g * sigma_b_g * tau_g / 2.0 * (exp(-2.0 * dt / tau_g) - 1.0));
	Scalar phi_g_d = exp(-1.0 / tau_g * dt);
	for (int i = 0; i < 3; ++i) {
		gyroscope_bias_[i] = phi_g_d * gyroscope_bias_[i] + sigma_b_g_d * norm_dist_(rd_);
		(angular_velocity_measurement_)[i] = (angular_velocity_measurement_)[i] + gyroscope_bias_[i] +
		                                     sigma_g_d * norm_dist_(rd_) + gyroscope_turn_on_bias_[i];
	}
}

}  // namespace flightlib
