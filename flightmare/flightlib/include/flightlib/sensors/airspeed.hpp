#pragma once

#include "flightlib/common/types.hpp"
#include "flightlib/sensors/sensor_base.hpp"

namespace flightlib {

class Airspeed : SensorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Airspeed();
  ~Airspeed();
  
  inline void setOffsetStd(Scalar offset_std){offset_std_ = offset_std;}
  inline void setNoiseStd(Scalar noise_std){noise_std_ = noise_std;}
  Scalar getMeasurement(Scalar u_rel);
  void reset();

 private:
   Scalar offset_std_;
   Scalar offset_;
   Scalar noise_std_;
};
}  // namespace flightlib
