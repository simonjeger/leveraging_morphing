#pragma once

#include <stdlib.h>
#include <iostream>

#include "flightlib/common/types.hpp"
#include <random>

namespace flightlib {
class SensorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SensorBase();
  virtual ~SensorBase();

  std::default_random_engine rd_;
  std::mt19937 random_gen_{rd_()};
  std::uniform_real_distribution<Scalar> uni_dist_{-1.0, 1.0};
  std::normal_distribution<Scalar> norm_dist_{0.0, 1.0};

 private:
};

}  // namespace flightlib
