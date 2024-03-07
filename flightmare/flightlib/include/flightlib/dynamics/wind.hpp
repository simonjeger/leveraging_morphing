#pragma once

// std lib
#include <stdlib.h>
#include <unistd.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <unordered_map>
#include <vector>

// yaml cpp
#include <yaml-cpp/yaml.h>

// flightlib
#include "flightlib/common/types.hpp"

namespace flightlib {

class Wind {
 public:
  Wind(const YAML::Node &cfg_node);

  bool setTurbulence(Ref<Matrix<Dynamic,3>> turbulence);
  Vector<6> getWindCurl(Vector<3> position, Scalar dt);
  Vector<3> getWindPred();
  inline Vector<3> getCurrentWind(){ return current_wind_; }
  inline Vector<3> getCurrentCurl(){ return current_curl_; }
  Scalar getWindVar(){ return var_; }
  void reset();

 private:
  Vector<3> curl(const Vector<3>& position);
  Vector<3> interpolate(const Vector<3>& point);
  Vector<3> trilinearInterpolation(const Vector<3>& point, const Vector<3> c[8]);
  Vector<3> getGust(Vector<3> position);

  Scalar mag_min_;
  Scalar mag_max_;
  Scalar mag_;
  Scalar var_;
  Vector<3> dir_;
  Vector<3> dir_scaled_;
  Vector<3> current_wind_;
  Vector<3> current_curl_;
  Scalar curl_dist_;
  Scalar turbulence_scaling_;
  Vector<3> local_grid_[8];

  Matrix<Dynamic,3> turbulence_;
  Vector<3> along_wind_;
  Scalar res_;
  int nx_,ny_,nz_, nx_c_;
  Vector<3> pos_center_;

  YAML::Node cfg_;

  std::default_random_engine rd_;
  std::mt19937 random_gen_{rd_()};
  std::uniform_real_distribution<Scalar> uni_dist_mag{0.0, 1.0};
  std::uniform_real_distribution<Scalar> uni_dist_dir{-1.0, 1.0};
  std::normal_distribution<Scalar> norm_dist_x{0.0, 1.0};
  std::normal_distribution<Scalar> norm_dist_y{0.0, 1.0};
  std::normal_distribution<Scalar> norm_dist_z{0.0, 1.0};
};

}  // namespace flightlib