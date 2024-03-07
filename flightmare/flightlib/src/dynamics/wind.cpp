#include "flightlib/dynamics/wind.hpp"

namespace flightlib {

Wind::Wind(const YAML::Node &cfg_node) {
  // load configuration file
  cfg_ = cfg_node;

  mag_min_ = cfg_["wind"]["mag_min"].as<Scalar>();
  mag_max_ = cfg_["wind"]["mag_max"].as<Scalar>();
  var_ = cfg_["turbulence"]["var"].as<Scalar>();


  // find nx, ny, nz
  if (cfg_["turbulence"]["on"].as<bool>()){
    Matrix<3, 2> world_box = Matrix<3, 2>((cfg_["environment"]["world_box"].as<std::vector<Scalar>>()).data());
    res_ = cfg_["turbulence"]["res"].as<Scalar>();
    Scalar vel_max = cfg_["wind"]["mag_max"].as<Scalar>();
    Scalar dt_max = cfg_["simulation"]["max_t"].as<Scalar>();
    curl_dist_ = cfg_["turbulence"]["curl_dist"].as<Scalar>();
    turbulence_scaling_ = cfg_["turbulence"]["scaling"].as<Scalar>();

    nx_ = (int)std::ceil(((world_box(3)-world_box(0))*std::pow(2.0,0.5) + vel_max*dt_max + 2*curl_dist_)/res_);
    ny_ = (int)std::ceil(((world_box(4)-world_box(1))*std::pow(2.0,0.5) + 2*curl_dist_)/res_);
    nz_ = (int)std::ceil(((world_box(5)-world_box(2))*std::pow(2.0,0.5) + 2*curl_dist_)/res_);
    nx_c_ = (int)std::ceil(((world_box(3)-world_box(0))*std::pow(2.0,0.5) + 2*curl_dist_)/res_);

    pos_center_ << (world_box(3) + world_box(0))/2.0, (world_box(4) + world_box(1))/2.0, (world_box(5) + world_box(2))/2.0;

    for (int k = 0; k < 2; ++k) {
      for (int j = 0; j < 2; ++j) {
        for (int i = 0; i < 2; ++i) {
          // Calculate the index in the local_grid array
          int index = k * 4 + j * 2 + i;
          local_grid_[index] << i, k, j; //the order needs to be like that because of the way the trilinearInterpolation is written
        }
      }
    }
  }

  reset();
}

void Wind::reset() {
  uni_dist_mag = std::uniform_real_distribution<Scalar>(mag_min_, mag_max_);
  mag_ = uni_dist_mag(random_gen_);

  // in case the yaml entry for wind direction is a vector
  Vector<3> dir;
  try {
    dir = Vector<3>((cfg_["wind"]["dir"].as<std::vector<Scalar>>()).data());
    if (dir.norm() == 0){
      std::cout << "Wind: [0,0,0] is not a valid direction \n";
    }
  }

  catch (...) {
    // in case the yaml entry for wind direction is not a vector, so we take a
    // random direction
    for (int i = 0; i < 3; i++) {
      dir(i) = uni_dist_dir(random_gen_);
    }
    
    // wind in z is usually smaller than in x or y
    dir(2) = dir(2)/10.0;

    if (cfg_["wind"]["dir"].as<std::string>() == "random"){
      // change nothing 
    }
    else if (cfg_["wind"]["dir"].as<std::string>() == "front"){
      // make sure the wind has never more than 45 degrees angle and has negative x component
      if (abs(dir(0)) < abs(dir(1))){
        Scalar swap = dir(1);
        dir_(1) = dir(0);
        dir_(0) = swap;
      }
      dir(0) = -abs(dir(0));
    } 
    else if (cfg_["wind"]["dir"].as<std::string>() == "front_2D"){
      dir(0) = -1;
      dir(1) = 0;
      dir(2) = 0.42 * uni_dist_dir(random_gen_);
    } 
    else{
      std::cout << "Wind: choose a valid option in config file \n";
    }
  }
  dir_ = dir/dir.norm();
  dir_scaled_ = dir_*mag_;

  if (cfg_["turbulence"]["on"].as<bool>()){
    along_wind_.setZero();
  } else {
    norm_dist_x = std::normal_distribution<Scalar>(dir_scaled_[0], var_);
    norm_dist_y = std::normal_distribution<Scalar>(dir_scaled_[1], var_);
    norm_dist_z = std::normal_distribution<Scalar>(dir_scaled_[2], var_);  
  }

  current_wind_.setZero();
  current_curl_.setZero();
}

bool Wind::setTurbulence(Ref<Matrix<Dynamic,3>> turbulence){
  turbulence_ = turbulence;
  if (turbulence_.rows() != nx_*ny_*nz_){
    std::cout << "ERROR IN TURBULENCE: expected " << nx_*ny_*nz_  << " entries (" << nx_ << " x " << ny_ << " x "  << nz_ << ") , but got " << turbulence_.rows() << "\n";
  }

  // calculating variance (for rendering)
  Scalar mean = turbulence_.mean();
  var_ = (turbulence_.array() - mean).square().sum() / (turbulence_.size() - 1);

  return true;
}

Vector<6> Wind::getWindCurl(Vector<3> position, Scalar dt) {
  if (cfg_["turbulence"]["on"].as<bool>()){
    // Running variable along wind
    along_wind_(0) += mag_*dt;

    // Interpolate
    current_wind_ = dir_scaled_ + turbulence_scaling_ * interpolate(position);
    if (cfg_["turbulence"]["curl"].as<bool>()){
      current_curl_ = turbulence_scaling_ * curl(position);
    }
  }
  else{
    Scalar noise_x = norm_dist_x(random_gen_);
    Scalar noise_y = norm_dist_y(random_gen_);
    Scalar noise_z = norm_dist_z(random_gen_);

    current_wind_ = Vector<3>{noise_x, noise_y, noise_z};
  }
  
  Vector<6> wind_curl;
  wind_curl << current_wind_, current_curl_;
  return wind_curl;
}

Vector<3> Wind::getWindPred() {
  if (dir_scaled_.norm() != 0.0){
    return dir_*(mag_max_ + mag_min_)/2.0;
  }
  else{
    return Vector<3>::Zero();
  } 
}

Vector<3> Wind::curl(const Vector<3>& point){
  Vector<3> up = interpolate(point + Vector<3>{0,0,curl_dist_/2});
  Vector<3> down = interpolate(point + Vector<3>{0,0,-curl_dist_/2.0});
  Vector<3> front = interpolate(point + Vector<3>{curl_dist_/2.0,0,0});
  Vector<3> back = interpolate(point + Vector<3>{-curl_dist_/2.0,0,0});
  Vector<3> right = interpolate(point + Vector<3>{0,-curl_dist_/2.0,0});
  Vector<3> left = interpolate(point + Vector<3>{0,curl_dist_/2.0,0});

  // Calculate velocity gradients using central differences
  Vector<3> curl;
  
  curl(0) = (left(2) - right(2) + down(1) - up(1))/(2*curl_dist_);
  curl(1) = (back(2) - front(2) + up(0) - down(0))/(2*curl_dist_);
  curl(2) = (front(1) - back(1) + right(0) - left(0))/(2*curl_dist_);
  
  return curl;
}

// Vector<3> Wind::interpolate(const Vector<3>& position){
//   return getGust(position);
// }

Vector<3> Wind::interpolate(const Vector<3>& position){
  // Calculate the grid indices based on the resolution
  Vector<3> position_grid;
  position_grid(0) = std::floor(position(0)/res_)*res_;
  position_grid(1) = std::floor(position(1)/res_)*res_;
  position_grid(2) = std::floor(position(2)/res_)*res_;

  Vector<3> c[8];
  for (int i = 0; i < 8; ++i) {
    c[i] = getGust(position_grid + local_grid_[i]*res_);
  }
  // Interpolate
  return trilinearInterpolation((position - position_grid)/res_, c);
}

Vector<3> Wind::trilinearInterpolation(const Vector<3>& point, const Vector<3> c[8]) {
  // Expects values in point to be in [0,1]
  Scalar x = point(0);
  Scalar y = point(1);
  Scalar z = point(2);

  Vector<3> xCoeff(1 - x, x, 0);
  Vector<3> yCoeff(1 - y, y, 0);
  Vector<3> zCoeff(1 - z, z, 0);

  Vector<3> cx00 = c[0] * xCoeff[0] + c[1] * xCoeff[1];
  Vector<3> cx01 = c[2] * xCoeff[0] + c[3] * xCoeff[1];
  Vector<3> cx10 = c[4] * xCoeff[0] + c[5] * xCoeff[1];
  Vector<3> cx11 = c[6] * xCoeff[0] + c[7] * xCoeff[1];

  Vector<3> cxy0 = cx00 * yCoeff[0] + cx10 * yCoeff[1];
  Vector<3> cxy1 = cx01 * yCoeff[0] + cx11 * yCoeff[1];

  Vector<3> interpolatedValue = cxy0 * zCoeff[0] + cxy1 * zCoeff[1];

  return interpolatedValue;
}

Vector<3> Wind::getGust(Vector<3> position){
  // Find rotation matrix to rotate position
  Scalar theta = std::acos(dir_.dot(Eigen::Vector3d::UnitZ()));
  Vector<3> rotation_axis = dir_.cross(Eigen::Vector3d::UnitZ()).normalized();
  Eigen::AngleAxisd rotation(theta, rotation_axis);
  Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();

  // Position in wind frame
  Vector<3> position_w = (rotation_matrix*(position - pos_center_) + along_wind_);

  // Find index
  int x = (int)(nx_c_/2 + position_w(0)/res_);
  int y = (int)(ny_/2 + position_w(1)/res_);
  int z = (int)(nz_/2 + position_w(2)/res_);

  if ((x < 0) || (x >= nx_)){
    std::cout << "ERROR IN TURBULENCE: index in x out of bounds \n";
    x = std::clamp(x,0,nx_-1);
  }

  if ((y < 0) || (y >= ny_)){
    std::cout << "ERROR IN TURBULENCE: index in y out of bounds \n";
    y = std::clamp(y,0,ny_-1);
  }

  if ((z < 0) || (z >= nz_)){
    std::cout << "ERROR IN TURBULENCE: index in z out of bounds \n";
    z = std::clamp(z,0,nz_-1);
  }

  int index = x*nz_*ny_ + y*nz_ + z;

  return turbulence_.row(index);
}


}  // namespace flightlib