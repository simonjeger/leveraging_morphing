#include "flightlib/controller/lowlevel_controller_base.hpp"

namespace flightlib {

LowLevelControllerBase::LowLevelControllerBase() {
  state_.setZero();
  motor_omega_des_.setZero();
}

}  // namespace flightlib
