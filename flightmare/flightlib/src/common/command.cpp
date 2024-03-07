#include "flightlib/common/command.hpp"


namespace flightlib {

Command::Command()
  : t_(0.0),
    // thrust(0.0, 0.0, 0.0, 0.0),
    collective_thrust_(0.0),
    omega_(0.0, 0.0, 0.0),
    cmd_mode_(1) {}

Command::~Command() {}

bool Command::setCmdMode(const int mode) {
  if (mode != 0 && mode != 1) {
    return false;
  }
  cmd_mode_ = mode;
  return true;
}

bool Command::valid() const {
  return std::isfinite(t_) &&
         ((std::isfinite(collective_thrust_) && omega_.allFinite() &&
           (cmd_mode_ == robotcmd::THRUSTRATE)) ||
          (raw_.allFinite() && (cmd_mode_ == robotcmd::RAW)));
}

bool Command::isRaw() const {
  return (cmd_mode_ == robotcmd::RAW) && raw_.allFinite();
}

bool Command::setRaw(const Vector<Dynamic> raw) {
  raw_ = raw;
  return true;
}

Vector<Dynamic> Command::getRaw() const {
  return raw_;
}

bool Command::isThrustRates() const {
  return (cmd_mode_ == robotcmd::THRUSTRATE) &&
         (std::isfinite(collective_thrust_) && omega_.allFinite());
}

bool Command::setThrustRates(Vector<Dynamic> thrust_rate) {
  collective_thrust_ = thrust_rate(0);
  omega_ = thrust_rate.segment<3>(1);
  return true;
}

Scalar Command::getCollectiveThrust() const {
  return collective_thrust_;
}

Vector<3> Command::getOmega() const {
  return omega_;
}


void Command::reset(Vector<Dynamic> mean) {
  t_ = 0.0;
  raw_ = mean;
  collective_thrust_ = 0;
  omega_.setZero();
}

}  // namespace flightlib