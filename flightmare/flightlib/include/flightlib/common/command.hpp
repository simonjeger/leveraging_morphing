#pragma once

#include <iostream>
#include <stdlib.h>

#include "flightlib/common/types.hpp"

namespace flightlib {

namespace robotcmd {

enum CMDMODE : int {
  RAW = 0,
  THRUSTRATE = 1,
};

}  // namespace robotcmd
class Command {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Command();
  ~Command();

  //
  bool valid() const;
  bool isRaw() const;
  bool setRaw(Vector<Dynamic> raw);
  Vector<Dynamic> getRaw() const;
  bool isThrustRates() const;
  bool setThrustRates(Vector<Dynamic> raw);
  Scalar getCollectiveThrust() const;
  Vector<3> getOmega() const;

  //
  void reset(const Vector<Dynamic> mean);
  void setCmdVector(const Vector<4>& cmd);
  bool setCmdMode(const int cmd_mode);

  /// time in [s]
  Scalar t_;

  /// Direct raw inputs
  Vector<Dynamic> raw_;

  /// Collective mass-normalized thrust in [m/s^2]
  Scalar collective_thrust_;

  /// Bodyrates in [rad/s]
  Vector<3> omega_;

  ///
  int cmd_mode_;
};

}  // namespace flightlib