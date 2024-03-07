#pragma once

// yaml cpp
#include <yaml-cpp/yaml.h>

#include <random>

#include "flightlib/common/robot_state.hpp"
#include "flightlib/common/types.hpp"

namespace flightlib {

class DynamicsBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  using DynamicsFunction =
    std::function<bool(const Ref<const Vector<>>, Ref<Vector<>>)>;

  DynamicsBase();
  virtual ~DynamicsBase();

  // main functions
  virtual DynamicsFunction getDynamicsFunction() const = 0;
  virtual Vector<7> getAccelerations(RobotState state, const Vector<Dynamic>& cmd_ll,
                                     const Scalar sim_dt, Vector<6> wind_curl, bool direct=false) = 0;

  // help functions
  virtual bool valid() const = 0;
  virtual bool reset() = 0;
  virtual bool randomizeParams() = 0;
  virtual bool setModelParams(const Vector<Dynamic>& params) = 0;
  virtual bool loadModelParams(const Vector<Dynamic>& params) = 0;
  virtual bool setActuator(const Vector<Dynamic>& actuator) = 0;

  // Helpers to apply limits.
  virtual Vector<Dynamic> clampRaw(const Vector<Dynamic> raw) const = 0;
  virtual Scalar clampCollectiveThrust(const Scalar thrust) const = 0;
  virtual Vector<3> clampTorque(const Vector<3>& torque) const = 0;
  virtual Vector<3> clampBodyrates(const Vector<3>& omega) const = 0;
  virtual Scalar clampVelu(const Scalar vel) const = 0;
  virtual Scalar getConsumption(void) const = 0;
  virtual Vector<Dynamic> getActuator() const = 0;

  // Get functions
  virtual Scalar getMass(void) const = 0;
  virtual Matrix<3, 3> getJ(void) const = 0;
  virtual Matrix<4, 4> getAllocationMatrix(void) const = 0;
  virtual Vector<3> getOmegaMax(void) const = 0;
  virtual Scalar getVeluMax(void) const = 0;
  virtual Vector<Dynamic> getRawMean(void) const = 0;
  virtual Vector<Dynamic> getRawStd(void) const = 0;
  virtual Scalar getForceMax(void) const = 0;


 private:
};

}  // namespace flightlib
