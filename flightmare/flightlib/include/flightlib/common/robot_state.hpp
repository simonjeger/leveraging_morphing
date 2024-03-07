#pragma once

#include <math.h>

#include <memory>
#include <iostream>

#include "flightlib/common/types.hpp"

namespace flightlib {

struct RobotState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum IDX : int {
    // position [m] (NWU global frame)
    POS = 0,
    POSX = 0,
    POSY = 1,
    POSZ = 2,
    NPOS = 3,
    // quaternion (NWU global frame)
    ATT = 3,
    ATTW = 3,
    ATTX = 4,
    ATTY = 5,
    ATTZ = 6,
    NATT = 4,
    // linear velocity [m/s] (NWU world frame for multicopter, FLU body frame for fixed wing / morphing wing)
    VEL = 7,
    VELX = 7,
    VELY = 8,
    VELZ = 9,
    NVEL = 3,
    // body rate [rad/s] (FLU body frame)
    OME = 10,
    OMEX = 10,
    OMEY = 11,
    OMEZ = 12,
    NOME = 3,
    // linear acceleration [m/s^2] (FLU body frame)
    ACC = 13,
    ACCX = 13,
    ACCY = 14,
    ACCZ = 15,
    NACC = 3,
    // angular acceleration [rad/s^2] (FLU body frame)
    AAC = 16,
    AACX = 16,
    AACY = 17,
    AACZ = 18,
    NAAC = 3,
    // actuator position
    ACT = 19,
    ACT0 = 19,
    ACT1 = 20,
    ACT2 = 21,
    ACT3 = 22,
    ACT4 = 23,
    ACT5 = 24,
    ACT6 = 25,
    ACT7 = 26,
    NACT = 8,
    //
    SIZE = 27,
    NDYM = 19
  };

  RobotState();
  RobotState(const Vector<IDX::SIZE>& x, const Scalar t = NAN);
  RobotState(const RobotState& state);
  ~RobotState();

  inline static int size() { return SIZE; }
  Quaternion q() const;
  void q(const Quaternion quaternion);
  Vector<3> e() const;
  Matrix<3, 3> R() const;

  void setZero();
  inline bool valid() const { return x.allFinite() && std::isfinite(t); }
  Matrix<3, Dynamic> moveEuclWorldToEuclBodyFrame(Matrix<3, Dynamic> points);
  Matrix<3, Dynamic> moveEuclWorldToPolarBodyFrame(Matrix<3, Dynamic> points);
  Matrix<3, Dynamic> rotEuclWorldToEuclBodyFrame(Matrix<3, Dynamic> directions);
  Matrix<3, Dynamic> rotEuclBodyToEuclWorldFrame(Matrix<3, Dynamic> directions);
  Matrix<3, Dynamic> rotEuclWorldToNEDWorldFrame(Matrix<3, Dynamic> directions);
  Matrix<3, Dynamic> rotNEDWorldToEuclWorldFrame(Matrix<3, Dynamic> directions);
  Matrix<3, Dynamic> rotEuclWorldToFRDBodyFrame(Matrix<3, Dynamic> directions);
  Matrix<3, Dynamic> rotFRDBodyToEuclWorldFrame(Matrix<3, Dynamic> directions);
  Matrix<3, Dynamic> rotEuclBodyToFRDBodyFrame(Matrix<3, Dynamic> directions);
  Matrix<3, Dynamic> rotFRDBodyToEuclBodyFrame(Matrix<3, Dynamic> directions);

  Matrix<3, 3> rotEul(Scalar alpha, Scalar beta, Scalar gamma);

  Vector<IDX::SIZE> x = Vector<IDX::SIZE>::Constant(NAN);
  Scalar t{NAN};

  // position
  Ref<Vector<3>> p{x.segment<IDX::NPOS>(IDX::POS)};
  // orientation (quaternion)
  Ref<Vector<4>> qx{x.segment<IDX::NATT>(IDX::ATT)};
  // linear velocity (quadrotors: world frame, fixed wings: body frame)
  Ref<Vector<3>> v{x.segment<IDX::NVEL>(IDX::VEL)};
  // angular velocity
  Ref<Vector<3>> w{x.segment<IDX::NOME>(IDX::OME)};
  // linear accleration
  Ref<Vector<3>> a{x.segment<IDX::NACC>(IDX::ACC)};
  // body torque
  Ref<Vector<3>> aa{x.segment<IDX::NAAC>(IDX::AAC)};

  bool modelInRange = true;

  bool operator==(const RobotState& rhs) const {
    return t == rhs.t && x.isApprox(rhs.x, 1e-5);
  }

  friend std::ostream& operator<<(std::ostream& os, const RobotState& state);
};

using STATE = RobotState;

}  // namespace flightlib