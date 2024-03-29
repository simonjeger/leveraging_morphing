#pragma once

#include "flightlib/common/rigid_state.hpp"
#include "flightlib/objects/unity_object.hpp"

namespace flightlib {
class StaticSphere : public UnityObject {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StaticSphere(const std::string& id, const std::string& prefab_id = "rpg_sphere");
  ~StaticSphere();

  //
  Vector<3> getPos(void) override;
  Quaternion getQuat(void) override;

  //
  Vector<3> getSize(void) override;
  Vector<3> getScale(void) override;


  // publich set functions
  inline void setPosition(const Vector<3>& position) {
    sphere_state_.p = position;
  };
  inline void setRotation(const Quaternion& quaternion) {
    sphere_state_.q(quaternion);
  };
  inline void setSize(const Vector<3>& size) { size_ = size; };
  inline void setScale(const Vector<3>& scale) { scale_ = scale; };

 private:
  std::string id_;
  std::string prefab_id_;

  RigidState sphere_state_;
  Vector<3> size_;
  Vector<3> scale_;
};

}  // namespace flightlib
