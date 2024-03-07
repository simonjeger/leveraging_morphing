
#include "flightlib/objects/static_sphere.hpp"

namespace flightlib {

StaticSphere::StaticSphere(const std::string& id, const std::string& prefab_id)
  : UnityObject(id, prefab_id) {
  id_ = id;
  prefab_id_ = prefab_id;

  //
  sphere_state_.setZero();
  size_ << 1.0, 1.0, 1.0;
}

StaticSphere::~StaticSphere() {}

Vector<3> StaticSphere::getPos(void) { return sphere_state_.p; }

Quaternion StaticSphere::getQuat(void) { return sphere_state_.q(); }

Vector<3> StaticSphere::getSize(void) { return size_; }

Vector<3> StaticSphere::getScale(void) { return scale_; }


}  // namespace flightlib