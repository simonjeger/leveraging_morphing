#include "flightlib/sensors/airspeed.hpp"


namespace flightlib {

Airspeed::Airspeed() {
    offset_std_ = 0.0;
    noise_std_ = 0.0;
    reset();
}

Airspeed::~Airspeed() {}

Scalar Airspeed::getMeasurement(Scalar u_rel){
    u_rel = std::max(0.0,u_rel); //the sensor only gives reasonable values when wind is positive
    Scalar dist = norm_dist_(random_gen_)*noise_std_ + offset_;
    return u_rel + dist;
}

void Airspeed::reset(){
    offset_ = uni_dist_(random_gen_)*offset_std_; // [m/s]
}

}  // namespace flightlib
