#pragma once
#include "core/math/Vec3.h"

// ECI (Earth-Centered Inertial) state vector.
// Position in meters, velocity in m/s, time in seconds since simulation epoch.
struct OrbitState {
    Vec3   position;  // [m] ECI
    Vec3   velocity;  // [m/s] ECI
    double time_s{0.0};
};
