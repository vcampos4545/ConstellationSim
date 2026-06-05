#pragma once
#include "core/math/Quat.h"
#include "core/math/Vec3.h"

// Per-satellite attitude + rotational dynamics state.
// Integrated in the same RK4 step as the orbital state.
struct AttitudeState {
    Quat attitude{};        // body-to-ECI quaternion
    Vec3 omega{};           // body angular rate [rad/s]
    Vec3 h_wheels{};        // total reaction wheel angular momentum in body frame [N·m·s]
};
