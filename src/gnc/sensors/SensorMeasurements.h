#pragma once
#include "core/math/Vec3.h"
#include "core/math/Quat.h"

// Noisy measurement outputs from each sensor model.
// FSW reads these; it never sees the true physics state directly.

struct GpsMeasurement {
    Vec3 position;      // ECI [m]   — noisy
    Vec3 velocity;      // ECI [m/s] — noisy
    bool valid{false};
};

struct ImuMeasurement {
    Vec3 gyro_rad_s;    // body frame angular rate [rad/s] — biased + noisy
    bool valid{false};
};

struct StarTrackerMeasurement {
    Quat attitude;      // body-to-ECI estimate — noisy
    bool valid{false};
};
