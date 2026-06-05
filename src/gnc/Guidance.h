#pragma once
#include "core/math/Quat.h"
#include "orbit/OrbitState.h"
#include <string>

enum class ADCSMode {
    OFF,            // no attitude control
    NADIR,          // body -Z toward Earth center (LVLH)
    SUN_POINTING,   // body +X toward Sun
    INERTIAL_HOLD   // maintain a fixed inertial attitude
};

ADCSMode adcsModeFromString(const std::string& s);
std::string  adcsModeToString(ADCSMode mode);

namespace Guidance {
    // Compute the target body-to-ECI quaternion for the given mode.
    // For INERTIAL_HOLD, target_inertial is used directly.
    Quat targetAttitude(ADCSMode mode,
                        const OrbitState& orbit,
                        const Vec3& sun_dir_eci,
                        const Quat& target_inertial = Quat::identity());
}
