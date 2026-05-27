#pragma once
#include "core/math/Vec3.h"

// Low-precision Moon position for optional third-body perturbation.
namespace MoonModel {

    // Moon position in ECI [m] for Julian Date jd.
    // Accuracy: ~0.3 deg; adequate for perturbation force magnitude.
    Vec3 position_eci(double jd);

    Vec3 direction_eci(double t_s, double epoch_jd);

} // namespace MoonModel
