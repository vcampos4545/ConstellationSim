#pragma once
#include "core/math/Vec3.h"

// Low-precision Sun position model.
// Returns Sun position in ECI [m] and unit direction from Earth to Sun.
// Accurate to ~1 degree; sufficient for eclipse and SRP calculations.
namespace SunModel {

    // Sun position in ECI [m] for Julian Date jd.
    Vec3 position_eci(double jd);

    // Unit vector from Earth center to Sun in ECI, at time t_s since epoch.
    // epoch_jd: Julian date of the simulation epoch.
    Vec3 direction_eci(double t_s, double epoch_jd);

} // namespace SunModel
