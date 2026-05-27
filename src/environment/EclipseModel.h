#pragma once
#include "core/math/Vec3.h"

// Cylindrical and conical shadow models.
// Cylindrical is fast and conservative (slightly overestimates eclipse time).
// Conical computes actual umbra/penumbra boundaries.
namespace EclipseModel {

    enum class ShadowState { Sunlit, Penumbra, Umbra };

    // Fast cylindrical shadow check.
    // Returns true if the satellite is in shadow (behind Earth relative to Sun).
    bool inEclipse(const Vec3& sat_pos_eci, const Vec3& sun_dir_eci);

    // Conical shadow model — returns the actual shadow state.
    ShadowState shadowState(const Vec3& sat_pos_eci,
                            const Vec3& sun_pos_eci,
                            double earth_radius_m,
                            double sun_radius_m);

} // namespace EclipseModel
