#include "physics/J2Perturbation.h"

Vec3 J2Perturbation::acceleration(const OrbitState& state,
                                   const PhysicalProperties& /*props*/,
                                   double /*time_s*/) const {
    const Vec3& r = state.position;
    const double r2  = r.normSq();
    const double rmag = std::sqrt(r2);
    const double r5  = r2 * r2 * rmag;
    const double z2_r2 = (r.z * r.z) / r2;
    const double coeff = 1.5 * j2_ * mu_ * re_ * re_ / r5;

    return {
         coeff * r.x * (5.0*z2_r2 - 1.0),
         coeff * r.y * (5.0*z2_r2 - 1.0),
         coeff * r.z * (5.0*z2_r2 - 3.0)
    };
}
