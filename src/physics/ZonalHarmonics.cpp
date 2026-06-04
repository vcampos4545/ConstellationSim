#include "physics/ZonalHarmonics.h"
#include <cmath>

Vec3 ZonalHarmonics::acceleration(const OrbitState& state,
                                   const PhysicalProperties&,
                                   double) const
{
    const Vec3&  r    = state.position;
    const double r2   = r.normSq();
    const double rmag = std::sqrt(r2);
    const double s    = r.z / rmag;          // sin(geocentric latitude)
    const double s2   = s * s;

    // J2
    const double re2    = re_ * re_;
    const double r5     = r2 * r2 * rmag;
    const double coeff2 = 1.5 * j2_ * mu_ * re2 / r5;
    Vec3 a = {
        coeff2 * r.x * (5.0*s2 - 1.0),
        coeff2 * r.y * (5.0*s2 - 1.0),
        coeff2 * r.z * (5.0*s2 - 3.0)
    };

    // J3 (odd harmonic — asymmetric N/S)
    if (j3_ != 0.0) {
        const double s3     = s2 * s;
        const double s4     = s2 * s2;
        const double re3    = re2 * re_;
        const double r6     = r2 * r2 * r2;
        const double coeff3 = mu_ * j3_ * re3 / r6;
        const double xy3    = 0.5 * (35.0*s3 - 15.0*s);
        const double z3     = 0.5 * (35.0*s4 - 30.0*s2 + 3.0);
        a.x += coeff3 * r.x  * xy3;
        a.y += coeff3 * r.y  * xy3;
        a.z += coeff3 * rmag * z3;
    }

    // J4
    if (j4_ != 0.0) {
        const double s3     = s2 * s;
        const double s4     = s2 * s2;
        const double s5     = s4 * s;
        const double re4    = re2 * re2;
        const double r7     = r2 * r2 * r2 * rmag;
        const double coeff4 = mu_ * j4_ * re4 / r7;
        const double xy4    = (315.0*s4 - 210.0*s2 + 15.0) / 8.0;
        const double z4     = (315.0*s5 - 350.0*s3 + 75.0*s) / 8.0;
        a.x += coeff4 * r.x  * xy4;
        a.y += coeff4 * r.y  * xy4;
        a.z += coeff4 * rmag * z4;
    }

    return a;
}
