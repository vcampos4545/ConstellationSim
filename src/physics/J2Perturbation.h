#pragma once
#include "physics/ForceModel.h"

// J2 oblateness perturbation.
// Models Earth's equatorial bulge (dominant perturbation for LEO).
// a_J2 = 3/2 * J2 * mu * R_E^2 / r^5 * [x*(5z^2/r^2 - 1),
//                                          y*(5z^2/r^2 - 1),
//                                          z*(5z^2/r^2 - 3)]
class J2Perturbation final : public ForceModel {
public:
    J2Perturbation(double mu, double j2, double earth_radius_m)
        : mu_(mu), j2_(j2), re_(earth_radius_m) {}

    Vec3 acceleration(const OrbitState& state,
                      const PhysicalProperties& /*props*/,
                      double /*time_s*/) const override;

    std::string name() const override { return "J2"; }

private:
    double mu_, j2_, re_;
};
