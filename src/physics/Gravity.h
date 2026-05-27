#pragma once
#include "physics/ForceModel.h"

// Point-mass two-body gravitational acceleration.
// a = -mu/r^3 * r_vec
class Gravity final : public ForceModel {
public:
    explicit Gravity(double mu) : mu_(mu) {}

    Vec3 acceleration(const OrbitState& state,
                      const PhysicalProperties& /*props*/,
                      double /*time_s*/) const override;

    std::string name() const override { return "Gravity"; }

private:
    double mu_;  // gravitational parameter [m^3/s^2]
};
