#pragma once
#include "physics/ForceModel.h"

// Third-body gravitational perturbation (Sun or Moon).
//
// Perturbation acceleration in ECI:
//   a = mu_b * ( (r_b - r_s)/|r_b - r_s|^3  -  r_b/|r_b|^3 )
//
// The first term is the direct attraction; the second removes the
// acceleration that Earth itself experiences from the third body
// (i.e. keeps the equations in a truly inertial ECI frame).
//
// Body position is computed from the analytical SunModel / MoonModel
// at each timestep using the real Julian Date derived from epoch_jd.

enum class ThirdBodyType { Sun, Moon };

class ThirdBodyGravity final : public ForceModel {
public:
    ThirdBodyGravity(ThirdBodyType body, double epoch_jd);

    Vec3 acceleration(const OrbitState&          state,
                      const PhysicalProperties&  props,
                      double                     time_s) const override;

    std::string name() const override {
        return body_ == ThirdBodyType::Sun ? "SunGravity" : "MoonGravity";
    }

private:
    ThirdBodyType body_;
    double        mu_;        // gravitational parameter of the body [m^3/s^2]
    double        epoch_jd_;  // Julian Date of simulation epoch
};
