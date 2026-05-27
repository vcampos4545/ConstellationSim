#pragma once
#include "physics/ForceModel.h"
#include "environment/SunModel.h"
#include "environment/EclipseModel.h"

// Solar radiation pressure perturbation.
// a_SRP = -P_sun * Cr * (A/m) * r_sat_to_sun_hat
// Shadow function sets acceleration to zero during eclipse.
class SolarRadiationPressure final : public ForceModel {
public:
    SolarRadiationPressure(double epoch_jd, double earth_radius_m)
        : epoch_jd_(epoch_jd), re_(earth_radius_m) {}

    Vec3 acceleration(const OrbitState& state,
                      const PhysicalProperties& props,
                      double time_s) const override;

    std::string name() const override { return "SolarRadiationPressure"; }

private:
    double epoch_jd_;
    [[maybe_unused]] double re_;  // reserved for future penumbra-based masking
};
