#pragma once
#include "physics/ForceModel.h"
#include "environment/EarthModel.h"

// Atmospheric drag perturbation.
// a_drag = -0.5 * rho * Cd * (A/m) * |v_rel| * v_rel
// where v_rel = v_sat - v_atm  (atmosphere co-rotates with Earth)
class AtmosphericDrag final : public ForceModel {
public:
    explicit AtmosphericDrag(double earth_radius_m) : re_(earth_radius_m) {}

    Vec3 acceleration(const OrbitState& state,
                      const PhysicalProperties& props,
                      double time_s) const override;

    std::string name() const override { return "AtmosphericDrag"; }

    // Expose the last computed drag acceleration magnitude [m/s^2]
    // so the metrics system can accumulate it without calling acceleration() again.
    mutable double last_drag_accel_mag{0.0};

private:
    double re_;
};
