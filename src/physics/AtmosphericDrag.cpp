#include "physics/AtmosphericDrag.h"
#include "environment/AtmosphereModel.h"
#include <cmath>

Vec3 AtmosphericDrag::acceleration(const OrbitState& state,
                                    const PhysicalProperties& props,
                                    double /*time_s*/) const {
    const double alt_m = state.position.norm() - re_;
    const double rho   = AtmosphereModel::density(alt_m);

    // Velocity relative to atmosphere (co-rotating with Earth)
    const Vec3 v_atm = EarthModel::atmosphericVelocity(state.position);
    const Vec3 v_rel = state.velocity - v_atm;
    const double v_rel_mag = v_rel.norm();

    if (v_rel_mag < 1e-10 || rho < 1e-30) {
        last_drag_accel_mag = 0.0;
        return {};
    }

    // Ballistic coefficient: Cd * A / m
    const double beta = props.drag_coefficient * props.drag_area_m2 / props.mass_kg;
    const double accel_mag = 0.5 * rho * beta * v_rel_mag * v_rel_mag;
    last_drag_accel_mag = accel_mag;

    return (-accel_mag / v_rel_mag) * v_rel;
}
