#include "physics/SolarRadiationPressure.h"
#include "core/math/Constants.h"
#include <cmath>

Vec3 SolarRadiationPressure::acceleration(const OrbitState& state,
                                           const PhysicalProperties& props,
                                           double time_s) const {
    // Sun direction in ECI
    const Vec3 sun_dir = SunModel::direction_eci(time_s, epoch_jd_);

    // Skip if satellite is in eclipse (cylindrical model for speed)
    if (EclipseModel::inEclipse(state.position, sun_dir)) return {};

    // Sun position at ~1 AU; pressure scales with 1/r^2 but for LEO r_sun ≈ AU
    const Vec3 sun_pos = sun_dir * Constants::AU_M;
    const double r_sun_sat = (sun_pos - state.position).norm();
    const double P = Constants::SOLAR_PRESSURE_PA * (Constants::AU_M / r_sun_sat) * (Constants::AU_M / r_sun_sat);

    // Force direction: from Sun toward satellite
    const Vec3 sat_from_sun = (state.position - sun_pos).normalized();

    // Radiation coefficient: Cr * A / m
    const double coeff = P * props.reflectivity * props.srp_area_m2 / props.mass_kg;
    return coeff * sat_from_sun;
}
