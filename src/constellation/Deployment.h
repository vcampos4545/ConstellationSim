#pragma once
#include "core/math/Constants.h"

// Deployment delta-v estimates for raising satellites from launch orbit.
// These are rough Hohmann transfer estimates for trade study purposes.
namespace Deployment {

    struct DeploymentResult {
        double dv_total_ms;         // total delta-v for all satellites [m/s]
        double dv_per_sat_ms;       // per-satellite delta-v [m/s]
        double launch_to_op_dv_ms;  // launch orbit -> operational orbit [m/s]
    };

    // Estimate delta-v to raise from launch_alt_km to operational_alt_km (Hohmann).
    // mu = GM [m^3/s^2], Re = Earth radius [m]
    DeploymentResult estimateHohmann(double launch_alt_km,
                                     double operational_alt_km,
                                     int    num_satellites,
                                     double mu = Constants::GM_EARTH,
                                     double Re = Constants::EARTH_RADIUS_M);

} // namespace Deployment
