#include "constellation/Deployment.h"
#include <cmath>

Deployment::DeploymentResult Deployment::estimateHohmann(
    double launch_alt_km, double operational_alt_km,
    int num_satellites, double mu, double Re)
{
    const double r1 = Re + launch_alt_km * 1000.0;
    const double r2 = Re + operational_alt_km * 1000.0;

    const double v1_circ  = std::sqrt(mu / r1);
    const double v2_circ  = std::sqrt(mu / r2);
    const double v_trans1 = std::sqrt(2.0*mu*r2 / (r1*(r1+r2)));
    const double v_trans2 = std::sqrt(2.0*mu*r1 / (r2*(r1+r2)));

    const double dv1 = std::abs(v_trans1 - v1_circ);
    const double dv2 = std::abs(v2_circ  - v_trans2);
    const double dv_per_sat = dv1 + dv2;

    return {dv_per_sat * num_satellites, dv_per_sat, dv_per_sat};
}
