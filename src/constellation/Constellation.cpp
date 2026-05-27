#include "constellation/Constellation.h"
#include "orbit/OrbitalElements.h"
#include "core/math/Constants.h"

Constellation Constellation::createWalker(const WalkerConfig& cfg, int start_id) {
    Constellation c;

    const int T = cfg.total_satellites;
    const int P = cfg.planes;
    const int S = T / P;   // satellites per plane (assume integer division)
    const int F = cfg.phasing_factor;

    const double sma_m     = (Constants::EARTH_RADIUS_M + cfg.altitude_km * 1000.0);
    const double inc_rad   = cfg.inclination_deg * Constants::DEG2RAD;
    const double aop_rad   = cfg.arg_of_perigee_deg * Constants::DEG2RAD;
    const double mu        = Constants::GM_EARTH;

    const double d_raan = Constants::TWO_PI / P;      // RAAN step between planes
    const double d_nu   = Constants::TWO_PI / S;      // mean anomaly step within plane
    const double d_phase = F * Constants::TWO_PI / T; // cross-plane phasing offset

    c.planes_.reserve(P);
    int global_id = start_id;

    for (int p = 0; p < P; ++p) {
        const double raan = p * d_raan;
        Plane plane(p, inc_rad, raan);
        plane.satellites().reserve(S);

        for (int s = 0; s < S; ++s) {
            // True anomaly: in-plane spacing + cross-plane phase offset
            const double ta_rad = std::fmod(s * d_nu + p * d_phase, Constants::TWO_PI);

            OrbitalElements elems;
            elems.sma  = sma_m;
            elems.ecc  = cfg.eccentricity;
            elems.inc  = inc_rad;
            elems.raan = raan;
            elems.aop  = aop_rad;
            elems.ta   = ta_rad;

            const OrbitState state = elems.toStateVector(mu);

            PhysicalProperties props;  // uses defaults; caller should override if needed
            plane.addSatellite(Satellite(global_id++, p, s, state, props));
        }
        c.planes_.push_back(std::move(plane));
    }

    c.rebuildSatPtrs();
    return c;
}

void Constellation::rebuildSatPtrs() {
    sat_ptrs_.clear();
    for (auto& plane : planes_) {
        for (auto& sat : plane.satellites()) {
            sat_ptrs_.push_back(&sat);
        }
    }
}
