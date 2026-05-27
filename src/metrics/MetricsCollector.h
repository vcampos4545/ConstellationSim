#pragma once

#include "core/ConfigLoader.h"
#include "orbit/Satellite.h"
#include "environment/EclipseModel.h"
#include "environment/SunModel.h"
#include "environment/EarthModel.h"
#include "environment/AtmosphereModel.h"
#include <vector>
#include <string>
#include <unordered_map>

// ---------------------------------------------------------------------------
// Satellite-level result (one per satellite per simulation run)
// ---------------------------------------------------------------------------
struct SatelliteResult {
    int    run_id{0};
    int    sat_id{0};
    int    plane_id{0};
    int    seat_id{0};
    double time_in_sunlight_pct{0.0};  // [%]
    double time_in_eclipse_pct{0.0};   // [%]
    double avg_drag_accel_ms2{0.0};    // average drag acceleration [m/s^2]
    double total_drag_dv_ms{0.0};      // accumulated drag ΔV [m/s]
    double stationkeeping_dv_ms{0.0};  // estimated SK ΔV [m/s]
    double avg_altitude_km{0.0};       // [km]
    double min_altitude_km{1e9};       // [km]
    double orbital_lifetime_days{0.0}; // rough estimate
};

// ---------------------------------------------------------------------------
// Constellation-level result (one per simulation run)
// ---------------------------------------------------------------------------
struct ConstellationResult {
    int    run_id{0};
    std::string run_name;

    // Constellation parameters (echoed from config)
    double altitude_km{0.0};
    double inclination_deg{0.0};
    int    total_satellites{0};
    int    planes{0};
    int    sats_per_plane{0};

    // Coverage
    double coverage_pct{0.0};          // % of Earth covered (time-averaged)
    double revisit_time_avg_s{0.0};    // average revisit gap [s]
    double revisit_time_max_s{0.0};    // maximum revisit gap [s]

    // Ground access
    double avg_pass_duration_s{0.0};
    double avg_elevation_deg{0.0};

    // Inter-satellite
    double avg_nearest_neighbor_km{0.0};

    // Fleet averages
    double avg_drag_dv_ms{0.0};
    double avg_sk_dv_ms{0.0};
    double avg_sunlit_pct{0.0};
    double avg_altitude_km{0.0};
    double min_altitude_km{0.0};

    // Deployment
    double deployment_dv_per_sat_ms{0.0};
};

// ---------------------------------------------------------------------------
// MetricsCollector: updated each simulation timestep
// ---------------------------------------------------------------------------
class MetricsCollector {
public:
    explicit MetricsCollector(const MetricsConfig& cfg, const SimConfig& sim_cfg);

    // Called at each timestep with the current satellite states.
    void update(const std::vector<Satellite*>& satellites, double time_s);

    // Called once at end of simulation to finalize all metrics.
    ConstellationResult finalize(int run_id, const SimConfig& cfg);

    const std::vector<SatelliteResult>& satelliteResults() const { return sat_results_; }

private:
    MetricsConfig cfg_;
    double        epoch_jd_;
    double        total_time_s_{0.0};
    int           update_count_{0};

    // Per-satellite accumulation state
    struct SatAccum {
        double sunlit_s{0.0};
        double eclipse_s{0.0};
        double drag_dv_ms{0.0};
        double drag_accel_sum{0.0};
        int    drag_samples{0};
        double alt_sum_km{0.0};
        double min_alt_km{1e9};
        int    alt_samples{0};
    };
    std::vector<SatAccum> sat_accum_;
    int num_satellites_{0};

    // Coverage grid
    struct GridPoint {
        Vec3   pos_ecef;   // fixed ECEF position
        double last_coverage_time_s{-1e9};
        double revisit_sum_s{0.0};
        double max_revisit_s{0.0};
        int    revisit_count{0};
        bool   currently_covered{false};
        double coverage_time_s{0.0};
    };
    std::vector<GridPoint> grid_points_;
    double sample_timer_{0.0};
    int    coverage_samples_{0};
    double coverage_acc_{0.0};

    void initGrid(double resolution_deg);
    void updateCoverage(const std::vector<Satellite*>& sats, double time_s);
    void updateSatMetrics(const std::vector<Satellite*>& sats, double dt);

    std::vector<SatelliteResult> sat_results_;
};
