#include "metrics/MetricsCollector.h"
#include "environment/AtmosphereModel.h"
#include "core/math/Constants.h"
#include <cmath>
#include <algorithm>
#include <numeric>

MetricsCollector::MetricsCollector(const MetricsConfig& cfg, const SimConfig& sim_cfg)
    : cfg_(cfg), epoch_jd_(sim_cfg.epoch_jd)
{
    if (cfg_.coverage.enabled)
        initGrid(cfg_.coverage.grid_resolution_deg);

    for (const auto& gt : sim_cfg.ground_targets) {
        GroundTargetAccum a;
        a.name     = gt.name;
        a.lat_deg  = gt.lat_deg;
        a.lon_deg  = gt.lon_deg;
        a.pos_ecef = EarthModel::geodeticToECEF(gt.lat_deg, gt.lon_deg, 0.0);
        gt_accum_.push_back(std::move(a));
    }
}

void MetricsCollector::initGrid(double resolution_deg) {
    // Latitude from -90 to +90, longitude from -180 to +180
    // Use equal-angle grid (not equal-area, but sufficient for trade studies)
    grid_points_.clear();
    for (double lat = -90.0 + resolution_deg/2.0; lat < 90.0; lat += resolution_deg) {
        for (double lon = -180.0 + resolution_deg/2.0; lon < 180.0; lon += resolution_deg) {
            GridPoint gp;
            gp.pos_ecef = EarthModel::geodeticToECEF(lat, lon, 0.0);
            grid_points_.push_back(gp);
        }
    }
}

void MetricsCollector::update(const std::vector<Satellite*>& sats, double time_s) {
    if (sat_accum_.empty() && !sats.empty()) {
        num_satellites_ = static_cast<int>(sats.size());
        sat_accum_.resize(num_satellites_);
    }

    const double dt = (update_count_ == 0) ? 0.0 : time_s - total_time_s_;
    total_time_s_ = time_s;
    ++update_count_;

    updateSatMetrics(sats, dt);

    // Coverage is sampled at reduced interval for performance
    if (cfg_.coverage.enabled) {
        sample_timer_ += dt;
        if (sample_timer_ >= cfg_.coverage.sample_interval_s || update_count_ == 1) {
            updateCoverage(sats, time_s);
            sample_timer_ = 0.0;
        }
    }
}

void MetricsCollector::updateSatMetrics(const std::vector<Satellite*>& sats, double dt) {
    if (dt <= 0.0) return;

    const Vec3 sun_dir = SunModel::direction_eci(total_time_s_, epoch_jd_);

    for (int i = 0; i < static_cast<int>(sats.size()); ++i) {
        const Satellite* sat = sats[i];
        auto& acc = sat_accum_[i];

        // Sunlight / eclipse
        if (cfg_.sunlight || cfg_.drag) {
            const bool eclipsed = EclipseModel::inEclipse(sat->state().position, sun_dir);
            if (eclipsed) {
                acc.eclipse_s += dt;
            } else {
                acc.sunlit_s  += dt;
            }
        }

        // Drag ΔV estimate: ΔV = F_drag/m * dt
        if (cfg_.drag) {
            const double alt_m = sat->state().position.norm() - Constants::EARTH_RADIUS_M;
            const double rho   = AtmosphereModel::density(alt_m);
            const double v_rel = (sat->state().velocity -
                                  EarthModel::atmosphericVelocity(sat->state().position)).norm();
            const PhysicalProperties& p = sat->properties();
            const double beta = p.drag_coefficient * p.drag_area_m2 / p.mass_kg;
            const double drag_a = 0.5 * rho * beta * v_rel * v_rel;
            acc.drag_dv_ms += drag_a * dt;
            acc.drag_accel_sum += drag_a;
            ++acc.drag_samples;
        }

        // Altitude tracking
        const double alt_km = sat->state().position.norm() / 1000.0
                              - Constants::EARTH_RADIUS_KM;
        acc.alt_sum_km += alt_km;
        ++acc.alt_samples;
        acc.min_alt_km = std::min(acc.min_alt_km, alt_km);
    }
}

void MetricsCollector::updateCoverage(const std::vector<Satellite*>& sats, double time_s) {
    const double theta   = EarthModel::gstAtTime(time_s);
    const double sin_min = std::sin(cfg_.coverage.min_elevation_deg * Constants::DEG2RAD);
    const Vec3   sun_dir = SunModel::direction_eci(time_s, epoch_jd_);
    const double dt_s    = cfg_.coverage.sample_interval_s;

    // ---- global grid coverage ----
    int covered_count = 0;
    for (auto& gp : grid_points_) {
        const Vec3 gp_eci = EarthModel::ecefToECI(gp.pos_ecef, theta);
        const Vec3 gp_hat = gp_eci.normalized();

        bool covered = false;
        for (const Satellite* sat : sats) {
            const Vec3 slant = sat->state().position - gp_eci;
            if (gp_hat.dot(slant.normalized()) >= sin_min) { covered = true; break; }
        }

        if (covered) {
            ++covered_count;
            if (!gp.currently_covered && gp.last_coverage_time_s > -1e8) {
                const double gap = time_s - gp.last_coverage_time_s;
                gp.revisit_sum_s += gap;
                gp.max_revisit_s  = std::max(gp.max_revisit_s, gap);
                ++gp.revisit_count;
            }
            gp.coverage_time_s += dt_s;
            gp.last_coverage_time_s = time_s;
        } else if (gp.currently_covered) {
            gp.last_coverage_time_s = time_s;
        }
        gp.currently_covered = covered;
    }
    if (!grid_points_.empty()) {
        coverage_acc_ += static_cast<double>(covered_count) / grid_points_.size();
        ++coverage_samples_;
    }

    // ---- per-ground-target access ----
    for (auto& a : gt_accum_) {
        const Vec3 gt_eci = EarthModel::ecefToECI(a.pos_ecef, theta);
        const Vec3 gt_hat = gt_eci.normalized();

        double best_elev_deg   = -90.0;
        bool   any_visible     = false;
        bool   any_illuminated = false;

        for (const Satellite* sat : sats) {
            const Vec3   slant    = sat->state().position - gt_eci;
            const double sin_elev = gt_hat.dot(slant.normalized());
            if (sin_elev >= sin_min) {
                any_visible = true;
                const double elev_deg = std::asin(std::clamp(sin_elev, -1.0, 1.0))
                                        * Constants::RAD2DEG;
                if (elev_deg > best_elev_deg) best_elev_deg = elev_deg;
                if (!EclipseModel::inEclipse(sat->state().position, sun_dir))
                    any_illuminated = true;
            }
        }

        if (any_visible) {
            a.visible_s      += dt_s;
            a.elev_sum_deg   += best_elev_deg;
            a.max_elev_deg    = std::max(a.max_elev_deg, best_elev_deg);
            ++a.visible_samples;
            if (any_illuminated) a.illuminated_s += dt_s;
            if (!a.in_pass) { a.in_pass = true; a.pass_start_s = time_s; ++a.pass_count; }
        } else if (a.in_pass) {
            a.pass_dur_sum_s += time_s - a.pass_start_s;
            a.in_pass = false;
        }
    }
}

ConstellationResult MetricsCollector::finalize(int run_id, const SimConfig& cfg) {
    // Build per-satellite results
    sat_results_.resize(num_satellites_);
    double fleet_sunlit  = 0.0;
    double fleet_drag_dv = 0.0;
    double fleet_sk_dv   = 0.0;
    double fleet_alt     = 0.0;
    double fleet_min_alt = 1e9;

    for (int i = 0; i < num_satellites_; ++i) {
        auto& r = sat_results_[i];
        auto& a = sat_accum_[i];
        const double total_s = a.sunlit_s + a.eclipse_s;

        r.run_id   = run_id;
        r.sat_id   = i;
        r.time_in_sunlight_pct = (total_s > 0) ? 100.0*a.sunlit_s/total_s  : 0.0;
        r.time_in_eclipse_pct  = (total_s > 0) ? 100.0*a.eclipse_s/total_s : 0.0;
        r.avg_drag_accel_ms2   = (a.drag_samples > 0) ? a.drag_accel_sum/a.drag_samples : 0.0;
        r.total_drag_dv_ms     = a.drag_dv_ms;
        r.stationkeeping_dv_ms = a.drag_dv_ms;  // SK ΔV ≈ drag ΔV for circular orbits
        r.avg_altitude_km      = (a.alt_samples > 0) ? a.alt_sum_km/a.alt_samples : 0.0;
        r.min_altitude_km      = a.min_alt_km;

        // Orbital lifetime estimate via altitude decay rate.
        // For a circular orbit, drag ΔV lowers altitude at ~2a·Δv/v per impulse.
        // We extrapolate the measured daily decay to reach 120 km (reentry).
        const double avg_sma = Constants::EARTH_RADIUS_M + r.avg_altitude_km * 1000.0;
        const double orbital_v = std::sqrt(Constants::GM_EARTH / avg_sma);
        if (a.drag_dv_ms > 0.0 && total_s > 0.0) {
            // dv_per_day [m/s/day]: drag ΔV averaged over the simulation
            const double dv_per_day = a.drag_dv_ms / (total_s / Constants::SEC_PER_DAY);
            // Altitude decay per day: Δh = 2·a·Δv/v  [m/day]
            const double dh_per_day = 2.0 * avg_sma * dv_per_day / orbital_v;
            const double h_deorbit_m = 120e3;  // reentry altitude [m]
            const double h_current_m = r.avg_altitude_km * 1000.0;
            if (dh_per_day > 0.0 && h_current_m > h_deorbit_m) {
                r.orbital_lifetime_days = (h_current_m - h_deorbit_m) / dh_per_day;
            }
        }

        fleet_sunlit  += r.time_in_sunlight_pct;
        fleet_drag_dv += r.total_drag_dv_ms;
        fleet_sk_dv   += r.stationkeeping_dv_ms;
        fleet_alt     += r.avg_altitude_km;
        fleet_min_alt  = std::min(fleet_min_alt, r.min_altitude_km);
    }

    ConstellationResult cr;
    cr.run_id          = run_id;
    cr.run_name        = cfg.run_name;
    cr.altitude_km     = cfg.constellation.altitude_km;
    cr.inclination_deg = cfg.constellation.inclination_deg;
    cr.total_satellites = cfg.constellation.total_satellites;
    cr.planes           = cfg.constellation.planes;
    cr.sats_per_plane   = (cfg.constellation.planes > 0)
                        ? cfg.constellation.total_satellites / cfg.constellation.planes : 0;

    if (num_satellites_ > 0) {
        cr.avg_sunlit_pct = fleet_sunlit  / num_satellites_;
        cr.avg_drag_dv_ms = fleet_drag_dv / num_satellites_;
        cr.avg_sk_dv_ms   = fleet_sk_dv   / num_satellites_;
        cr.avg_altitude_km = fleet_alt    / num_satellites_;
        cr.min_altitude_km = fleet_min_alt;
    }

    // Coverage metrics
    if (cfg_.coverage.enabled && coverage_samples_ > 0) {
        cr.coverage_pct = 100.0 * coverage_acc_ / coverage_samples_;

        double revisit_sum = 0.0;
        double max_revisit = 0.0;
        int    n_revisit   = 0;

        for (const auto& gp : grid_points_) {
            if (gp.revisit_count > 0) {
                revisit_sum += gp.revisit_sum_s / gp.revisit_count;
                max_revisit  = std::max(max_revisit, gp.max_revisit_s);
                ++n_revisit;
            }
        }
        cr.revisit_time_avg_s = (n_revisit > 0) ? revisit_sum / n_revisit : 0.0;
        cr.revisit_time_max_s = max_revisit;
    }

    // Ground target results
    const double total_s = total_time_s_;
    gt_results_.resize(gt_accum_.size());
    for (size_t i = 0; i < gt_accum_.size(); ++i) {
        auto& a = gt_accum_[i];
        // Close any open pass
        if (a.in_pass) { a.pass_dur_sum_s += total_s - a.pass_start_s; a.in_pass = false; }

        auto& r = gt_results_[i];
        r.name              = a.name;
        r.lat_deg           = a.lat_deg;
        r.lon_deg           = a.lon_deg;
        r.coverage_time_s   = a.visible_s;
        r.illuminated_time_s = a.illuminated_s;
        r.visible_pct       = (total_s > 0) ? 100.0 * a.visible_s / total_s : 0.0;
        r.illuminated_pct   = (total_s > 0) ? 100.0 * a.illuminated_s / total_s : 0.0;
        r.avg_elevation_deg = (a.visible_samples > 0)
                              ? a.elev_sum_deg / a.visible_samples : 0.0;
        r.max_elevation_deg = a.max_elev_deg;
        r.pass_count        = a.pass_count;
        r.avg_pass_duration_s = (a.pass_count > 0)
                                ? a.pass_dur_sum_s / a.pass_count : 0.0;
    }

    return cr;
}
