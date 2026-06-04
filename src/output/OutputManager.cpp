#include "output/OutputManager.h"
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

OutputManager::OutputManager(const std::string& base_dir,
                             const std::string& experiment_name) {
    exp_dir_ = base_dir + "/" + experiment_name;
    ensureDir(exp_dir_);
    summary_path_ = exp_dir_ + "/experiment_summary.csv";
    summary_writer_ = std::make_unique<CsvWriter>(summary_path_);
}

void OutputManager::ensureDir(const std::string& path) {
    fs::create_directories(path);
}

std::string OutputManager::runDirName(int run_id) {
    std::ostringstream oss;
    oss << "run_" << std::setw(4) << std::setfill('0') << run_id;
    return oss.str();
}

std::string OutputManager::runDir(int run_id) const {
    return exp_dir_ + "/" + runDirName(run_id);
}

void OutputManager::writeSummaryRow(CsvWriter& w, const ConstellationResult& cr) {
    w.writeRowV(cr.run_id, cr.run_name,
                cr.altitude_km, cr.inclination_deg,
                cr.total_satellites, cr.planes, cr.sats_per_plane,
                cr.coverage_pct,
                cr.revisit_time_avg_s,
                cr.revisit_time_max_s,
                cr.avg_drag_dv_ms,
                cr.avg_sk_dv_ms,
                cr.avg_sunlit_pct,
                cr.avg_altitude_km,
                cr.min_altitude_km,
                cr.deployment_dv_per_sat_ms);
}

void OutputManager::writeRun(int run_id,
                              const ConstellationResult& cr,
                              const std::vector<SatelliteResult>& sat_results,
                              const std::vector<GroundTargetResult>& gt_results) {
    std::lock_guard lock(mutex_);

    // Write experiment-level summary header once
    if (!summary_header_written_) {
        summary_writer_->writeHeader({
            "run_id","run_name",
            "altitude_km","inclination_deg",
            "total_satellites","planes","sats_per_plane",
            "coverage_pct",
            "revisit_time_avg_s","revisit_time_max_s",
            "avg_drag_dv_ms","avg_sk_dv_ms",
            "avg_sunlit_pct","avg_altitude_km","min_altitude_km",
            "deployment_dv_per_sat_ms"
        });
        summary_header_written_ = true;
    }

    writeSummaryRow(*summary_writer_, cr);
    summary_writer_->flush();

    // Per-run directory and files
    const std::string rdir = runDir(run_id);
    ensureDir(rdir);

    // Run summary (single-row CSV)
    {
        CsvWriter w(rdir + "/summary.csv");
        w.writeHeader({
            "run_id","run_name",
            "altitude_km","inclination_deg",
            "total_satellites","planes","sats_per_plane",
            "coverage_pct","revisit_time_avg_s","revisit_time_max_s",
            "avg_drag_dv_ms","avg_sk_dv_ms",
            "avg_sunlit_pct","avg_altitude_km","min_altitude_km",
            "deployment_dv_per_sat_ms"
        });
        writeSummaryRow(w, cr);
    }

    // Satellite results
    writeSatelliteCsv(rdir + "/satellites.csv", sat_results);

    // Ground target results (only written if any targets were configured)
    if (!gt_results.empty())
        writeGroundTargetCsv(rdir + "/ground_targets.csv", gt_results);
}

void OutputManager::writeSatelliteCsv(const std::string& path,
                                       const std::vector<SatelliteResult>& sats) {
    CsvWriter w(path);
    w.writeHeader({
        "run_id","satellite_id","plane_id","seat_id",
        "time_in_sunlight_pct","time_in_eclipse_pct",
        "avg_drag_accel_ms2","total_drag_dv_ms",
        "stationkeeping_dv_ms","avg_altitude_km","min_altitude_km",
        "orbital_lifetime_days"
    });
    for (const auto& s : sats) {
        w.writeRowV(s.run_id, s.sat_id, s.plane_id, s.seat_id,
                    s.time_in_sunlight_pct, s.time_in_eclipse_pct,
                    s.avg_drag_accel_ms2, s.total_drag_dv_ms,
                    s.stationkeeping_dv_ms, s.avg_altitude_km,
                    s.min_altitude_km, s.orbital_lifetime_days);
    }
}

void OutputManager::writeGroundTargetCsv(const std::string& path,
                                          const std::vector<GroundTargetResult>& gts) {
    CsvWriter w(path);
    w.writeHeader({
        "name","lat_deg","lon_deg",
        "visible_pct","illuminated_pct",
        "avg_elevation_deg","max_elevation_deg",
        "avg_pass_duration_s","pass_count",
        "coverage_time_s","illuminated_time_s"
    });
    for (const auto& g : gts) {
        w.writeRowV(g.name, g.lat_deg, g.lon_deg,
                    g.visible_pct, g.illuminated_pct,
                    g.avg_elevation_deg, g.max_elevation_deg,
                    g.avg_pass_duration_s, g.pass_count,
                    g.coverage_time_s, g.illuminated_time_s);
    }
}

void OutputManager::writeTrajectory(
    int run_id,
    const std::vector<SimulationEngine::OrbitalSnapshot>& snapshots)
{
    if (snapshots.empty()) return;
    const std::string path = runDir(run_id) + "/trajectory.csv";
    CsvWriter w(path);
    w.writeHeader({"time_s","sat_id","sma_km","eccentricity",
                   "inclination_deg","raan_deg","aop_deg",
                   "true_anomaly_deg","altitude_km"});
    for (const auto& s : snapshots) {
        w.writeRowV(s.time_s, s.sat_id, s.sma_km, s.eccentricity,
                    s.inclination_deg, s.raan_deg, s.aop_deg,
                    s.true_anomaly_deg, s.altitude_km);
    }
}

void OutputManager::finalize() {
    std::lock_guard lock(mutex_);
    summary_writer_->flush();
}
