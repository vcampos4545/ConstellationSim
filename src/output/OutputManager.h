#pragma once

#include "metrics/MetricsCollector.h"
#include "core/SimulationEngine.h"
#include "output/CsvWriter.h"
#include <string>
#include <filesystem>
#include <memory>
#include <mutex>

// OutputManager creates the per-run directory structure and writes CSV files.
//
// Directory layout:
//   <output_dir>/<experiment_name>/
//     run_0001/
//       summary.csv       — constellation-level result (one row)
//       satellites.csv    — per-satellite results
//     run_0002/ ...
//     experiment_summary.csv  — all runs combined (one row per run)
class OutputManager {
public:
    explicit OutputManager(const std::string& base_dir,
                           const std::string& experiment_name);

    // Write results for a single run. Thread-safe.
    void writeRun(int run_id,
                  const ConstellationResult& cr,
                  const std::vector<SatelliteResult>& sat_results,
                  const std::vector<GroundTargetResult>& gt_results = {});

    // Write orbital element trajectory snapshots (RAAN, inclination, etc.) to CSV.
    void writeTrajectory(int run_id,
                         const std::vector<SimulationEngine::OrbitalSnapshot>& snapshots);

    // Finalize: flush the experiment summary CSV.
    void finalize();

    std::string runDir(int run_id) const;
    std::string experimentDir() const { return exp_dir_; }

private:
    std::string exp_dir_;
    std::string summary_path_;
    std::mutex  mutex_;

    std::unique_ptr<CsvWriter> summary_writer_;
    bool summary_header_written_{false};

    static std::string runDirName(int run_id);
    void ensureDir(const std::string& path);
    void writeSummaryRow(CsvWriter& w, const ConstellationResult& cr);
    void writeSatelliteCsv(const std::string& path,
                           const std::vector<SatelliteResult>& sats);
    void writeGroundTargetCsv(const std::string& path,
                               const std::vector<GroundTargetResult>& gts);
};
