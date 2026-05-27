#pragma once
#include "core/ConfigLoader.h"
#include "metrics/MetricsCollector.h"
#include <vector>

// Result bundle for a single batch run.
struct BatchResult {
    int                        run_id;
    SimConfig                  config;
    ConstellationResult        constellation_result;
    std::vector<SatelliteResult> satellite_results;
};

// SimulationBatch holds a list of SimConfigs and executes them
// via the ExperimentManager's thread pool.
// This header is thin — the heavy lifting is in ExperimentManager.
class SimulationBatch {
public:
    explicit SimulationBatch(std::vector<SimConfig> configs)
        : configs_(std::move(configs)) {}

    const std::vector<SimConfig>& configs() const { return configs_; }
    int size() const { return static_cast<int>(configs_.size()); }

private:
    std::vector<SimConfig> configs_;
};
