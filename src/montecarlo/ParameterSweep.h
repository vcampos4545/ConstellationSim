#pragma once

#include "core/ConfigLoader.h"
#include <vector>

// Generates the set of SimConfigs for a Monte Carlo experiment.
// Supports two strategies:
//   "grid"   — Cartesian product of all parameter value lists
//   "random" — Random sampling from each parameter list (with replacement)
class ParameterSweep {
public:
    explicit ParameterSweep(const MCConfig& mc_cfg);

    // Generate all run configs.
    std::vector<SimConfig> generateConfigs() const;

    int totalRuns() const { return total_runs_; }

private:
    const MCConfig& mc_cfg_;
    int total_runs_;

    std::vector<SimConfig> gridSweep() const;
    std::vector<SimConfig> randomSweep() const;
};
