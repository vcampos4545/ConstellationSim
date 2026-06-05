#pragma once
#include "core/ConfigLoader.h"
#include "gnc/sensors/SensorMeasurements.h"
#include "orbit/OrbitState.h"
#include <random>

class GPSModel {
public:
    explicit GPSModel(const GPSSensorConfig& cfg, unsigned seed = 42);

    GpsMeasurement sample(const OrbitState& true_state);
    const GpsMeasurement& last() const { return last_; }

private:
    GPSSensorConfig cfg_;
    GpsMeasurement  last_;
    std::mt19937    rng_;
    std::normal_distribution<double> pos_noise_;
    std::normal_distribution<double> vel_noise_;
};
