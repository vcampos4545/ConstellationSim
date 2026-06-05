#pragma once
#include "core/ConfigLoader.h"
#include "gnc/sensors/SensorMeasurements.h"
#include <random>

class StarTrackerModel {
public:
    explicit StarTrackerModel(const StarTrackerSensorConfig& cfg, unsigned seed = 456);

    StarTrackerMeasurement sample(const Quat& true_attitude);
    const StarTrackerMeasurement& last() const { return last_; }

private:
    StarTrackerSensorConfig cfg_;
    StarTrackerMeasurement  last_;
    std::mt19937            rng_;
    std::normal_distribution<double> noise_dist_;
};
