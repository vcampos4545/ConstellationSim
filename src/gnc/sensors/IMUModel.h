#pragma once
#include "core/ConfigLoader.h"
#include "gnc/sensors/SensorMeasurements.h"
#include <random>

class IMUModel {
public:
    explicit IMUModel(const IMUSensorConfig& cfg, unsigned seed = 123);

    ImuMeasurement sample(const Vec3& true_omega_body, double dt);
    const ImuMeasurement& last()          const { return last_; }
    Vec3                  estimatedBias() const { return bias_; }

private:
    IMUSensorConfig  cfg_;
    ImuMeasurement   last_;
    Vec3             bias_{};
    std::mt19937     rng_;
    std::normal_distribution<double> noise_dist_;
    std::normal_distribution<double> bias_walk_dist_;
};
