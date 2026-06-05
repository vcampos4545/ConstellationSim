#include "gnc/sensors/IMUModel.h"
#include <cmath>

IMUModel::IMUModel(const IMUSensorConfig& cfg, unsigned seed)
    : cfg_(cfg),
      rng_(seed),
      noise_dist_(0.0, cfg.gyro_noise_rad_s),
      bias_walk_dist_(0.0, cfg.gyro_bias_rad_s)
{}

ImuMeasurement IMUModel::sample(const Vec3& true_omega_body, double dt) {
    if (!cfg_.enabled) return last_;

    // Bias random walk step
    const double bw = bias_walk_dist_(rng_) * std::sqrt(dt);
    bias_.x += bw; bias_.y += bias_walk_dist_(rng_) * std::sqrt(dt);
    bias_.z += bias_walk_dist_(rng_) * std::sqrt(dt);

    // White noise (power spectral density → std = sigma / sqrt(dt))
    const double n_scale = 1.0 / std::sqrt(dt > 0 ? dt : 1.0);
    last_.gyro_rad_s = {
        true_omega_body.x + bias_.x + noise_dist_(rng_) * n_scale,
        true_omega_body.y + bias_.y + noise_dist_(rng_) * n_scale,
        true_omega_body.z + bias_.z + noise_dist_(rng_) * n_scale
    };
    last_.valid = true;
    return last_;
}
