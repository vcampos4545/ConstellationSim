#include "gnc/sensors/GPSModel.h"

GPSModel::GPSModel(const GPSSensorConfig& cfg, unsigned seed)
    : cfg_(cfg),
      rng_(seed),
      pos_noise_(0.0, cfg.position_noise_m),
      vel_noise_(0.0, cfg.velocity_noise_ms)
{}

GpsMeasurement GPSModel::sample(const OrbitState& true_state) {
    if (!cfg_.enabled) return last_;

    last_.position = {
        true_state.position.x + pos_noise_(rng_),
        true_state.position.y + pos_noise_(rng_),
        true_state.position.z + pos_noise_(rng_)
    };
    last_.velocity = {
        true_state.velocity.x + vel_noise_(rng_),
        true_state.velocity.y + vel_noise_(rng_),
        true_state.velocity.z + vel_noise_(rng_)
    };
    last_.valid = true;
    return last_;
}
