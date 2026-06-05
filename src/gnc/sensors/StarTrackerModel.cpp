#include "gnc/sensors/StarTrackerModel.h"

StarTrackerModel::StarTrackerModel(const StarTrackerSensorConfig& cfg, unsigned seed)
    : cfg_(cfg),
      rng_(seed),
      noise_dist_(0.0, cfg.attitude_noise_rad)
{}

StarTrackerMeasurement StarTrackerModel::sample(const Quat& true_attitude) {
    if (!cfg_.enabled) return last_;

    // Small random rotation noise about each axis.
    const Vec3 rv{noise_dist_(rng_), noise_dist_(rng_), noise_dist_(rng_)};
    const Quat noise_q = Quat::fromRotVec(rv);
    last_.attitude = (noise_q * true_attitude).normalized();
    last_.valid    = true;
    return last_;
}
