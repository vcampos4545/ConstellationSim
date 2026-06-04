#ifdef CONSTELLATION_VIZ_ENABLED

#include "viz/Renderer.h"

Renderer::Renderer(std::shared_ptr<FrameQueue> queue,
                   std::vector<GroundTarget>   ground_targets,
                   double                      min_elevation_deg,
                   int width, int height)
    : queue_(std::move(queue))
{
    sat_renderer_ = std::make_unique<SatelliteRenderer>(
        queue_, std::move(ground_targets), min_elevation_deg, width, height);
}

void Renderer::run() {
    sat_renderer_->run();
}

#endif // CONSTELLATION_VIZ_ENABLED
