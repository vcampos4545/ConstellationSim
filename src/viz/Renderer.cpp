#ifdef CONSTELLATION_VIZ_ENABLED

#include "viz/Renderer.h"

Renderer::Renderer(FrameVec frames, int width, int height)
    : frames_(std::move(frames))
{
    sat_renderer_ = std::make_unique<SatelliteRenderer>(frames_, width, height);
}

void Renderer::run() {
    sat_renderer_->run();
}

#endif // CONSTELLATION_VIZ_ENABLED
