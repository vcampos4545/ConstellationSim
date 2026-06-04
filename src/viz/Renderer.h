#pragma once

#ifdef CONSTELLATION_VIZ_ENABLED

#include "viz/FrameQueue.h"
#include "viz/SatelliteRenderer.h"
#include <memory>

// Owns the FrameQueue reference and drives the SatelliteRenderer.
//
// Flow:
//   1. main() creates a FrameQueue and launches a background sim thread that pushes frames to it.
//   2. main() constructs Renderer(queue) — window opens immediately.
//   3. Renderer::run() streams and renders frames as they arrive — blocks until window is closed.
class Renderer {
public:
    explicit Renderer(std::shared_ptr<FrameQueue> queue,
                      std::vector<GroundTarget>   ground_targets = {},
                      double                      min_elevation_deg = 10.0,
                      int width = 1280, int height = 720);

    // Blocks until the window is closed.
    void run();

private:
    std::shared_ptr<FrameQueue>        queue_;
    std::unique_ptr<SatelliteRenderer> sat_renderer_;
};

#endif // CONSTELLATION_VIZ_ENABLED
