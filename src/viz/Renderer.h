#pragma once

#ifdef CONSTELLATION_VIZ_ENABLED

#include "viz/SatelliteRenderer.h"
#include "core/SimulationEngine.h"
#include <vector>
#include <memory>

// Owns the pre-captured frame vector and drives the SatelliteRenderer.
//
// Flow:
//   1. SimulationEngine::runAndCapture() produces the frame vector.
//   2. main() moves the vector into Renderer.
//   3. Renderer::run() opens the window and starts playback — blocks until closed.
class Renderer {
public:
    using FrameVec = std::vector<SimulationEngine::FrameData>;

    explicit Renderer(FrameVec frames, int width = 1280, int height = 720);

    // Blocks until the window is closed.
    void run();

    int frameCount() const { return static_cast<int>(frames_.size()); }

private:
    FrameVec                           frames_;
    std::unique_ptr<SatelliteRenderer> sat_renderer_;
};

#endif // CONSTELLATION_VIZ_ENABLED
