#pragma once

#ifdef CONSTELLATION_VIZ_ENABLED

#include "core/SimulationEngine.h"
#include <vgl/vgl.h>
#include <vgl/OrbitalCamera.h>
#include <vector>

// Plays back a pre-captured sequence of simulation frames.
//
// Controls:
//   Space        — pause / resume
//   1 / 2 / 3 / 4  — speed presets: 1×, 10×, 100×, 1000×
//   + / =        — step up to next speed preset
//   -            — step down to previous speed preset
//   Left drag    — orbit camera
//   Right drag   — pan target
//   Scroll       — zoom
//
// Playback loops when it reaches the end of the captured frames.
// Window title shows current sim time, speed, and pause state.
class SatelliteRenderer {
public:
    using FrameVec = std::vector<SimulationEngine::FrameData>;

    // Scene-space scale: 1 ECI meter -> scene units (Earth radius = 1.0)
    static constexpr float  SCENE_SCALE     = 1.0f / 6'378'137.0f;
    static constexpr float  EARTH_DISPLAY_R = 1.0f;
    static constexpr float  SAT_DOT_R       = 0.012f;

    // Number of trailing frames drawn per satellite.
    // Trails are drawn only when total satellite count <= TRAIL_MAX_SATS.
    static constexpr int TRAIL_FRAMES   = 90;
    static constexpr int TRAIL_MAX_SATS = 200;

    SatelliteRenderer(const FrameVec& frames,
                      int window_w = 1280, int window_h = 720);

    // Blocks until the window is closed.
    void run();

private:
    const FrameVec& frames_;
    GUI             gui_;
    OrbitalCamera   orbital_cam_;

    // ---------------------------------------------------------------------------
    // Playback state
    // ---------------------------------------------------------------------------
    struct Playback {
        double sim_time_s{0.0};   // current position in sim timeline [s]
        int    frame_idx{0};      // index into frames_ for current sim_time
        double speed{1.0};        // real-time multiplier
        bool   paused{false};
        double wall_prev{-1.0};   // glfwGetTime() at previous render frame
    } pb_;

    // Available speed presets (index into this array)
    static constexpr double SPEED_PRESETS[4] = {1.0, 10.0, 100.0, 1000.0};
    int speed_preset_idx_{0};

    // ---------------------------------------------------------------------------
    // Camera input state — OrbitalCamera needs an explicit delta per frame
    // ---------------------------------------------------------------------------
    glm::vec2 prev_mouse_pos_{0.0f};
    bool      first_mouse_frame_{true};

    // ---------------------------------------------------------------------------
    // Internal helpers
    // ---------------------------------------------------------------------------
    void handleInput();
    void advancePlayback();
    int  findFrameIdx(double sim_time_s) const;
    void updateWindowTitle();

    void drawEarth(const glm::vec3& sun_dir);
    void drawSunIndicator(const glm::vec3& sun_dir);
    void drawSatellites(int frame_idx);
    void drawTrails(int frame_idx);

    glm::vec3 eciToScene(const Vec3& eci_m) const;
};

#endif // CONSTELLATION_VIZ_ENABLED
