#pragma once

#ifdef CONSTELLATION_VIZ_ENABLED

#include "viz/FrameQueue.h"
#include "core/ConfigLoader.h"
#include "core/math/Vec3.h"
#include <vgl/vgl.h>
#include <vgl/OrbitalCamera.h>
#include <memory>
#include <vector>
#include <deque>

// Streams and renders live simulation frames from a FrameQueue.
//
// The simulation runs on a background thread and pushes frames to the queue
// as they are computed. The renderer opens a window immediately, consuming
// frames as they become available. Satellite positions are linearly
// interpolated between consecutive frames so playback is smooth at any speed.
//
// Controls:
//   Space           — pause / resume
//   1 / 2 / 3 / 4  — speed presets: 1×, 10×, 100×, 1000×
//   + / =           — step up to next speed preset
//   -               — step down to previous speed preset
//   Left drag       — orbit camera
//   Right drag      — pan target
//   Scroll          — zoom
class SatelliteRenderer {
public:
    static constexpr float  SCENE_SCALE     = 1.0f / 6'378'137.0f;
    static constexpr float  EARTH_DISPLAY_R = 1.0f;
    static constexpr float  SAT_DOT_R       = 0.012f;

    static constexpr int TRAIL_FRAMES   = 90;
    static constexpr int TRAIL_MAX_SATS = 200;

    SatelliteRenderer(std::shared_ptr<FrameQueue> queue,
                      std::vector<GroundTarget>   ground_targets = {},
                      double                      min_elevation_deg = 10.0,
                      int window_w = 1280, int window_h = 720);

    // Blocks until the window is closed.
    void run();

private:
    std::shared_ptr<FrameQueue> queue_;
    GUI           gui_;
    OrbitalCamera orbital_cam_;

    // ---------------------------------------------------------------------------
    // Playback state
    // ---------------------------------------------------------------------------
    struct Playback {
        double sim_time_s{0.0};
        double speed{1.0};
        bool   paused{false};
        double wall_prev{-1.0};
    } pb_;

    static constexpr double SPEED_PRESETS[4] = {1.0, 10.0, 100.0, 1000.0};
    int speed_preset_idx_{0};

    glm::vec2 prev_mouse_pos_{0.0f};

    // ---------------------------------------------------------------------------
    // Textures
    // ---------------------------------------------------------------------------
    Texture earth_tex_;
    Texture star_tex_;

    static constexpr double EARTH_ROT_RATE = 7.2921159e-5;  // rad/s

    // ---------------------------------------------------------------------------
    // Per-render-tick interpolated state (owned by render thread only)
    // ---------------------------------------------------------------------------
    std::vector<glm::vec3> interp_pos_;  // scene-space, interpolated between frames
    std::vector<bool>      interp_ecl_;
    glm::vec3              interp_sun_{1.0f, 0.0f, 0.0f};
    int                    lo_frame_idx_{-1};

    // Trail ring-buffer: trail_buf_[sat_i] = deque of past scene-space positions
    std::vector<std::deque<glm::vec3>> trail_buf_;

    // ---------------------------------------------------------------------------
    // Ground targets
    // ---------------------------------------------------------------------------
    struct GroundTargetViz {
        std::string name;
        Vec3        pos_ecef;  // ECEF position [m]
    };
    std::vector<GroundTargetViz> ground_targets_;
    float                        min_elev_sin_{0.0f};   // sin(min_elevation_deg)
    std::vector<glm::vec3>       gt_scene_pos_;         // updated each tick

    static constexpr float GT_MARKER_R = 0.018f;

    void drawGroundTargets();
    void drawGroundLinks();

    // ---------------------------------------------------------------------------
    // Internal helpers
    // ---------------------------------------------------------------------------
    void handleInput();
    void advancePlayback();
    void buildInterpState();   // queries queue, updates interp_* members
    void updateWindowTitle();

    void drawStarBackground();
    void drawEarth();
    void drawSunIndicator();
    void drawSatellites();
    void drawTrails();

    glm::vec3 eciToScene(const Vec3& eci_m) const;
};

#endif // CONSTELLATION_VIZ_ENABLED
