#ifdef CONSTELLATION_VIZ_ENABLED

#include "viz/SatelliteRenderer.h"
#include "core/math/Constants.h"
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <cstdio>
#include <algorithm>

constexpr double SatelliteRenderer::SPEED_PRESETS[4];

SatelliteRenderer::SatelliteRenderer(const FrameVec& frames,
                                     int window_w, int window_h)
    : frames_(frames),
      gui_(window_w, window_h, "ConstellationSim"),
      // Start with Earth filling roughly 1/3 of the view, pitched slightly down.
      orbital_cam_(3.5f, 25.0f, -20.0f, glm::vec3(0.0f))
{
    gui_.camera
        .setFOV(45.0f)
        .setClipPlanes(0.001f, 500.0f);

    // Logarithmic depth buffer eliminates z-fighting between Earth surface
    // and satellite dots (depth range spans ~7 orders of magnitude).
    gui_.setLogDepth(500.0f);
    gui_.setLighting(true);

    orbital_cam_
        .setMinDistance(1.05f)   // can't go inside Earth
        .setMaxDistance(20.0f)
        .setZoomSensitivity(0.3f)
        .setPanSensitivity(0.002f);

    pb_.wall_prev = glfwGetTime();
}

// ---------------------------------------------------------------------------
// Coordinate conversion
// ---------------------------------------------------------------------------
glm::vec3 SatelliteRenderer::eciToScene(const Vec3& eci_m) const {
    return {
        static_cast<float>(eci_m.x) * SCENE_SCALE,
        static_cast<float>(eci_m.y) * SCENE_SCALE,
        static_cast<float>(eci_m.z) * SCENE_SCALE
    };
}

// ---------------------------------------------------------------------------
// Playback control
// ---------------------------------------------------------------------------
int SatelliteRenderer::findFrameIdx(double sim_time_s) const {
    if (frames_.empty()) return 0;
    // Binary search: find the largest index where time_s <= sim_time_s
    int lo = 0, hi = static_cast<int>(frames_.size()) - 1;
    while (lo < hi) {
        const int mid = (lo + hi + 1) / 2;
        if (frames_[mid].time_s <= sim_time_s) lo = mid;
        else                                    hi = mid - 1;
    }
    return lo;
}

void SatelliteRenderer::advancePlayback() {
    const double wall_now = glfwGetTime();
    const double real_dt  = wall_now - pb_.wall_prev;
    pb_.wall_prev = wall_now;

    if (pb_.paused || frames_.empty()) return;

    pb_.sim_time_s += real_dt * pb_.speed;

    // Loop back to beginning when we reach the last frame
    const double max_t = frames_.back().time_s;
    if (pb_.sim_time_s > max_t) {
        pb_.sim_time_s = std::fmod(pb_.sim_time_s, max_t > 0.0 ? max_t : 1.0);
    }

    pb_.frame_idx = findFrameIdx(pb_.sim_time_s);
}

void SatelliteRenderer::updateWindowTitle() {
    const double t    = pb_.sim_time_s;
    const int days    = static_cast<int>(t / 86400.0);
    const int hrs     = static_cast<int>(std::fmod(t, 86400.0) / 3600.0);
    const int mins    = static_cast<int>(std::fmod(t, 3600.0) / 60.0);
    const int sats    = frames_.empty() ? 0
                        : static_cast<int>(frames_[pb_.frame_idx].positions.size());

    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "ConstellationSim | T+%dd %02dh %02dm | %d sats | Speed: %.0f×%s",
        days, hrs, mins, sats,
        pb_.speed,
        pb_.paused ? " [PAUSED]" : "");

    glfwSetWindowTitle(gui_.getWindow(), buf);
}

// ---------------------------------------------------------------------------
// Input handling
// ---------------------------------------------------------------------------
void SatelliteRenderer::handleInput() {
    // Pause / resume
    if (gui_.isKeyJustPressed(GLFW_KEY_SPACE)) {
        pb_.paused = !pb_.paused;
    }

    // Speed presets: keys 1–4
    const int preset_keys[4] = {GLFW_KEY_1, GLFW_KEY_2, GLFW_KEY_3, GLFW_KEY_4};
    for (int i = 0; i < 4; ++i) {
        if (gui_.isKeyJustPressed(preset_keys[i])) {
            speed_preset_idx_ = i;
            pb_.speed = SPEED_PRESETS[i];
        }
    }

    // +/- step through presets
    const bool inc = gui_.isKeyJustPressed(GLFW_KEY_EQUAL)
                   || gui_.isKeyJustPressed(GLFW_KEY_KP_ADD);
    const bool dec = gui_.isKeyJustPressed(GLFW_KEY_MINUS)
                   || gui_.isKeyJustPressed(GLFW_KEY_KP_SUBTRACT);
    if (inc) {
        speed_preset_idx_ = std::min(speed_preset_idx_ + 1, 3);
        pb_.speed = SPEED_PRESETS[speed_preset_idx_];
    }
    if (dec) {
        speed_preset_idx_ = std::max(speed_preset_idx_ - 1, 0);
        pb_.speed = SPEED_PRESETS[speed_preset_idx_];
    }

    // ---------------------------------------------------------------------------
    // Orbital camera — compute mouse delta manually since VGL needs it explicitly
    // ---------------------------------------------------------------------------
    const glm::vec2 mouse_pos = gui_.getMousePosition();

    glm::vec2 mouse_delta{0.0f};
    if (!first_mouse_frame_) {
        // Only feed delta when a mouse button is held; OrbitalCamera routes
        // left-drag to orbit and right-drag to pan internally.
        const bool held = gui_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT)
                       || gui_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_RIGHT);
        if (held) mouse_delta = mouse_pos - prev_mouse_pos_;
    }
    first_mouse_frame_ = false;
    prev_mouse_pos_    = mouse_pos;

    orbital_cam_.handleInput(gui_, mouse_delta, gui_.getScrollDelta());
    orbital_cam_.applyToCamera(gui_.camera);
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------
void SatelliteRenderer::drawEarth(const glm::vec3& sun_dir) {
    gui_.setLightDirection(sun_dir);
    gui_.setLighting(true);
    gui_.drawSphere({0.0f, 0.0f, 0.0f}, EARTH_DISPLAY_R, {0.12f, 0.30f, 0.75f});

    // Equatorial ring (helps gauge inclination at a glance)
    gui_.setLighting(false);
    gui_.drawCircle({0.0f, 0.0f, 0.0f}, EARTH_DISPLAY_R * 1.002f,
                    glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f)),
                    {0.3f, 0.5f, 1.0f});

    // Prime meridian line  (ECEF X-axis direction — drifts in ECI with Earth rotation,
    // omitted here since we visualize in ECI; the ring is sufficient orientation cue)
    gui_.setLighting(true);
}

void SatelliteRenderer::drawSunIndicator(const glm::vec3& sun_dir) {
    gui_.setLighting(false);
    const float len = EARTH_DISPLAY_R * 2.2f;
    gui_.drawArrow({0.0f, 0.0f, 0.0f}, sun_dir * len, {1.0f, 0.95f, 0.3f}, 1.5f);
    gui_.setLighting(true);
}

void SatelliteRenderer::drawSatellites(int frame_idx) {
    if (frames_.empty() || frame_idx < 0) return;
    const auto& frame = frames_[frame_idx];
    const int n = static_cast<int>(frame.positions.size());

    gui_.setLighting(false);
    for (int i = 0; i < n; ++i) {
        const glm::vec3 pos = eciToScene(frame.positions[i]);
        const glm::vec3 col = frame.in_eclipse[i]
            ? glm::vec3{0.30f, 0.30f, 0.55f}   // dim in shadow
            : glm::vec3{1.00f, 0.95f, 0.65f};   // bright in sunlight
        gui_.drawSphere(pos, SAT_DOT_R, col);
    }
    gui_.setLighting(true);
}

void SatelliteRenderer::drawTrails(int frame_idx) {
    if (frames_.empty() || frame_idx <= 0) return;
    const int n_sats = static_cast<int>(frames_[frame_idx].positions.size());

    // Skip trails for large constellations — too expensive and too cluttered
    if (n_sats > TRAIL_MAX_SATS) return;

    const int start_frame = std::max(0, frame_idx - TRAIL_FRAMES);
    const int span        = frame_idx - start_frame;
    if (span == 0) return;

    gui_.setLighting(false);

    for (int s = 0; s < n_sats; ++s) {
        for (int f = start_frame + 1; f <= frame_idx; ++f) {
            const glm::vec3 a = eciToScene(frames_[f - 1].positions[s]);
            const glm::vec3 b = eciToScene(frames_[f    ].positions[s]);
            // Fade trail: newest segment = full brightness, oldest = dim
            const float t     = static_cast<float>(f - start_frame) / span;
            const glm::vec3 c = glm::mix(glm::vec3{0.05f, 0.2f, 0.05f},
                                          glm::vec3{0.2f,  0.8f, 0.3f},  t);
            gui_.drawLine(a, b, c, 1.0f);
        }
    }

    gui_.setLighting(true);
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void SatelliteRenderer::run() {
    if (frames_.empty()) {
        std::fprintf(stderr, "[Renderer] No frames captured — nothing to show.\n");
        return;
    }

    while (!gui_.shouldClose()) {
        gui_.beginFrame();

        advancePlayback();
        handleInput();
        updateWindowTitle();

        const auto& frame = frames_[pb_.frame_idx];
        const glm::vec3 sun_dir{
            static_cast<float>(frame.sun_dir_eci.x),
            static_cast<float>(frame.sun_dir_eci.y),
            static_cast<float>(frame.sun_dir_eci.z)
        };

        drawEarth(sun_dir);
        drawSunIndicator(sun_dir);
        drawTrails(pb_.frame_idx);
        drawSatellites(pb_.frame_idx);

        gui_.endFrame();
    }
}

#endif // CONSTELLATION_VIZ_ENABLED
