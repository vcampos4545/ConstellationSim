#ifdef CONSTELLATION_VIZ_ENABLED

#include "viz/SatelliteRenderer.h"
#include "core/math/Constants.h"
#include "environment/EarthModel.h"
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <cstdio>
#include <algorithm>

constexpr double SatelliteRenderer::SPEED_PRESETS[4];

SatelliteRenderer::SatelliteRenderer(std::shared_ptr<FrameQueue> queue,
                                     std::vector<GroundTarget>   ground_targets,
                                     double                      min_elevation_deg,
                                     int window_w, int window_h)
    : queue_(std::move(queue)),
      gui_(window_w, window_h, "ConstellationSim"),
      orbital_cam_(3.5f, 25.0f, -20.0f, glm::vec3(0.0f)),
      min_elev_sin_(static_cast<float>(std::sin(min_elevation_deg * Constants::DEG2RAD)))
{
    gui_.camera
        .setFOV(45.0f)
        .setClipPlanes(0.001f, 500.0f)
        .setUp({0.0f, 0.0f, 1.0f});

    gui_.setLogDepth(500.0f);
    gui_.setLighting(true);

    orbital_cam_
        .setMinDistance(1.05f)
        .setMaxDistance(20.0f)
        .setZoomSensitivity(0.3f)
        .setPanSensitivity(0.25f);

    earth_tex_.loadFromFile("resources/textures/earth.jpg");
    star_tex_.loadFromFile("resources/textures/stars.jpg");

    for (const auto& gt : ground_targets) {
        GroundTargetViz v;
        v.name     = gt.name;
        v.pos_ecef = EarthModel::geodeticToECEF(gt.lat_deg, gt.lon_deg, 0.0);
        ground_targets_.push_back(std::move(v));
    }

    pb_.wall_prev = glfwGetTime();
}

// ---------------------------------------------------------------------------
// Coordinate conversion
// ---------------------------------------------------------------------------
glm::vec3 SatelliteRenderer::eciToScene(const Vec3 &eci_m) const
{
    return {
        static_cast<float>(eci_m.x) * SCENE_SCALE,
        static_cast<float>(eci_m.y) * SCENE_SCALE,
        static_cast<float>(eci_m.z) * SCENE_SCALE};
}

// ---------------------------------------------------------------------------
// Playback control
// ---------------------------------------------------------------------------
void SatelliteRenderer::advancePlayback()
{
    const double wall_now = glfwGetTime();
    const double real_dt  = wall_now - pb_.wall_prev;
    pb_.wall_prev = wall_now;

    if (pb_.paused) return;

    pb_.sim_time_s += real_dt * pb_.speed;

    const double max_t = queue_->maxSimTime();
    if (max_t <= 0.0) {
        pb_.sim_time_s = 0.0;
        return;
    }

    if (pb_.sim_time_s > max_t) {
        if (queue_->isSimDone()) {
            // Loop once the full simulation is captured
            pb_.sim_time_s = std::fmod(pb_.sim_time_s, max_t);
            // Looping resets the trail buffer so old trails don't persist
            trail_buf_.clear();
            lo_frame_idx_ = -1;
        } else {
            // Sim still running — clamp to available data
            pb_.sim_time_s = max_t;
        }
    }
}

// ---------------------------------------------------------------------------
// Interpolated state
// ---------------------------------------------------------------------------
void SatelliteRenderer::buildInterpState()
{
    const int new_lo = queue_->findIdx(pb_.sim_time_s);
    if (new_lo < 0) return;

    SimulationEngine::FrameData lo_frame, hi_frame;
    const bool has_hi = queue_->copyFramePair(new_lo, new_lo + 1, lo_frame, hi_frame);

    float alpha = 0.0f;
    if (has_hi) {
        const double dt = hi_frame.time_s - lo_frame.time_s;
        if (dt > 0.0)
            alpha = std::clamp(
                static_cast<float>((pb_.sim_time_s - lo_frame.time_s) / dt),
                0.0f, 1.0f);
    }

    const int n = static_cast<int>(lo_frame.positions.size());

    // Update per-satellite trail buffer when base frame advances
    if (new_lo != lo_frame_idx_) {
        if (static_cast<int>(trail_buf_.size()) != n)
            trail_buf_.assign(n, std::deque<glm::vec3>{});

        for (int i = 0; i < n; ++i) {
            trail_buf_[i].push_back(eciToScene(lo_frame.positions[i]));
            while (static_cast<int>(trail_buf_[i].size()) > TRAIL_FRAMES + 1)
                trail_buf_[i].pop_front();
        }
        lo_frame_idx_ = new_lo;
    }

    // Linearly interpolate positions between lo and hi frames
    interp_pos_.resize(n);
    for (int i = 0; i < n; ++i) {
        const glm::vec3 a = eciToScene(lo_frame.positions[i]);
        const glm::vec3 b = has_hi ? eciToScene(hi_frame.positions[i]) : a;
        interp_pos_[i] = glm::mix(a, b, alpha);
    }

    interp_ecl_ = lo_frame.in_eclipse;
    interp_sun_ = {
        static_cast<float>(lo_frame.sun_dir_eci.x),
        static_cast<float>(lo_frame.sun_dir_eci.y),
        static_cast<float>(lo_frame.sun_dir_eci.z)};

    // Rotate ground targets from ECEF to ECI for current sim time
    const double gst = Constants::EARTH_OMEGA_RAD_S * pb_.sim_time_s;
    gt_scene_pos_.resize(ground_targets_.size());
    for (size_t i = 0; i < ground_targets_.size(); ++i) {
        const Vec3 eci = EarthModel::ecefToECI(ground_targets_[i].pos_ecef, gst);
        gt_scene_pos_[i] = eciToScene(eci);
    }
}

void SatelliteRenderer::updateWindowTitle()
{
    const double t    = pb_.sim_time_s;
    const int days    = static_cast<int>(t / 86400.0);
    const int hrs     = static_cast<int>(std::fmod(t, 86400.0) / 3600.0);
    const int mins    = static_cast<int>(std::fmod(t, 3600.0) / 60.0);
    const int n_sats  = static_cast<int>(interp_pos_.size());

    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "ConstellationSim | T+%dd %02dh %02dm | %d sats | Speed: %.0f×%s%s",
        days, hrs, mins, n_sats,
        pb_.speed,
        pb_.paused ? " [PAUSED]" : "",
        !queue_->isSimDone() ? " [SIMULATING...]" : "");

    glfwSetWindowTitle(gui_.getWindow(), buf);
}

// ---------------------------------------------------------------------------
// Input handling
// ---------------------------------------------------------------------------
void SatelliteRenderer::handleInput()
{
    if (gui_.isKeyJustPressed(GLFW_KEY_SPACE))
        pb_.paused = !pb_.paused;

    const int preset_keys[4] = {GLFW_KEY_1, GLFW_KEY_2, GLFW_KEY_3, GLFW_KEY_4};
    for (int i = 0; i < 4; ++i) {
        if (gui_.isKeyJustPressed(preset_keys[i])) {
            speed_preset_idx_ = i;
            pb_.speed = SPEED_PRESETS[i];
        }
    }

    const bool inc = gui_.isKeyJustPressed(GLFW_KEY_EQUAL) || gui_.isKeyJustPressed(GLFW_KEY_KP_ADD);
    const bool dec = gui_.isKeyJustPressed(GLFW_KEY_MINUS) || gui_.isKeyJustPressed(GLFW_KEY_KP_SUBTRACT);
    if (inc) { speed_preset_idx_ = std::min(speed_preset_idx_ + 1, 3); pb_.speed = SPEED_PRESETS[speed_preset_idx_]; }
    if (dec) { speed_preset_idx_ = std::max(speed_preset_idx_ - 1, 0); pb_.speed = SPEED_PRESETS[speed_preset_idx_]; }

    const glm::vec2 mouse_pos    = gui_.getMousePosition();
    const bool left_held         = gui_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT);
    const bool right_held        = gui_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_RIGHT);
    const bool just_pressed      = gui_.isMouseButtonJustPressed(GLFW_MOUSE_BUTTON_LEFT)
                                || gui_.isMouseButtonJustPressed(GLFW_MOUSE_BUTTON_RIGHT);

    glm::vec2 mouse_delta{0.0f};
    if ((left_held || right_held) && !just_pressed)
        mouse_delta = mouse_pos - prev_mouse_pos_;
    prev_mouse_pos_ = mouse_pos;

    orbital_cam_.handleInput(gui_, mouse_delta, gui_.getScrollDelta());
    orbital_cam_.applyToCamera(gui_.camera);
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------
void SatelliteRenderer::drawStarBackground()
{
    gui_.drawBackground(star_tex_);
}

void SatelliteRenderer::drawEarth()
{
    const float sidereal_angle = static_cast<float>(EARTH_ROT_RATE * pb_.sim_time_s);

    // The VGL sphere mesh has Y as its north pole, but ECI uses Z.
    // Rotate -90° around X so the mesh north (Y) maps to world north (Z),
    // then apply the sidereal rotation around ECI Z.
    const auto pole_fix   = glm::angleAxis(-glm::half_pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));
    const auto earth_spin = glm::angleAxis(sidereal_angle, glm::vec3(0.0f, 0.0f, 1.0f));
    const auto rot        = earth_spin * pole_fix;

    gui_.setLightDirection(interp_sun_);
    gui_.drawTexturedSphere({0.0f, 0.0f, 0.0f}, EARTH_DISPLAY_R, rot, earth_tex_);

    gui_.setLighting(false);
    gui_.drawCircle({0.0f, 0.0f, 0.0f}, EARTH_DISPLAY_R * 1.002f,
                    glm::quat(glm::vec3(0.0f, 0.0f, 0.0f)),
                    {0.3f, 0.5f, 1.0f});
    gui_.setLighting(true);
}

void SatelliteRenderer::drawSunIndicator()
{
    gui_.setLighting(false);
    const float len = EARTH_DISPLAY_R * 2.2f;
    gui_.drawArrow({0.0f, 0.0f, 0.0f}, interp_sun_ * len, {1.0f, 0.95f, 0.3f}, 1.5f);
    gui_.setLighting(true);
}

void SatelliteRenderer::drawSatellites()
{
    if (interp_pos_.empty()) return;
    const int n = static_cast<int>(interp_pos_.size());

    gui_.setLighting(false);
    for (int i = 0; i < n; ++i) {
        const glm::vec3 col = (i < static_cast<int>(interp_ecl_.size()) && interp_ecl_[i])
            ? glm::vec3{0.30f, 0.30f, 0.55f}
            : glm::vec3{1.00f, 0.95f, 0.65f};
        gui_.drawSphere(interp_pos_[i], SAT_DOT_R, col);
    }
    gui_.setLighting(true);
}

void SatelliteRenderer::drawTrails()
{
    const int n = static_cast<int>(trail_buf_.size());
    if (n == 0 || n > TRAIL_MAX_SATS) return;

    gui_.setLighting(false);
    for (int s = 0; s < n; ++s) {
        const auto& buf = trail_buf_[s];
        const int sz = static_cast<int>(buf.size());
        if (sz == 0) continue;

        // Draw stored trail segments with fade from dim to bright
        for (int f = 1; f < sz; ++f) {
            const float t = static_cast<float>(f) / sz;
            const glm::vec3 c = glm::mix(glm::vec3{0.05f, 0.2f, 0.05f},
                                         glm::vec3{0.2f,  0.8f, 0.3f}, t);
            gui_.drawLine(buf[f - 1], buf[f], c, 1.0f);
        }

        // Final segment: last stored point → current interpolated position
        if (s < static_cast<int>(interp_pos_.size()))
            gui_.drawLine(buf.back(), interp_pos_[s], {0.2f, 0.8f, 0.3f}, 1.0f);
    }
    gui_.setLighting(true);
}

void SatelliteRenderer::drawGroundTargets()
{
    if (gt_scene_pos_.empty()) return;
    gui_.setLighting(false);
    for (const auto& pos : gt_scene_pos_) {
        gui_.drawSphere(pos, GT_MARKER_R, {1.0f, 0.55f, 0.05f});  // orange dot
    }
    gui_.setLighting(true);
}

void SatelliteRenderer::drawGroundLinks()
{
    if (gt_scene_pos_.empty() || interp_pos_.empty()) return;
    const int n_sats    = static_cast<int>(interp_pos_.size());
    const int n_targets = static_cast<int>(gt_scene_pos_.size());

    gui_.setLighting(false);
    for (int t = 0; t < n_targets; ++t) {
        const glm::vec3& gnd     = gt_scene_pos_[t];
        const glm::vec3  gnd_hat = glm::normalize(gnd);

        for (int s = 0; s < n_sats; ++s) {
            // Only draw link when satellite is in sunlight (can reflect)
            if (s < static_cast<int>(interp_ecl_.size()) && interp_ecl_[s]) continue;

            const glm::vec3 slant    = interp_pos_[s] - gnd;
            const float     sin_elev = glm::dot(gnd_hat, glm::normalize(slant));
            if (sin_elev >= min_elev_sin_) {
                // Fade color by elevation: brighter = higher elevation
                const float brightness = 0.4f + 0.6f * (sin_elev - min_elev_sin_)
                                                       / (1.0f - min_elev_sin_);
                gui_.drawLine(gnd, interp_pos_[s],
                              {brightness, brightness * 0.85f, 0.1f}, 0.8f);
            }
        }
    }
    gui_.setLighting(true);
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void SatelliteRenderer::run()
{
    while (!gui_.shouldClose())
    {
        gui_.beginFrame();

        advancePlayback();
        handleInput();
        buildInterpState();
        updateWindowTitle();

        drawStarBackground();
        drawEarth();
        drawGroundTargets();

        if (!interp_pos_.empty()) {
            drawSunIndicator();
            drawTrails();
            drawSatellites();
            drawGroundLinks();
        }

        gui_.endFrame();
    }
}

#endif // CONSTELLATION_VIZ_ENABLED
