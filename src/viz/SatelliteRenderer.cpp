#ifdef CONSTELLATION_VIZ_ENABLED

#include "viz/SatelliteRenderer.h"
#include "core/math/Constants.h"
#include "environment/EarthModel.h"
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <limits>

constexpr double SatelliteRenderer::SPEED_PRESETS[4];

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

SatelliteRenderer::SatelliteRenderer(std::shared_ptr<FrameQueue>                  queue,
                                     std::vector<GroundTarget>                    ground_targets,
                                     double                                       min_elevation_deg,
                                     std::vector<SimulationEngine::SatelliteInfo> sat_info,
                                     int window_w, int window_h)
    : queue_(std::move(queue)),
      sat_info_(std::move(sat_info)),
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

    // --- Dear ImGui init (after GL context is live via GUI constructor) ---
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    // Tune alpha so panels are readable but not opaque
    ImGui::GetStyle().Alpha = 0.92f;
    ImGui::GetStyle().WindowRounding = 6.0f;

    ImGui_ImplGlfw_InitForOpenGL(gui_.getWindow(), true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

SatelliteRenderer::~SatelliteRenderer()
{
    // Shut down ImGui before the GL context (owned by GUI) is destroyed.
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

// ---------------------------------------------------------------------------
// Coordinate conversion
// ---------------------------------------------------------------------------
glm::vec3 SatelliteRenderer::eciToScene(const Vec3& eci_m) const
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
            pb_.sim_time_s = std::fmod(pb_.sim_time_s, max_t);
            trail_buf_.clear();
            lo_frame_idx_ = -1;
        } else {
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

    interp_pos_.resize(n);
    interp_pos_eci_.resize(n);
    interp_vel_eci_.resize(n);

    for (int i = 0; i < n; ++i) {
        const glm::vec3 a = eciToScene(lo_frame.positions[i]);
        const glm::vec3 b = has_hi ? eciToScene(hi_frame.positions[i]) : a;
        interp_pos_[i] = glm::mix(a, b, alpha);

        // ECI position (unscaled) for telemetry
        const Vec3& p0 = lo_frame.positions[i];
        if (has_hi) {
            const Vec3& p1 = hi_frame.positions[i];
            interp_pos_eci_[i] = {
                p0.x + alpha * (p1.x - p0.x),
                p0.y + alpha * (p1.y - p0.y),
                p0.z + alpha * (p1.z - p0.z)};
        } else {
            interp_pos_eci_[i] = p0;
        }

        // ECI velocity (unscaled) for telemetry
        const bool has_vel = (i < (int)lo_frame.velocities.size());
        if (has_vel) {
            const Vec3& v0 = lo_frame.velocities[i];
            if (has_hi && i < (int)hi_frame.velocities.size()) {
                const Vec3& v1 = hi_frame.velocities[i];
                interp_vel_eci_[i] = {
                    v0.x + alpha * (v1.x - v0.x),
                    v0.y + alpha * (v1.y - v0.y),
                    v0.z + alpha * (v1.z - v0.z)};
            } else {
                interp_vel_eci_[i] = v0;
            }
        } else {
            interp_vel_eci_[i] = {};
        }
    }

    interp_ecl_ = lo_frame.in_eclipse;
    interp_sun_ = {
        static_cast<float>(lo_frame.sun_dir_eci.x),
        static_cast<float>(lo_frame.sun_dir_eci.y),
        static_cast<float>(lo_frame.sun_dir_eci.z)};

    const double gst = Constants::EARTH_OMEGA_RAD_S * pb_.sim_time_s;
    gt_scene_pos_.resize(ground_targets_.size());
    for (size_t i = 0; i < ground_targets_.size(); ++i) {
        const Vec3 eci = EarthModel::ecefToECI(ground_targets_[i].pos_ecef, gst);
        gt_scene_pos_[i] = eciToScene(eci);
    }
}

// ---------------------------------------------------------------------------
// Camera tracking
// ---------------------------------------------------------------------------
void SatelliteRenderer::applyTracking()
{
    if (selected_sat_idx_ < 0 || selected_sat_idx_ >= (int)interp_pos_.size()) return;
    orbital_cam_.setTarget(interp_pos_[selected_sat_idx_]);
    orbital_cam_.applyToCamera(gui_.camera);
}

void SatelliteRenderer::updateWindowTitle()
{
    const double t   = pb_.sim_time_s;
    const int days   = static_cast<int>(t / 86400.0);
    const int hrs    = static_cast<int>(std::fmod(t, 86400.0) / 3600.0);
    const int mins   = static_cast<int>(std::fmod(t, 3600.0) / 60.0);
    const int n_sats = static_cast<int>(interp_pos_.size());

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
// Satellite picking — ray-sphere intersection
// ---------------------------------------------------------------------------
int SatelliteRenderer::pickSatellite(glm::vec2 mouse_pos) const
{
    if (interp_pos_.empty()) return -1;

    const glm::vec3 ray_dir  = glm::normalize(gui_.getMouseRay(mouse_pos));
    const glm::vec3 ray_orig = gui_.camera.position;

    int   best_idx = -1;
    float best_t   = std::numeric_limits<float>::max();

    for (int i = 0; i < (int)interp_pos_.size(); ++i) {
        const glm::vec3 oc  = ray_orig - interp_pos_[i];
        const float     b   = glm::dot(oc, ray_dir);           // half-b form
        const float     c   = glm::dot(oc, oc) - SAT_PICK_R * SAT_PICK_R;
        const float     disc = b * b - c;
        if (disc < 0.0f) continue;
        const float t = -b - std::sqrt(disc);
        if (t > 0.0f && t < best_t) {
            best_t   = t;
            best_idx = i;
        }
    }
    return best_idx;
}

// ---------------------------------------------------------------------------
// Input handling
// ---------------------------------------------------------------------------
void SatelliteRenderer::handleInput()
{
    const bool imgui_wants_kb    = ImGui::GetIO().WantCaptureKeyboard;
    const bool imgui_wants_mouse = ImGui::GetIO().WantCaptureMouse;

    if (!imgui_wants_kb) {
        if (gui_.isKeyJustPressed(GLFW_KEY_SPACE))
            pb_.paused = !pb_.paused;

        if (gui_.isKeyJustPressed(GLFW_KEY_ESCAPE))
            selected_sat_idx_ = -1;

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
    }

    const glm::vec2 mouse_pos       = gui_.getMousePosition();
    const bool left_just_pressed    = gui_.isMouseButtonJustPressed(GLFW_MOUSE_BUTTON_LEFT);
    const bool left_just_released   = gui_.isMouseButtonJustReleased(GLFW_MOUSE_BUTTON_LEFT);
    const bool left_held            = gui_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT);
    const bool right_held           = gui_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_RIGHT);
    const bool just_pressed_any     = left_just_pressed
                                   || gui_.isMouseButtonJustPressed(GLFW_MOUSE_BUTTON_RIGHT);

    // --- Click detection: track drag distance from press point ---
    if (left_just_pressed && !imgui_wants_mouse) {
        mouse_at_press_ = mouse_pos;
        drag_started_   = false;
    }
    if (left_held && !drag_started_) {
        if (glm::length(mouse_pos - mouse_at_press_) > CLICK_DRAG_THRESHOLD)
            drag_started_ = true;
    }

    // On clean click-release, pick a satellite (or deselect on miss)
    if (left_just_released && !drag_started_ && !imgui_wants_mouse)
        selected_sat_idx_ = pickSatellite(mouse_pos);

    if (imgui_wants_mouse) {
        prev_mouse_pos_ = mouse_pos;
        return;
    }

    glm::vec2 mouse_delta{0.0f};
    if ((left_held || right_held) && !just_pressed_any)
        mouse_delta = mouse_pos - prev_mouse_pos_;
    prev_mouse_pos_ = mouse_pos;

    orbital_cam_.handleInput(gui_, mouse_delta, gui_.getScrollDelta());
    orbital_cam_.applyToCamera(gui_.camera);
}

// ---------------------------------------------------------------------------
// Drawing helpers
// ---------------------------------------------------------------------------
void SatelliteRenderer::drawStarBackground()
{
    gui_.drawBackground(star_tex_);
}

void SatelliteRenderer::drawEarth()
{
    const float sidereal_angle = static_cast<float>(EARTH_ROT_RATE * pb_.sim_time_s);
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
        const bool in_ecl = (i < (int)interp_ecl_.size() && interp_ecl_[i]);

        if (i == selected_sat_idx_) {
            // Selection halo + bright dot
            gui_.drawSphere(interp_pos_[i], SAT_DOT_R * 2.0f, {0.0f, 0.85f, 1.0f});
            gui_.drawSphere(interp_pos_[i], SAT_DOT_R,         {1.0f, 1.0f,  1.0f});
        } else {
            const glm::vec3 col = in_ecl
                ? glm::vec3{0.30f, 0.30f, 0.55f}
                : glm::vec3{1.00f, 0.95f, 0.65f};
            gui_.drawSphere(interp_pos_[i], SAT_DOT_R, col);
        }
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

        // Highlight the selected satellite's trail
        const bool is_selected = (s == selected_sat_idx_);

        for (int f = 1; f < sz; ++f) {
            const float t = static_cast<float>(f) / sz;
            glm::vec3 c;
            if (is_selected)
                c = glm::mix(glm::vec3{0.0f, 0.3f, 0.5f}, glm::vec3{0.0f, 0.85f, 1.0f}, t);
            else
                c = glm::mix(glm::vec3{0.05f, 0.2f, 0.05f}, glm::vec3{0.2f, 0.8f, 0.3f}, t);
            gui_.drawLine(buf[f - 1], buf[f], c, is_selected ? 1.5f : 1.0f);
        }

        if (s < (int)interp_pos_.size())
            gui_.drawLine(buf.back(), interp_pos_[s],
                          is_selected ? glm::vec3{0.0f, 0.85f, 1.0f} : glm::vec3{0.2f, 0.8f, 0.3f},
                          is_selected ? 1.5f : 1.0f);
    }
    gui_.setLighting(true);
}

void SatelliteRenderer::drawGroundTargets()
{
    if (gt_scene_pos_.empty()) return;
    gui_.setLighting(false);
    for (const auto& pos : gt_scene_pos_)
        gui_.drawSphere(pos, GT_MARKER_R, {1.0f, 0.55f, 0.05f});
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
            if (s < (int)interp_ecl_.size() && interp_ecl_[s]) continue;

            const glm::vec3 slant    = interp_pos_[s] - gnd;
            const float     sin_elev = glm::dot(gnd_hat, glm::normalize(slant));
            if (sin_elev >= min_elev_sin_) {
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
// ImGui overlays
// ---------------------------------------------------------------------------

void SatelliteRenderer::drawHUD()
{
    const double t   = pb_.sim_time_s;
    const int days   = static_cast<int>(t / 86400.0);
    const int hrs    = static_cast<int>(std::fmod(t, 86400.0) / 3600.0);
    const int mins   = static_cast<int>(std::fmod(t, 3600.0) / 60.0);
    const int secs   = static_cast<int>(std::fmod(t, 60.0));
    const int n_sats = static_cast<int>(interp_pos_.size());

    ImGui::SetNextWindowPos({4.0f, 4.0f});
    ImGui::SetNextWindowBgAlpha(0.55f);
    ImGui::Begin("##hud", nullptr,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove  |
        ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);

    // Mission time
    ImGui::TextColored({0.7f, 0.9f, 1.0f, 1.0f}, "T+");
    ImGui::SameLine(0, 4);
    ImGui::Text("%02dd %02dh %02dm %02ds", days, hrs, mins, secs);

    ImGui::SameLine(0, 16);
    ImGui::TextColored({0.9f, 0.9f, 0.5f, 1.0f}, "%.0f\xc3\x97", pb_.speed);  // "×" utf-8

    ImGui::SameLine(0, 8);
    if (pb_.paused)
        ImGui::TextColored({1.0f, 0.8f, 0.1f, 1.0f}, "PAUSED");
    else
        ImGui::TextColored({0.5f, 0.5f, 0.5f, 1.0f}, "RUNNING");

    if (!queue_->isSimDone()) {
        ImGui::SameLine(0, 8);
        ImGui::TextColored({0.3f, 1.0f, 0.4f, 1.0f}, "[SIMULATING...]");
    }

    ImGui::SameLine(0, 16);
    ImGui::TextColored({0.75f, 0.75f, 0.75f, 1.0f}, "%d sats", n_sats);

    if (selected_sat_idx_ < 0 && !interp_pos_.empty()) {
        ImGui::SameLine(0, 16);
        ImGui::TextDisabled("Click a satellite to select");
    }

    ImGui::End();
}

void SatelliteRenderer::drawTelemetryPanel()
{
    if (selected_sat_idx_ < 0 || selected_sat_idx_ >= (int)interp_pos_.size()) return;

    const int idx = selected_sat_idx_;

    // --- Compute orbital quantities ---
    const Vec3& pos = interp_pos_eci_[idx];   // ECI [m]
    const Vec3& vel = interp_vel_eci_[idx];   // ECI [m/s]

    const double r = pos.norm();
    const double alt_km = (r - Constants::EARTH_RADIUS_M) / 1000.0;

    // ECI → ECEF (rotate about Z by -GST)
    const double gst = Constants::EARTH_OMEGA_RAD_S * pb_.sim_time_s;
    const double cx = std::cos(gst), sx = std::sin(gst);
    const double ecef_x =  pos.x * cx + pos.y * sx;
    const double ecef_y = -pos.x * sx + pos.y * cx;
    const double ecef_z =  pos.z;

    const double lat_deg = std::atan2(ecef_z, std::sqrt(ecef_x * ecef_x + ecef_y * ecef_y))
                           * Constants::RAD2DEG;
    const double lon_deg = std::atan2(ecef_y, ecef_x) * Constants::RAD2DEG;

    const double speed_kms = vel.norm() / 1000.0;

    // Circular-orbit period estimate
    const double period_min = (Constants::TWO_PI * std::sqrt(r * r * r / Constants::GM_EARTH)) / 60.0;

    // Inclination from angular momentum vector h = r × v
    const Vec3 h = pos.cross(vel);
    const double h_norm = h.norm();
    const double inc_deg = (h_norm > 0.0)
        ? std::acos(std::clamp(h.z / h_norm, -1.0, 1.0)) * Constants::RAD2DEG
        : 0.0;

    const bool in_ecl = (idx < (int)interp_ecl_.size()) && interp_ecl_[idx];

    // --- Layout ---
    ImGui::SetNextWindowPos({8.0f, 38.0f}, ImGuiCond_Always);
    ImGui::SetNextWindowSize({300.0f, 0.0f}, ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.88f);
    ImGui::Begin("Telemetry", nullptr,
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoCollapse);

    // --- Header ---
    if (idx < (int)sat_info_.size()) {
        const auto& info = sat_info_[idx];
        ImGui::TextColored({0.0f, 0.85f, 1.0f, 1.0f}, "SAT-%03d", info.id);
        ImGui::SameLine(0, 12);
        ImGui::TextDisabled("Plane %d  Seat %d", info.plane_id, info.seat_id);
    } else {
        ImGui::TextColored({0.0f, 0.85f, 1.0f, 1.0f}, "SAT-%03d", idx);
    }
    ImGui::Separator();

    // --- Position ---
    ImGui::TextColored({0.8f, 0.8f, 0.5f, 1.0f}, "POSITION");
    ImGui::Columns(2, "pos_cols", false);
    ImGui::SetColumnWidth(0, 110.0f);

    ImGui::Text("Altitude");  ImGui::NextColumn();
    ImGui::Text("%.1f km", alt_km);  ImGui::NextColumn();

    ImGui::Text("Latitude");  ImGui::NextColumn();
    ImGui::Text("%+.2f\xc2\xb0", lat_deg);  ImGui::NextColumn();  // °

    ImGui::Text("Longitude"); ImGui::NextColumn();
    ImGui::Text("%+.2f\xc2\xb0", lon_deg);  ImGui::NextColumn();

    ImGui::Columns(1);
    ImGui::Separator();

    // --- Velocity / Orbit ---
    ImGui::TextColored({0.8f, 0.8f, 0.5f, 1.0f}, "ORBIT");
    ImGui::Columns(2, "orb_cols", false);
    ImGui::SetColumnWidth(0, 110.0f);

    ImGui::Text("Speed");       ImGui::NextColumn();
    ImGui::Text("%.3f km/s", speed_kms);  ImGui::NextColumn();

    ImGui::Text("Period");      ImGui::NextColumn();
    ImGui::Text("%.1f min", period_min);  ImGui::NextColumn();

    ImGui::Text("Inclination"); ImGui::NextColumn();
    ImGui::Text("%.2f\xc2\xb0", inc_deg);  ImGui::NextColumn();

    ImGui::Columns(1);
    ImGui::Separator();

    // --- Status ---
    ImGui::TextColored({0.8f, 0.8f, 0.5f, 1.0f}, "STATUS");
    if (in_ecl) {
        ImGui::TextColored({0.4f, 0.4f, 0.9f, 1.0f}, "\xe2\x97\x8f Eclipse (shadow)");
    } else {
        ImGui::TextColored({1.0f, 0.95f, 0.4f, 1.0f}, "\xe2\x98\x80 Sunlit");
    }
    ImGui::Separator();

    // --- Deselect ---
    if (ImGui::Button("Deselect  [Esc]", {-1.0f, 0.0f}))
        selected_sat_idx_ = -1;

    ImGui::End();
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void SatelliteRenderer::run()
{
    while (!gui_.shouldClose())
    {
        // --- ImGui new frame (before gui_.beginFrame() so camera tracking
        //     happens before shader matrices are uploaded) ---
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        advancePlayback();
        handleInput();
        buildInterpState();

        // Apply satellite tracking BEFORE beginFrame so the shader gets the
        // updated camera matrices.
        applyTracking();

        // --- Scene render ---
        gui_.beginFrame();

        drawStarBackground();
        drawEarth();
        drawGroundTargets();

        if (!interp_pos_.empty()) {
            drawSunIndicator();
            drawTrails();
            drawSatellites();
            drawGroundLinks();
        }

        // --- ImGui overlays (rendered on top of the 3D scene) ---
        drawHUD();
        drawTelemetryPanel();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        updateWindowTitle();
        gui_.endFrame();
    }
}

#endif // CONSTELLATION_VIZ_ENABLED
