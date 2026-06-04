#include "core/SimulationEngine.h"
#include "physics/Gravity.h"
#include "physics/J2Perturbation.h"
#include "physics/AtmosphericDrag.h"
#include "physics/SolarRadiationPressure.h"
#include "environment/EclipseModel.h"
#include "environment/SunModel.h"
#include "core/math/Constants.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <mutex>

// Serialize all simulation progress output so multi-threaded MC runs
// don't interleave partial lines on the terminal.
static std::mutex g_console_mutex;

SimulationEngine::SimulationEngine(const SimConfig& cfg)
    : cfg_(cfg),
      constellation_(Constellation::createWalker(cfg.constellation)),
      metrics_(cfg.metrics, cfg)
{
    buildPropagator();

    // Apply the satellite physical properties from config to all satellites.
    for (auto* sat : constellation_.satellites()) {
        sat->state();  // ensure state is initialized
        // Set properties (Constellation::createWalker uses defaults; override here)
        const_cast<PhysicalProperties&>(sat->properties()) = cfg.satellite;
    }
}

void SimulationEngine::buildPropagator() {
    const auto& phy = cfg_.physics;
    const double mu  = Constants::GM_EARTH;
    const double re  = Constants::EARTH_RADIUS_M;
    const double j2  = Constants::J2;

    if (phy.gravity) {
        auto g = std::make_unique<Gravity>(mu);
        propagator_.addForceModel(std::move(g));
    }
    if (phy.j2) {
        auto j = std::make_unique<J2Perturbation>(mu, j2, re);
        propagator_.addForceModel(std::move(j));
    }
    if (phy.drag) {
        auto d = std::make_unique<AtmosphericDrag>(re);
        propagator_.addForceModel(std::move(d));
    }
    if (phy.srp) {
        auto s = std::make_unique<SolarRadiationPressure>(cfg_.epoch_jd, re);
        propagator_.addForceModel(std::move(s));
    }
}

void SimulationEngine::broadcastFrame(double time_s) {
    if (!frame_cb_) return;

    FrameData frame;
    frame.time_s     = time_s;
    frame.sun_dir_eci = SunModel::direction_eci(time_s, cfg_.epoch_jd);

    const auto& sats = constellation_.satellites();
    frame.positions.reserve(sats.size());
    frame.in_eclipse.reserve(sats.size());

    for (const auto* sat : sats) {
        frame.positions.push_back(sat->state().position);
        frame.in_eclipse.push_back(
            EclipseModel::inEclipse(sat->state().position, frame.sun_dir_eci));
    }

    (*frame_cb_)(frame);
}

ConstellationResult SimulationEngine::run(int run_id) {
    const double dt     = cfg_.timestep_s;
    const double t_end  = cfg_.duration_s();
    auto& sats          = constellation_.satellites();

    // (Progress reporting removed to keep MC output clean)
    [[maybe_unused]] const double report_interval = t_end / 10.0;
    [[maybe_unused]] double next_report = report_interval;

    {
        std::lock_guard lock(g_console_mutex);
        std::cout << "[" << cfg_.run_name << "] propagating "
                  << constellation_.totalSatellites() << " satellites for "
                  << cfg_.duration_days << " days\n";
    }

    metrics_.update(sats, 0.0);
    broadcastFrame(0.0);

    for (double t = 0.0; t < t_end; t += dt) {
        for (auto* sat : sats) {
            propagator_.step(sat->state(), sat->properties(), dt);
        }
        metrics_.update(sats, t + dt);
        broadcastFrame(t + dt);

        if (t + dt >= next_report) {
            next_report += report_interval;
        }
    }

    {
        std::lock_guard lock(g_console_mutex);
        std::cout << "[" << cfg_.run_name << "] done\n";
    }
    return metrics_.finalize(run_id, cfg_);
}

const std::vector<SatelliteResult>& SimulationEngine::satelliteResults() const {
    return metrics_.satelliteResults();
}

const std::vector<GroundTargetResult>& SimulationEngine::groundTargetResults() const {
    return metrics_.groundTargetResults();
}

std::pair<ConstellationResult, std::vector<SimulationEngine::FrameData>>
SimulationEngine::runAndCapture(int run_id) {
    std::vector<FrameData> frames;
    // Reserve capacity: duration / timestep + small headroom
    frames.reserve(static_cast<std::size_t>(cfg_.duration_s() / cfg_.timestep_s) + 2);

    setFrameCallback([&frames](const FrameData& f) { frames.push_back(f); });
    ConstellationResult cr = run(run_id);
    frame_cb_.reset();  // detach callback so the engine can be reused

    return {std::move(cr), std::move(frames)};
}
