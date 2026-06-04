#pragma once

#include "core/ConfigLoader.h"
#include "orbit/Propagator.h"
#include "constellation/Constellation.h"
#include "metrics/MetricsCollector.h"
#include <functional>
#include <optional>

// Single-simulation execution engine.
//
// The engine owns:
//   - A Propagator (shared force models applied to all satellites)
//   - A Constellation (satellite state)
//   - A MetricsCollector (incremental metrics accumulation)
//
// Visualization is decoupled via an optional frame callback that the
// renderer subscribes to. If no callback is registered the engine runs
// fully headless with no synchronization overhead.
class SimulationEngine {
public:
    // Per-frame state snapshot pushed to the renderer when viz is enabled.
    struct FrameData {
        double              time_s;
        std::vector<Vec3>   positions;       // ECI [m]
        std::vector<Vec3>   velocities;      // ECI [m/s]
        std::vector<bool>   in_eclipse;
        Vec3                sun_dir_eci;
    };

    // Static per-satellite metadata (same every frame; populated in constructor).
    struct SatelliteInfo {
        int id{0};
        int plane_id{0};
        int seat_id{0};
    };

    using FrameCallback = std::function<void(const FrameData&)>;

    explicit SimulationEngine(const SimConfig& cfg);

    // Register a viz callback (called at each timestep when visualization is on).
    void setFrameCallback(FrameCallback cb) { frame_cb_ = std::move(cb); }

    // Execute the full simulation. Returns the constellation-level result.
    ConstellationResult run(int run_id = 0);

    // Run headless and capture every frame into a vector for visualization playback.
    // Returns the ConstellationResult and all captured FrameData objects.
    std::pair<ConstellationResult, std::vector<FrameData>>
    runAndCapture(int run_id = 0);

    // Access results after run().
    const std::vector<SatelliteResult>&    satelliteResults()    const;
    const std::vector<GroundTargetResult>& groundTargetResults() const;
    const std::vector<SatelliteInfo>&      satelliteInfo()       const;

private:
    SimConfig        cfg_;
    Propagator       propagator_;
    Constellation    constellation_;
    MetricsCollector metrics_;

    std::optional<FrameCallback> frame_cb_;
    std::vector<SatelliteInfo>   sat_info_;   // populated in constructor

    void buildPropagator();
    void broadcastFrame(double time_s);
};
