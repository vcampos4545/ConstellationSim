#pragma once

#include "core/SimulationEngine.h"
#include <mutex>
#include <optional>

// Lock-based state buffer between the simulation thread and the render thread.
// The simulation pushes frames; the renderer polls for the latest frame.
// The renderer never blocks the simulation: if it can't acquire the lock it
// simply skips the current frame.
class StateBuffer {
public:
    void push(const SimulationEngine::FrameData& frame) {
        std::lock_guard lock(mutex_);
        latest_ = frame;
    }

    // Returns the latest frame if one is available, without blocking.
    std::optional<SimulationEngine::FrameData> tryGet() {
        std::unique_lock lock(mutex_, std::try_to_lock);
        if (!lock || !latest_) return std::nullopt;
        return latest_;
    }

private:
    std::mutex mutex_;
    std::optional<SimulationEngine::FrameData> latest_;
};
