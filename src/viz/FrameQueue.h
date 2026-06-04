#pragma once

#ifdef CONSTELLATION_VIZ_ENABLED

#include "core/SimulationEngine.h"
#include <vector>
#include <mutex>
#include <atomic>

// Thread-safe, append-only store of simulation frames.
//
// Producer (sim thread): push() each FrameData as it is computed, then markSimDone().
// Consumer (render thread): findIdx() + copyFramePair() to get the two frames
// bracketing the current playback time for interpolation.
class FrameQueue {
public:
    using FrameData = SimulationEngine::FrameData;

    // Hard cap to bound memory usage (50 k frames ≈ 20–200 MB depending on constellation size).
    static constexpr size_t MAX_FRAMES = 50'000;

    void push(FrameData f) {
        std::lock_guard<std::mutex> lk(mutex_);
        if (frames_.size() < MAX_FRAMES)
            frames_.push_back(std::move(f));
    }

    void markSimDone() {
        sim_done_.store(true, std::memory_order_release);
    }

    bool isSimDone() const {
        return sim_done_.load(std::memory_order_acquire);
    }

    // Highest time_s currently in the queue; 0.0 if empty.
    double maxSimTime() const {
        std::lock_guard<std::mutex> lk(mutex_);
        return frames_.empty() ? 0.0 : frames_.back().time_s;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lk(mutex_);
        return frames_.size();
    }

    // Binary search: returns largest index i where frames_[i].time_s <= sim_t.
    // Returns -1 if no such frame exists (queue empty or all frames are after sim_t).
    int findIdx(double sim_t) const {
        std::lock_guard<std::mutex> lk(mutex_);
        if (frames_.empty() || frames_.front().time_s > sim_t) return -1;
        int lo = 0, hi = static_cast<int>(frames_.size()) - 1;
        while (lo < hi) {
            const int mid = (lo + hi + 1) / 2;
            if (frames_[mid].time_s <= sim_t) lo = mid;
            else                               hi = mid - 1;
        }
        return lo;
    }

    // Copy a single frame by index. Returns false if out of range.
    bool copyFrame(int idx, FrameData& out) const {
        std::lock_guard<std::mutex> lk(mutex_);
        if (idx < 0 || idx >= static_cast<int>(frames_.size())) return false;
        out = frames_[idx];
        return true;
    }

    // Copy two adjacent frames for interpolation. Returns false if either index is out of range.
    bool copyFramePair(int lo, int hi, FrameData& lo_out, FrameData& hi_out) const {
        std::lock_guard<std::mutex> lk(mutex_);
        const int n = static_cast<int>(frames_.size());
        if (lo < 0 || hi >= n) return false;
        lo_out = frames_[lo];
        hi_out = frames_[hi];
        return true;
    }

private:
    mutable std::mutex      mutex_;
    std::vector<FrameData>  frames_;
    std::atomic<bool>       sim_done_{false};
};

#endif // CONSTELLATION_VIZ_ENABLED
