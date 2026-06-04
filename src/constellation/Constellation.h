#pragma once

#include "constellation/Plane.h"
#include "core/ConfigLoader.h"
#include <vector>
#include <span>

// A Walker or custom satellite constellation.
//
// Walker T/P/F notation:
//   T = total satellites
//   P = number of planes
//   F = phasing factor (relative phase offset between adjacent planes)
//
// RAAN spacing:   dΩ = 360/P  [deg]
// In-plane spacing: dν = 360/S  [deg], S = T/P
// Cross-plane phase: Δν = F * 360/T  [deg]
class Constellation {
public:
    // Generate a Walker Delta constellation from config.
    static Constellation createWalker(const WalkerConfig& cfg, int start_id = 0);

    // Build a fully custom constellation from explicit per-satellite Keplerian specs.
    static Constellation createCustom(const std::vector<SatelliteSpec>& specs);

    const std::vector<Plane>&     planes()     const { return planes_; }
    const std::vector<Satellite*>& satellites() const { return sat_ptrs_; }

    int totalSatellites() const { return static_cast<int>(sat_ptrs_.size()); }

    // Rebuild the flat satellite pointer cache (call after adding satellites).
    void rebuildSatPtrs();

private:
    std::vector<Plane>      planes_;
    std::vector<Satellite*> sat_ptrs_;  // non-owning, points into planes_
};
