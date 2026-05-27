#pragma once

#include "orbit/OrbitState.h"
#include "core/ConfigLoader.h"
#include "physics/ForceModel.h"
#include <vector>
#include <memory>

// RK4 numerical propagator.
// Integrates a 6-element state vector [x, y, z, vx, vy, vz] through time.
//
// Force models are added once at startup and evaluated at each RK4 stage.
// Disabled models are skipped, so toggling models mid-run is supported
// (though not thread-safe without external synchronization).
class Propagator {
public:
    void addForceModel(std::unique_ptr<ForceModel> model);

    // Advance state by dt seconds. t is elapsed simulation time [s].
    // Returns the total acceleration magnitude at the start of the step [m/s^2].
    double step(OrbitState& state, const PhysicalProperties& props, double dt) const;

    const std::vector<std::unique_ptr<ForceModel>>& forceModels() const { return forces_; }

private:
    std::vector<std::unique_ptr<ForceModel>> forces_;

    // Derivative of state at (state, t): returns [v, a] packed as PropState
    PropState derivative(const PropState& y,
                         const PhysicalProperties& props,
                         double t) const;

    Vec3 totalAcceleration(const OrbitState& state,
                           const PhysicalProperties& props,
                           double t) const;
};
