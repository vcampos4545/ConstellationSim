#pragma once

#include "core/math/Vec3.h"
#include "orbit/OrbitState.h"
#include "core/ConfigLoader.h"
#include <string>

// Abstract base for all perturbation / force models.
// Each model receives the current ECI state, satellite physical properties,
// and elapsed simulation time (seconds since epoch).
//
// Returning Vec3{} when disabled avoids virtual dispatch overhead
// in the common disabled case by short-circuiting at the call site.
class ForceModel {
public:
    virtual ~ForceModel() = default;

    // Compute gravitational / perturbation acceleration [m/s^2] in ECI.
    virtual Vec3 acceleration(const OrbitState& state,
                              const PhysicalProperties& props,
                              double time_s) const = 0;

    virtual std::string name() const = 0;

    bool isEnabled() const       { return enabled_; }
    void setEnabled(bool enable) { enabled_ = enable; }

protected:
    bool enabled_{true};
};
