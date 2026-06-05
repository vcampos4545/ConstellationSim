#pragma once

#include "orbit/OrbitState.h"
#include "gnc/AttitudeState.h"
#include "core/ConfigLoader.h"
#include <string>
#include <memory>

// Forward declaration so the header doesn't pull in the entire GNC stack.
class FlightSoftwareStack;

struct SatelliteMetrics {
    double time_in_sunlight_s    = 0.0;
    double time_in_eclipse_s     = 0.0;
    double accumulated_drag_dv_ms = 0.0;
    double accumulated_sk_dv_ms  = 0.0;
    double total_time_s          = 0.0;

    double sunlit_fraction()  const { return (total_time_s > 0) ? time_in_sunlight_s / total_time_s : 0.0; }
    double eclipse_fraction() const { return (total_time_s > 0) ? time_in_eclipse_s  / total_time_s : 0.0; }
};

class Satellite {
public:
    Satellite(int id, int plane_id, int seat_id,
              const OrbitState&      initial_state,
              const PhysicalProperties& props,
              const FSWConfig&       fsw_cfg);

    // Destructor and move ops defined in .cpp (unique_ptr<FlightSoftwareStack> needs full type).
    ~Satellite();
    Satellite(Satellite&&);
    Satellite& operator=(Satellite&&);

    int id()       const { return id_; }
    int planeId()  const { return plane_id_; }
    int seatId()   const { return seat_id_; }

    OrbitState&               state()       { return state_; }
    const OrbitState&         state() const { return state_; }
    const PhysicalProperties& properties()  const { return props_; }
    SatelliteMetrics&         metrics()     { return metrics_; }
    const SatelliteMetrics&   metrics() const { return metrics_; }

    AttitudeState&            attitudeState()       { return att_state_; }
    const AttitudeState&      attitudeState() const { return att_state_; }

    // (Re)initialize the FSW stack with the given config and inertia.
    // Call this after applying PhysicalProperties to ensure ADCS gains are correct.
    void initFSW(const FSWConfig& cfg, const Vec3& inertia_kgm2);

    // FSW interface — delegates to internal stack; no-op if FSW disabled.
    void fswTick(double dt, const Vec3& sun_dir_eci);
    Vec3 fswTorqueCommand() const;
    bool hasFSW() const { return fsw_ != nullptr; }

    double altitude_m(double earth_radius_m) const;

private:
    int                id_;
    int                plane_id_;
    int                seat_id_;
    OrbitState         state_;
    AttitudeState      att_state_;
    PhysicalProperties props_;
    SatelliteMetrics   metrics_;
    std::unique_ptr<FlightSoftwareStack> fsw_;
};
