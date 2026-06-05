#pragma once
#include "core/ConfigLoader.h"
#include "gnc/AttitudeState.h"
#include "gnc/Guidance.h"
#include "gnc/AttitudeController.h"
#include "gnc/WheelAllocator.h"
#include "gnc/MEKF.h"
#include "gnc/sensors/GPSModel.h"
#include "gnc/sensors/IMUModel.h"
#include "gnc/sensors/StarTrackerModel.h"
#include "orbit/OrbitState.h"

// Per-satellite flight software stack.
//
// All missions share this same class; configuration selects the mode and
// sensor parameters at runtime — no code changes needed between missions.
//
// Data flow each tick():
//   true state → sensors (noisy) → MEKF (estimated state)
//              → Guidance (target attitude)
//              → AttitudeController (torque command)
//              → WheelAllocator (per-wheel allocation)
//              → torque_cmd_ fed back into AttitudeDynamics by SimulationEngine
class FlightSoftwareStack {
public:
    FlightSoftwareStack(const FSWConfig& cfg,
                        const Vec3&      inertia_kgm2,
                        const OrbitState&  initial_orbit,
                        const AttitudeState& initial_att);

    // Call every physics step dt [s].
    // Internally rate-gates sensor sampling, MEKF propagation, and ADCS update.
    void tick(double             dt,
              const OrbitState&  true_orbit,
              const AttitudeState& true_att,
              const Vec3&        sun_dir_eci);

    // Net torque to apply to the satellite body this step [N·m, body frame].
    Vec3 torqueCommand() const { return torque_cmd_; }

    // MEKF estimated attitude (for telemetry / visualization).
    const AttitudeState& estimatedState() const { return estimated_att_; }

    ADCSMode mode() const { return mode_; }

    bool enabled() const { return cfg_.enabled; }

private:
    FSWConfig           cfg_;
    ADCSMode            mode_;
    Vec3                torque_cmd_{};
    AttitudeState       estimated_att_{};

    GPSModel            gps_;
    IMUModel            imu_;
    StarTrackerModel    star_tracker_;
    MEKF                mekf_;
    AttitudeController  adcs_;
    WheelAllocator      allocator_;

    // Rate timers (count up; fire when >= period)
    double gps_timer_{0};
    double imu_timer_{0};
    double adcs_timer_{0};
    double star_tracker_timer_{0};
};
