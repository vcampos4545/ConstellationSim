#include "gnc/FlightSoftwareStack.h"
#include "environment/SunModel.h"

static WheelAllocator buildAllocator(const WheelSuiteConfig& w) {
    WheelAllocator::WheelConfig wc;
    wc.max_torque_Nm    = w.max_torque_Nm;
    wc.max_momentum_Nms = w.max_momentum_Nms;
    wc.spin_axes        = w.spin_axes;
    if (wc.spin_axes.empty())
        wc.spin_axes = {{1,0,0}, {0,1,0}, {0,0,1}};  // default 3-axis aligned
    return WheelAllocator(wc);
}

FlightSoftwareStack::FlightSoftwareStack(const FSWConfig&       cfg,
                                          const Vec3&            inertia_kgm2,
                                          const OrbitState&      initial_orbit,
                                          const AttitudeState&   initial_att)
    : cfg_(cfg),
      mode_(adcsModeFromString(cfg.adcs_mode)),
      gps_(cfg.sensors.gps),
      imu_(cfg.sensors.imu),
      star_tracker_(cfg.sensors.star_tracker),
      allocator_(buildAllocator(cfg.wheels))
{
    adcs_.initialize(inertia_kgm2);

    // Prime MEKF with the initial (true) attitude so it converges fast.
    if (cfg.enabled) {
        mekf_.initialize(initial_att.attitude);
        estimated_att_ = initial_att;
    }
    (void)initial_orbit;  // reserved for future OD filter initialization
}

void FlightSoftwareStack::tick(double             dt,
                                const OrbitState&  true_orbit,
                                const AttitudeState& true_att,
                                const Vec3&        sun_dir_eci) {
    if (!cfg_.enabled) return;

    // --- IMU: sample at imu_update_hz ---
    const double imu_period = 1.0 / cfg_.sensors.imu.update_hz;
    imu_timer_ += dt;
    if (imu_timer_ >= imu_period) {
        const auto& meas = imu_.sample(true_att.omega, imu_timer_);
        mekf_.propagate(meas.gyro_rad_s, imu_timer_);
        imu_timer_ = 0.0;
    }

    // --- Star Tracker: sample at star_tracker_update_hz ---
    if (cfg_.sensors.star_tracker.enabled) {
        const double st_period = 1.0 / cfg_.sensors.star_tracker.update_hz;
        star_tracker_timer_ += dt;
        if (star_tracker_timer_ >= st_period) {
            const auto& meas = star_tracker_.sample(true_att.attitude);
            if (meas.valid) mekf_.updateStarTracker(meas.attitude);
            star_tracker_timer_ = 0.0;
        }
    } else {
        // Without star tracker, use sun + nadir TRIAD at GPS rate.
        // GPS triggers TRIAD update.
    }

    // --- GPS: sample at gps_update_hz, trigger TRIAD if no star tracker ---
    const double gps_period = 1.0 / cfg_.sensors.gps.update_hz;
    gps_timer_ += dt;
    if (gps_timer_ >= gps_period) {
        gps_.sample(true_orbit);

        if (!cfg_.sensors.star_tracker.enabled) {
            // TRIAD: reference vectors = sun and nadir in ECI
            const Vec3 nadir_eci = -true_orbit.position.normalized();
            const Vec3 sun_eci   = sun_dir_eci.normalized();

            // Body measurements: rotate ECI vectors into body frame using true attitude
            const Vec3 nadir_body = true_att.attitude.conjugate().rotate(nadir_eci);
            const Vec3 sun_body   = true_att.attitude.conjugate().rotate(sun_eci);

            mekf_.update(sun_eci, nadir_eci, sun_body, nadir_body);
        }
        gps_timer_ = 0.0;
    }

    // --- ADCS: run at adcs_update_hz ---
    const double adcs_period = 1.0 / cfg_.adcs_update_hz;
    adcs_timer_ += dt;
    if (adcs_timer_ >= adcs_period) {
        // Update estimated attitude state from MEKF
        if (mekf_.isInitialized()) {
            estimated_att_.attitude = mekf_.getAttitude();
            estimated_att_.omega    = mekf_.getAngularVelocity();
        }
        estimated_att_.h_wheels = true_att.h_wheels;  // wheels are directly sensed

        // Guidance: compute target attitude
        const Quat target = Guidance::targetAttitude(mode_, true_orbit, sun_dir_eci);

        // Attitude controller → desired body torque
        const Vec3 tau_cmd = adcs_.computeTorque(
            estimated_att_.attitude, target, estimated_att_.omega, adcs_period);

        // Wheel allocator → per-wheel torques → recomposed body torque
        // (clamped to max_torque_Nm per wheel)
        const auto wheel_torques = allocator_.allocate(tau_cmd);
        torque_cmd_ = allocator_.recompose(wheel_torques);

        adcs_timer_ = 0.0;
    }
}
