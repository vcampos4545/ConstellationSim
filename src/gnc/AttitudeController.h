#pragma once
#include "core/math/Quat.h"
#include "core/math/Vec3.h"

// PD attitude controller with integral anti-windup.
//
// Control law (body frame):
//   tau = Kp * e_att + Ki * integral(e_att) + Kd * (0 - omega)
//
// Where e_att is the attitude error expressed as a rotation vector (axis*angle)
// in the body frame, and omega is the body angular rate.
//
// Gains are auto-tuned from the diagonal inertia tensor and desired
// second-order response parameters using the pole-placement formula:
//   omega_n = 4 / (zeta * t_settle)
//   Kp = I * omega_n^2
//   Kd = 2 * zeta * I * omega_n
class AttitudeController {
public:
    // settling_time_s: 2% settling time [s]
    // damping:         damping ratio (0.7–0.9 typical)
    void initialize(const Vec3& inertia_kgm2,
                    double settling_time_s = 60.0,
                    double damping        = 0.8);

    Vec3 computeTorque(const Quat& current_attitude,
                       const Quat& target_attitude,
                       const Vec3& omega_body,
                       double      dt);

    void reset();

    bool isInitialized() const { return initialized_; }

private:
    Vec3 Kp_{}, Ki_{}, Kd_{};
    Vec3 integral_{};
    double integral_max_{0.5};
    bool   initialized_{false};
};
