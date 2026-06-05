#include "gnc/AttitudeController.h"
#include <cmath>
#include <algorithm>

void AttitudeController::initialize(const Vec3& inertia_kgm2,
                                     double settling_time_s,
                                     double damping) {
    // omega_n = 4 / (zeta * ts),  Kp = I*wn^2,  Kd = 2*zeta*I*wn
    const double wn = 4.0 / (damping * settling_time_s);
    Kp_ = {inertia_kgm2.x * wn*wn,
           inertia_kgm2.y * wn*wn,
           inertia_kgm2.z * wn*wn};
    Kd_ = {2.0 * damping * inertia_kgm2.x * wn,
           2.0 * damping * inertia_kgm2.y * wn,
           2.0 * damping * inertia_kgm2.z * wn};
    Ki_ = {Kp_.x * 0.01, Kp_.y * 0.01, Kp_.z * 0.01};
    reset();
    initialized_ = true;
}

Vec3 AttitudeController::computeTorque(const Quat& current_att,
                                        const Quat& target_att,
                                        const Vec3& omega_body,
                                        double      dt) {
    if (!initialized_) return {};

    // Error quaternion in body frame: q_err = q_current^-1 * q_target
    // This expresses "how much rotation is needed, in body frame coords"
    Quat q_err = current_att.conjugate() * target_att;
    if (q_err.w < 0.0) q_err = -q_err;   // take short path

    // Attitude error as rotation vector in body frame
    const Vec3 e_att = q_err.toRotVec();

    // Integral update with anti-windup clamp
    integral_.x = std::clamp(integral_.x + e_att.x * dt, -integral_max_, integral_max_);
    integral_.y = std::clamp(integral_.y + e_att.y * dt, -integral_max_, integral_max_);
    integral_.z = std::clamp(integral_.z + e_att.z * dt, -integral_max_, integral_max_);

    // PID torque: drive error to zero, damp angular rate
    return {
        Kp_.x * e_att.x + Ki_.x * integral_.x - Kd_.x * omega_body.x,
        Kp_.y * e_att.y + Ki_.y * integral_.y - Kd_.y * omega_body.y,
        Kp_.z * e_att.z + Ki_.z * integral_.z - Kd_.z * omega_body.z
    };
}

void AttitudeController::reset() {
    integral_ = {};
}
