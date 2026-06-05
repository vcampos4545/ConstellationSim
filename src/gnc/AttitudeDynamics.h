#pragma once
#include "gnc/AttitudeState.h"

// RK4 integrator for rotational dynamics.
//
// Equations of motion (body frame):
//   q_dot   = 0.5 * q * [0, omega_body]          (quaternion kinematics)
//   I*omega_dot = tau_control                      (Euler's equation, simplified)
//               - omega × (I*omega + h_wheels)
//   h_wheels_dot = -tau_control                   (wheel absorbs reaction momentum)
//
// inertia_kgm2: diagonal of the inertia tensor [Ixx, Iyy, Izz] in body frame.
// tau_control:  net torque applied to the body by actuators [N·m], body frame.
namespace AttitudeDynamics {
    void step(AttitudeState& state,
              const Vec3&    inertia_kgm2,
              const Vec3&    tau_control,
              double         dt);
}
