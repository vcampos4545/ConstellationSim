#pragma once
#include "core/math/Vec3.h"
#include <vector>
#include <array>

// Distributes a 3-axis body torque command across N reaction wheels.
//
// Each wheel has a spin axis (unit vector in body frame).
// The torque applied to the body by wheel i is: axis_i * tau_wheel_i.
// Stacking all wheels: tau_body = W * tau_wheels, where W (3×N) has columns = wheel axes.
//
// For N=3 orthogonal wheels: W = I, tau_wheels = tau_body (trivial).
// For N>3 wheels: pseudoinverse W^+ = W^T * (W*W^T)^-1 gives minimum-norm solution.
class WheelAllocator {
public:
    struct WheelConfig {
        std::vector<Vec3> spin_axes;    // unit spin axis per wheel in body frame
        double max_torque_Nm  = 0.2;   // per-wheel saturation
        double max_momentum_Nms = 4.0; // per-wheel momentum limit
    };

    // Default: 3 orthogonal wheels aligned with body axes.
    WheelAllocator();
    explicit WheelAllocator(const WheelConfig& cfg);

    // Compute per-wheel torque commands from desired body torque.
    // Returns vector of length N (one entry per wheel).
    std::vector<double> allocate(const Vec3& tau_body) const;

    // Recompose net body torque from per-wheel torques (for verification).
    Vec3 recompose(const std::vector<double>& wheel_torques) const;

    int numWheels() const { return static_cast<int>(axes_.size()); }

private:
    std::vector<Vec3>              axes_;   // wheel spin axes (N)
    std::vector<std::array<double,3>> pinv_; // pseudoinverse rows (N×3)
    double max_torque_Nm_;

    void buildPseudoinverse();
};
