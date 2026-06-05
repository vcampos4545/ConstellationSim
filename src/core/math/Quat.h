#pragma once
#include "Vec3.h"
#include <cmath>

// Double-precision unit quaternion.
// Convention: q = w + xi + yj + zk  (scalar-first storage).
// Body-to-ECI: q rotates a vector from body frame to ECI frame.
//   v_eci = q * [0, v_body] * q*   (active rotation)
// Kinematics:  q_dot = 0.5 * q * pure(omega_body)
struct Quat {
    double w{1}, x{0}, y{0}, z{0};

    constexpr Quat() = default;
    constexpr Quat(double w, double x, double y, double z)
        : w(w), x(x), y(y), z(z) {}

    // Hamilton product (p then q applied in that order).
    Quat operator*(const Quat& q) const {
        return {
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        };
    }
    Quat operator*(double s) const { return {w*s, x*s, y*s, z*s}; }
    Quat operator+(const Quat& o) const { return {w+o.w, x+o.x, y+o.y, z+o.z}; }
    Quat operator-()              const { return {-w, -x, -y, -z}; }

    double normSq() const { return w*w + x*x + y*y + z*z; }
    double norm()   const { return std::sqrt(normSq()); }

    Quat normalized() const {
        const double n = norm();
        return (n > 1e-15) ? Quat{w/n, x/n, y/n, z/n} : Quat{};
    }

    Quat conjugate() const { return {w, -x, -y, -z}; }

    // Rotate vector: v_out = q * [0, v] * q*
    Vec3 rotate(const Vec3& v) const {
        const Quat qv{0, v.x, v.y, v.z};
        const Quat res = (*this) * qv * conjugate();
        return {res.x, res.y, res.z};
    }

    // Quaternion kinematics: q_dot = 0.5 * q * [0, omega_body]
    Quat qdot(const Vec3& omega_body) const {
        const Quat qw{0, omega_body.x, omega_body.y, omega_body.z};
        return (*this * qw) * 0.5;
    }

    // Convert rotation vector (axis*angle) to quaternion.
    static Quat fromRotVec(const Vec3& rv) {
        const double angle = rv.norm();
        if (angle < 1e-10) return Quat{1, rv.x*0.5, rv.y*0.5, rv.z*0.5};
        const Vec3  axis  = rv * (1.0 / angle);
        const double s    = std::sin(angle * 0.5);
        return {std::cos(angle * 0.5), axis.x*s, axis.y*s, axis.z*s};
    }

    // Convert to rotation vector (axis*angle).
    Vec3 toRotVec() const {
        const Quat qn = normalized();
        const double sin_half = Vec3{qn.x, qn.y, qn.z}.norm();
        if (sin_half < 1e-10) return {2.0*qn.x, 2.0*qn.y, 2.0*qn.z};
        const double angle = 2.0 * std::atan2(sin_half, qn.w);
        const Vec3   axis  = Vec3{qn.x, qn.y, qn.z} * (1.0 / sin_half);
        return axis * angle;
    }

    // Build body-to-ECI quaternion from orthonormal body axes expressed in ECI.
    // cols: body X, Y, Z axes in ECI frame.
    static Quat fromColAxes(const Vec3& cx, const Vec3& cy, const Vec3& cz);

    static Quat identity() { return {1, 0, 0, 0}; }
    static Quat pure(const Vec3& v) { return {0, v.x, v.y, v.z}; }
};

inline Quat operator*(double s, const Quat& q) { return q * s; }

// Shepperd's method: rotation matrix columns → quaternion.
// R[:,j] = body axis j expressed in ECI frame.
inline Quat Quat::fromColAxes(const Vec3& cx, const Vec3& cy, const Vec3& cz) {
    // R[row][col]: row = ECI component, col = body axis index
    const double R00 = cx.x, R10 = cx.y, R20 = cx.z;
    const double R01 = cy.x, R11 = cy.y, R21 = cy.z;
    const double R02 = cz.x, R12 = cz.y, R22 = cz.z;
    const double trace = R00 + R11 + R22;
    Quat q;
    if (trace > 0.0) {
        const double s = 0.5 / std::sqrt(trace + 1.0);
        q.w = 0.25 / s;
        q.x = (R12 - R21) * s;
        q.y = (R20 - R02) * s;
        q.z = (R01 - R10) * s;
    } else if (R00 > R11 && R00 > R22) {
        const double s = 2.0 * std::sqrt(1.0 + R00 - R11 - R22);
        q.w = (R12 - R21) / s;
        q.x = 0.25 * s;
        q.y = (R01 + R10) / s;
        q.z = (R20 + R02) / s;
    } else if (R11 > R22) {
        const double s = 2.0 * std::sqrt(1.0 + R11 - R00 - R22);
        q.w = (R20 - R02) / s;
        q.x = (R01 + R10) / s;
        q.y = 0.25 * s;
        q.z = (R12 + R21) / s;
    } else {
        const double s = 2.0 * std::sqrt(1.0 + R22 - R00 - R11);
        q.w = (R01 - R10) / s;
        q.x = (R20 + R02) / s;
        q.y = (R12 + R21) / s;
        q.z = 0.25 * s;
    }
    return q.normalized();
}
