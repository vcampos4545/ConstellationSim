#include "gnc/AttitudeDynamics.h"

namespace {
// Propagation state: [qw, qx, qy, qz, ox, oy, oz, hwx, hwy, hwz]
struct AttPropState {
    double qw{1},qx{},qy{},qz{};
    double ox{},oy{},oz{};
    double hwx{},hwy{},hwz{};

    AttPropState operator+(const AttPropState& o) const {
        return {qw+o.qw,qx+o.qx,qy+o.qy,qz+o.qz,
                ox+o.ox,oy+o.oy,oz+o.oz,
                hwx+o.hwx,hwy+o.hwy,hwz+o.hwz};
    }
    AttPropState operator*(double s) const {
        return {qw*s,qx*s,qy*s,qz*s,ox*s,oy*s,oz*s,hwx*s,hwy*s,hwz*s};
    }
    friend AttPropState operator*(double s, const AttPropState& p) { return p*s; }
};

AttPropState derivative(const AttPropState& y,
                        const Vec3& inertia,
                        const Vec3& tau) {
    const Quat q{y.qw, y.qx, y.qy, y.qz};
    const Vec3 omega{y.ox, y.oy, y.oz};
    const Vec3 h_w{y.hwx, y.hwy, y.hwz};

    // Quaternion kinematics
    const Quat qdot = q.qdot(omega);

    // Euler's equation: I*omega_dot = tau - omega × (I*omega + h_wheels)
    const Vec3 Io   = {inertia.x*omega.x, inertia.y*omega.y, inertia.z*omega.z};
    const Vec3 gyro = omega.cross(Io + h_w);
    const Vec3 alpha = {
        (tau.x - gyro.x) / inertia.x,
        (tau.y - gyro.y) / inertia.y,
        (tau.z - gyro.z) / inertia.z
    };

    // Wheels absorb equal and opposite momentum
    const Vec3 h_dot = -tau;

    return {qdot.w, qdot.x, qdot.y, qdot.z,
            alpha.x, alpha.y, alpha.z,
            h_dot.x, h_dot.y, h_dot.z};
}
} // namespace

namespace AttitudeDynamics {

void step(AttitudeState& state,
          const Vec3&    inertia,
          const Vec3&    tau,
          double         dt) {
    AttPropState y0{
        state.attitude.w, state.attitude.x, state.attitude.y, state.attitude.z,
        state.omega.x, state.omega.y, state.omega.z,
        state.h_wheels.x, state.h_wheels.y, state.h_wheels.z
    };

    const auto k1 = derivative(y0,               inertia, tau);
    const auto k2 = derivative(y0 + 0.5*dt*k1,   inertia, tau);
    const auto k3 = derivative(y0 + 0.5*dt*k2,   inertia, tau);
    const auto k4 = derivative(y0 + dt*k3,        inertia, tau);

    const AttPropState y1 = y0 + (dt/6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);

    state.attitude = Quat{y1.qw, y1.qx, y1.qy, y1.qz}.normalized();
    state.omega    = {y1.ox, y1.oy, y1.oz};
    state.h_wheels = {y1.hwx, y1.hwy, y1.hwz};
}

} // namespace AttitudeDynamics
