#include "orbit/Propagator.h"

void Propagator::addForceModel(std::unique_ptr<ForceModel> model) {
    forces_.push_back(std::move(model));
}

Vec3 Propagator::totalAcceleration(const OrbitState& state,
                                    const PhysicalProperties& props,
                                    double t) const {
    Vec3 accel;
    for (const auto& force : forces_) {
        if (force->isEnabled()) {
            accel += force->acceleration(state, props, t);
        }
    }
    return accel;
}

PropState Propagator::derivative(const PropState& y,
                                  const PhysicalProperties& props,
                                  double t) const {
    const OrbitState state{{y.x, y.y, y.z}, {y.vx, y.vy, y.vz}, t};
    const Vec3 a = totalAcceleration(state, props, t);
    return {y.vx, y.vy, y.vz, a.x, a.y, a.z};
}

double Propagator::step(OrbitState& state, const PhysicalProperties& props, double dt) const {
    const PropState y0{state.position.x, state.position.y, state.position.z,
                       state.velocity.x, state.velocity.y, state.velocity.z};
    const double t = state.time_s;

    // Classic RK4
    const PropState k1 = derivative(y0,                  props, t);
    const PropState k2 = derivative(y0 + 0.5*dt*k1,      props, t + 0.5*dt);
    const PropState k3 = derivative(y0 + 0.5*dt*k2,      props, t + 0.5*dt);
    const PropState k4 = derivative(y0 + dt*k3,           props, t + dt);

    const PropState y1 = y0 + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);

    state.position = {y1.x,  y1.y,  y1.z};
    state.velocity = {y1.vx, y1.vy, y1.vz};
    state.time_s   = t + dt;

    // Return acceleration magnitude from k1 (start of step) for metrics
    const Vec3 a0{k1.vx, k1.vy, k1.vz};
    return a0.norm();
}
