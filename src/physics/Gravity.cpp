#include "physics/Gravity.h"

Vec3 Gravity::acceleration(const OrbitState& state,
                           const PhysicalProperties& /*props*/,
                           double /*time_s*/) const {
    const double r2 = state.position.normSq();
    const double r3 = r2 * std::sqrt(r2);
    return (-mu_ / r3) * state.position;
}
