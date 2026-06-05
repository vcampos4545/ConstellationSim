#include "orbit/Satellite.h"
#include "gnc/FlightSoftwareStack.h"
#include "gnc/Guidance.h"
#include "core/math/Constants.h"
#include <cmath>

Satellite::Satellite(int id, int plane_id, int seat_id,
                     const OrbitState&       initial_state,
                     const PhysicalProperties& props,
                     const FSWConfig&        fsw_cfg)
    : id_(id), plane_id_(plane_id), seat_id_(seat_id),
      state_(initial_state), props_(props)
{
    // Initialize attitude to nadir-pointing LVLH at t=0.
    att_state_.attitude = Guidance::targetAttitude(
        ADCSMode::NADIR, initial_state, Vec3{1,0,0});
    att_state_.omega    = {};
    att_state_.h_wheels = {};

    if (fsw_cfg.enabled) {
        fsw_ = std::make_unique<FlightSoftwareStack>(
            fsw_cfg, props_.inertia_kgm2, initial_state, att_state_);
    }
}

// Destructor and move ops defined here so unique_ptr<FlightSoftwareStack> sees the full type.
Satellite::~Satellite()            = default;
Satellite::Satellite(Satellite&&)  = default;
Satellite& Satellite::operator=(Satellite&&) = default;

void Satellite::initFSW(const FSWConfig& cfg, const Vec3& inertia_kgm2) {
    fsw_.reset();
    if (cfg.enabled) {
        fsw_ = std::make_unique<FlightSoftwareStack>(
            cfg, inertia_kgm2, state_, att_state_);
    }
}

void Satellite::fswTick(double dt, const Vec3& sun_dir_eci) {
    if (fsw_) fsw_->tick(dt, state_, att_state_, sun_dir_eci);
}

Vec3 Satellite::fswTorqueCommand() const {
    return fsw_ ? fsw_->torqueCommand() : Vec3{};
}

double Satellite::altitude_m(double earth_radius_m) const {
    return state_.position.norm() - earth_radius_m;
}
