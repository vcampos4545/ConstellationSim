#include "gnc/Guidance.h"

ADCSMode adcsModeFromString(const std::string& s) {
    if (s == "nadir" || s == "nadir_pointing") return ADCSMode::NADIR;
    if (s == "sun"   || s == "sun_pointing")   return ADCSMode::SUN_POINTING;
    if (s == "inertial" || s == "inertial_hold") return ADCSMode::INERTIAL_HOLD;
    return ADCSMode::OFF;
}

std::string adcsModeToString(ADCSMode mode) {
    switch (mode) {
        case ADCSMode::NADIR:          return "nadir";
        case ADCSMode::SUN_POINTING:   return "sun_pointing";
        case ADCSMode::INERTIAL_HOLD:  return "inertial_hold";
        default:                       return "off";
    }
}

// Build body-to-ECI quaternion from LVLH-style frame.
// body_z_eci: where body +Z should point in ECI.
// body_y_eci: where body +Y should point in ECI (orbit normal suggestion).
static Quat buildPointing(const Vec3& body_z_eci, const Vec3& body_y_hint_eci) {
    const Vec3 z = body_z_eci.normalized();
    Vec3 y = body_y_hint_eci.normalized();
    // Re-orthogonalize y against z
    y = (y - z * y.dot(z)).normalized();
    if (y.normSq() < 1e-10) {
        // Degenerate: pick arbitrary perpendicular
        Vec3 ref = (std::abs(z.x) < 0.9) ? Vec3{1,0,0} : Vec3{0,1,0};
        y = z.cross(ref).normalized();
    }
    const Vec3 x = y.cross(z).normalized();
    return Quat::fromColAxes(x, y, z);
}

namespace Guidance {

Quat targetAttitude(ADCSMode mode, const OrbitState& orbit,
                    const Vec3& sun_dir_eci, const Quat& target_inertial) {
    switch (mode) {
    case ADCSMode::NADIR: {
        // body +Z toward zenith (body -Z nadir = toward Earth center)
        // Standard LVLH: Z = radial out, Y = orbit normal, X = along-track
        const Vec3 z_eci = orbit.position.normalized();           // radial out
        const Vec3 y_eci = orbit.position.cross(orbit.velocity).normalized(); // orbit normal
        return buildPointing(z_eci, y_eci);
    }
    case ADCSMode::SUN_POINTING: {
        // body +X toward Sun; keep body Z approximately radial
        const Vec3 x_eci   = sun_dir_eci.normalized();
        const Vec3 z_hint  = orbit.position.normalized();
        // Build frame: x = sun, z = re-orthogonalized radial, y = z×x...
        // Reuse buildPointing with x and y swapped (body_z = sun, hint = radial)
        // But we want body +X toward sun, so swap axes:
        const Vec3 z_eci = (z_hint - x_eci * z_hint.dot(x_eci)).normalized();
        const Vec3 y_eci = z_eci.cross(x_eci).normalized();
        return Quat::fromColAxes(x_eci, y_eci, z_eci);
    }
    case ADCSMode::INERTIAL_HOLD:
        return target_inertial;
    default:
        return Quat::identity();
    }
}

} // namespace Guidance
