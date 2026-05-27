#include "environment/MoonModel.h"
#include "core/math/Constants.h"
#include <cmath>

// Simplified Moon position (Meeus, "Astronomical Algorithms", Ch 47 — reduced terms).
Vec3 MoonModel::position_eci(double jd) {
    const double T = (jd - Constants::J2000_JD) / 36525.0;

    // Fundamental arguments (degrees)
    const double L0 = std::fmod(218.3164477 + 481267.88123421*T, 360.0);
    const double M_sun  = std::fmod(357.5291092 + 35999.0502909*T, 360.0);
    const double M_moon = std::fmod(134.9633964 + 477198.8675055*T, 360.0);
    const double D = std::fmod(297.8501921 + 445267.1114034*T, 360.0);
    const double F = std::fmod(93.2720950  + 483202.0175233*T, 360.0);

    const double L0r = L0 * Constants::DEG2RAD;
    const double Mr  = M_moon * Constants::DEG2RAD;
    const double Dr  = D * Constants::DEG2RAD;
    const double Fr  = F * Constants::DEG2RAD;
    const double Ms  = M_sun * Constants::DEG2RAD;

    // Longitude (degrees) — dominant terms only
    double lon = L0
        + 6.288774 * std::sin(Mr)
        + 1.274027 * std::sin(2.0*Dr - Mr)
        + 0.658314 * std::sin(2.0*Dr)
        + 0.213618 * std::sin(2.0*Mr)
        - 0.185116 * std::sin(Ms)
        - 0.114332 * std::sin(2.0*Fr);

    // Latitude (degrees)
    double lat = 5.128122 * std::sin(Fr)
               + 0.280602 * std::sin(Mr + Fr)
               + 0.277693 * std::sin(Mr - Fr)
               + 0.173237 * std::sin(2.0*Dr - Fr);

    // Distance (km)
    double dist_km = 385000.56
        + (-20905.355) * std::cos(Mr)
        + (-3699.111)  * std::cos(2.0*Dr - Mr)
        + (-2955.968)  * std::cos(2.0*Dr);

    const double lon_r = lon * Constants::DEG2RAD;
    const double lat_r = lat * Constants::DEG2RAD;
    const double dist_m = dist_km * 1000.0;

    // Convert ecliptic to ECI (J2000 obliquity ~23.439°)
    const double eps = 23.439291111 * Constants::DEG2RAD;
    return {
        dist_m * std::cos(lat_r) * std::cos(lon_r),
        dist_m * (std::cos(eps)*std::cos(lat_r)*std::sin(lon_r) - std::sin(eps)*std::sin(lat_r)),
        dist_m * (std::sin(eps)*std::cos(lat_r)*std::sin(lon_r) + std::cos(eps)*std::sin(lat_r))
    };
}

Vec3 MoonModel::direction_eci(double t_s, double epoch_jd) {
    return position_eci(epoch_jd + t_s / Constants::SEC_PER_DAY).normalized();
}
