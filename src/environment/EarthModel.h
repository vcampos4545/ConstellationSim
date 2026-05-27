#pragma once
#include "core/math/Constants.h"
#include "core/math/Vec3.h"

// Utility functions for Earth geometry and coordinate transformations.
// Uses a spherical Earth with WGS84 constants for radius and rotation rate.
namespace EarthModel {

    // Convert geodetic latitude/longitude (degrees) + altitude (m) to ECEF [m].
    inline Vec3 geodeticToECEF(double lat_deg, double lon_deg, double alt_m = 0.0) {
        const double lat = lat_deg * Constants::DEG2RAD;
        const double lon = lon_deg * Constants::DEG2RAD;
        const double r   = Constants::EARTH_RADIUS_M + alt_m;
        return {
            r * std::cos(lat) * std::cos(lon),
            r * std::cos(lat) * std::sin(lon),
            r * std::sin(lat)
        };
    }

    // Rotate an ECEF vector to ECI by applying the Greenwich Sidereal Angle.
    // theta_gst is the Earth rotation angle [rad] at the current epoch.
    inline Vec3 ecefToECI(const Vec3& ecef, double theta_gst) {
        const double c = std::cos(theta_gst);
        const double s = std::sin(theta_gst);
        return {
             c*ecef.x - s*ecef.y,
             s*ecef.x + c*ecef.y,
             ecef.z
        };
    }

    // Earth rotation angle at time t [s] since simulation epoch.
    // theta_0 is the GST at epoch [rad].
    inline double gstAtTime(double t_s, double theta_0_rad = 0.0) {
        return theta_0_rad + Constants::EARTH_OMEGA_RAD_S * t_s;
    }

    // Atmospheric co-rotation velocity at position r [ECI, m]: v = omega x r
    inline Vec3 atmosphericVelocity(const Vec3& r) {
        return { -Constants::EARTH_OMEGA_RAD_S * r.y,
                  Constants::EARTH_OMEGA_RAD_S * r.x,
                  0.0 };
    }

    // Elevation angle [rad] of a satellite at r_sat_eci from a ground point at r_gnd_eci.
    // Positive = above horizon.
    inline double elevationAngle(const Vec3& r_gnd_eci, const Vec3& r_sat_eci) {
        const Vec3 slant = r_sat_eci - r_gnd_eci;
        const Vec3 gnd_hat = r_gnd_eci.normalized();
        return std::asin(std::clamp(gnd_hat.dot(slant.normalized()), -1.0, 1.0));
    }

} // namespace EarthModel
