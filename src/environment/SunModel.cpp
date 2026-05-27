#include "environment/SunModel.h"
#include "core/math/Constants.h"
#include <cmath>

// Low-precision solar position (Vallado, "Fundamentals of Astrodynamics", Alg 29).
// Accuracy: ~0.01 deg in ecliptic longitude over a few centuries.
Vec3 SunModel::position_eci(double jd) {
    // Julian centuries since J2000
    const double T = (jd - Constants::J2000_JD) / 36525.0;

    // Mean longitude and anomaly (degrees)
    double L0 = std::fmod(280.46646 + 36000.76983 * T, 360.0);
    double M  = std::fmod(357.52911 + 35999.05029 * T - 0.0001537 * T*T, 360.0);
    const double M_rad = M * Constants::DEG2RAD;

    // Equation of center (degrees)
    const double C = (1.914602 - 0.004817*T - 0.000014*T*T) * std::sin(M_rad)
                   + (0.019993 - 0.000101*T) * std::sin(2.0*M_rad)
                   + 0.000289 * std::sin(3.0*M_rad);

    // Sun's true longitude and anomaly (degrees)
    const double sun_lon = L0 + C;
    const double f = M + C;  // true anomaly

    // Sun-Earth distance (AU)
    const double R_AU = 1.000001018 * (1.0 - 0.01671123*std::cos(M_rad) - 0.00014*std::cos(2.0*M_rad));

    // Apparent longitude (aberration correction, degrees)
    const double omega = 125.04 - 1934.136*T;
    const double lambda = sun_lon - 0.00569 - 0.00478*std::sin(omega * Constants::DEG2RAD);

    // Obliquity of ecliptic (degrees)
    const double eps0 = 23.439291111 - 0.013004167*T - 0.0000001639*T*T + 0.0000005036*T*T*T;
    const double eps  = eps0 + 0.00256 * std::cos(omega * Constants::DEG2RAD);
    const double eps_rad = eps * Constants::DEG2RAD;
    const double lam_rad = lambda * Constants::DEG2RAD;

    // ECI unit vector
    const Vec3 r_hat{
        std::cos(lam_rad),
        std::cos(eps_rad) * std::sin(lam_rad),
        std::sin(eps_rad) * std::sin(lam_rad)
    };

    return r_hat * (R_AU * Constants::AU_M);
}

Vec3 SunModel::direction_eci(double t_s, double epoch_jd) {
    const double jd = epoch_jd + t_s / Constants::SEC_PER_DAY;
    return position_eci(jd).normalized();
}
