#pragma once
#include "orbit/OrbitState.h"

// Classical Keplerian orbital elements.
// Angles stored in radians; distances in meters.
struct OrbitalElements {
    double sma{0.0};   // semi-major axis [m]
    double ecc{0.0};   // eccentricity [-]
    double inc{0.0};   // inclination [rad]
    double raan{0.0};  // right ascension of ascending node [rad]
    double aop{0.0};   // argument of perigee [rad]
    double ta{0.0};    // true anomaly [rad]

    // Convert Keplerian elements to ECI Cartesian state.
    // mu = gravitational parameter [m^3/s^2]
    OrbitState toStateVector(double mu) const;

    // Construct elements from an ECI state vector.
    static OrbitalElements fromStateVector(const OrbitState& state, double mu);

    // Solve Kepler's equation iteratively for mean anomaly -> eccentric anomaly.
    static double solveKepler(double mean_anomaly, double eccentricity);

    // Mean anomaly to true anomaly.
    static double meanToTrue(double mean_anomaly, double eccentricity);

    double altitude_m(double earth_radius_m) const { return sma - earth_radius_m; }
    double period_s(double mu) const;
    double meanMotion_rads(double mu) const;
};
