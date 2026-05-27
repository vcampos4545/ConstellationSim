#pragma once
#include "orbit/Satellite.h"
#include <vector>

// A single orbital plane in a Walker or custom constellation.
// Owns the Satellite objects for all seats in the plane.
class Plane {
public:
    Plane(int plane_id, double inclination_rad, double raan_rad)
        : plane_id_(plane_id), inclination_rad_(inclination_rad), raan_rad_(raan_rad) {}

    void addSatellite(Satellite sat) { satellites_.push_back(std::move(sat)); }

    int    planeId()       const { return plane_id_; }
    double inclinationRad() const { return inclination_rad_; }
    double raanRad()        const { return raan_rad_; }

    std::vector<Satellite>&       satellites()       { return satellites_; }
    const std::vector<Satellite>& satellites() const { return satellites_; }

private:
    int    plane_id_;
    double inclination_rad_;
    double raan_rad_;
    std::vector<Satellite> satellites_;
};
