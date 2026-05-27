#pragma once

#include <cmath>
#include <array>

// Double-precision 3D vector used throughout the simulation.
// All orbital mechanics calculations use doubles for numerical stability.
struct Vec3 {
    double x{0.0}, y{0.0}, z{0.0};

    constexpr Vec3() = default;
    constexpr Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s)       const { return {x * s,   y * s,   z * s};   }
    Vec3 operator/(double s)       const { return {x / s,   y / s,   z / s};   }
    Vec3 operator-()               const { return {-x, -y, -z}; }

    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x-=o.x; y-=o.y; z-=o.z; return *this; }
    Vec3& operator*=(double s)      { x*=s;   y*=s;   z*=s;   return *this; }

    double dot(const Vec3& o)  const { return x*o.x + y*o.y + z*o.z; }
    Vec3   cross(const Vec3& o) const {
        return {y*o.z - z*o.y,
                z*o.x - x*o.z,
                x*o.y - y*o.x};
    }

    double normSq() const { return x*x + y*y + z*z; }
    double norm()   const { return std::sqrt(normSq()); }

    Vec3 normalized() const {
        const double n = norm();
        return (n > 0.0) ? (*this / n) : Vec3{};
    }

    bool isZero() const { return normSq() == 0.0; }
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

// Propagation state vector: [x, y, z, vx, vy, vz] — used by RK4.
// Inline arithmetic avoids temporary allocations during integration.
struct PropState {
    double x{}, y{}, z{}, vx{}, vy{}, vz{};

    PropState operator+(const PropState& o) const {
        return {x+o.x, y+o.y, z+o.z, vx+o.vx, vy+o.vy, vz+o.vz};
    }
    PropState operator*(double s) const {
        return {x*s, y*s, z*s, vx*s, vy*s, vz*s};
    }
    friend PropState operator*(double s, const PropState& p) { return p * s; }

    Vec3 pos() const { return {x, y, z}; }
    Vec3 vel() const { return {vx, vy, vz}; }
};
