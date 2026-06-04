#pragma once
#include "physics/ForceModel.h"

// Combined J2 + J3 + J4 zonal harmonic perturbation.
//
// Replaces the standalone J2Perturbation when higher-order terms are enabled.
// All three terms share intermediate quantities (r powers, s = z/r) to avoid
// redundant sqrt/pow calls.
//
// Acceleration formulas (with s = z/r):
//   J2: coeff2 = 3/2 · μ·J2·Re² / r⁵
//       ax = coeff2·x·(5s²−1),  ay = coeff2·y·(5s²−1),  az = coeff2·z·(5s²−3)
//
//   J3: coeff3 = μ·J3·Re³ / r⁶
//       ax = coeff3·x·(35s³−15s)/2,  ay = same·y
//       az = coeff3·|r|·(35s⁴−30s²+3)/2
//
//   J4: coeff4 = μ·J4·Re⁴ / r⁷
//       ax = coeff4·x·(315s⁴−210s²+15)/8,  ay = same·y
//       az = coeff4·|r|·(315s⁵−350s³+75s)/8
class ZonalHarmonics final : public ForceModel {
public:
    ZonalHarmonics(double mu, double re,
                   double j2, double j3 = 0.0, double j4 = 0.0)
        : mu_(mu), re_(re), j2_(j2), j3_(j3), j4_(j4) {}

    Vec3 acceleration(const OrbitState& state,
                      const PhysicalProperties&,
                      double) const override;

    std::string name() const override { return "ZonalJ2J3J4"; }

private:
    double mu_, re_, j2_, j3_, j4_;
};
