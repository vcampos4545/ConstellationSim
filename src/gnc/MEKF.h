#pragma once
#include "core/math/Quat.h"
#include "core/math/Vec3.h"

// Multiplicative Extended Kalman Filter for attitude determination.
//
// State: [delta_theta(3), gyro_bias(3)] — 6 elements.
// Propagation: integrate gyro measurement (corrected for bias) using
//   quaternion kinematics; propagate 6×6 covariance with state transition matrix.
// Update: TRIAD algorithm constructs an attitude measurement from two reference
//   vector pairs, then a Kalman correction is applied multiplicatively to the
//   quaternion estimate so the unit constraint is always preserved.
//
// Reference: Markley & Crassidis "Fundamentals of Spacecraft ADCS", Ch. 7.
class MEKF {
public:
    MEKF();

    void initialize(const Quat& initial_attitude);
    void reset();

    // Call every IMU sample (high rate — gyro integration).
    void propagate(const Vec3& gyro_meas, double dt);

    // Call when reference-vector measurements are available (TRIAD update).
    // r1, r2: reference vectors in ECI.  b1, b2: same vectors in body frame.
    void update(const Vec3& r1_eci, const Vec3& r2_eci,
                const Vec3& b1_body, const Vec3& b2_body);

    // Update from star tracker quaternion measurement directly.
    void updateStarTracker(const Quat& meas_attitude);

    Quat getAttitude()       const { return attitude_; }
    Vec3 getAngularVelocity() const { return omega_; }
    Vec3 getGyroBias()       const { return gyro_bias_; }
    bool isInitialized()     const { return initialized_; }

    void setGyroNoise(double sigma)    { gyro_noise_  = sigma; }
    void setGyroBiasNoise(double sigma){ bias_noise_  = sigma; }
    void setAttMeasNoise(double sigma) { att_meas_noise_ = sigma; }

private:
    Quat attitude_{};
    Vec3 omega_{};
    Vec3 gyro_bias_{};

    double P_[6][6]{};
    double gyro_noise_{1e-3};       // [rad/s/sqrt(Hz)]
    double bias_noise_{1e-4};       // [rad/s^2/sqrt(Hz)]
    double att_meas_noise_{0.01};   // [rad]
    bool   initialized_{false};

    static Quat triad(const Vec3& r1, const Vec3& r2,
                      const Vec3& b1, const Vec3& b2);

    void computeF(const Vec3& omega, double dt, double F[6][6]) const;
    void computeQ(double dt, double Q[6][6]) const;
    void kalmanUpdate(const Vec3& innovation, double R_scalar);
};
