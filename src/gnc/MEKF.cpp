#include "gnc/MEKF.h"
#include <cstring>
#include <cmath>

MEKF::MEKF() {
    reset();
}

void MEKF::initialize(const Quat& initial_attitude) {
    attitude_   = initial_attitude.normalized();
    omega_      = {};
    gyro_bias_  = {};
    std::memset(P_, 0, sizeof(P_));
    for (int i = 0; i < 3; i++) { P_[i][i] = 0.01; P_[i+3][i+3] = 0.001; }
    initialized_ = true;
}

void MEKF::reset() {
    attitude_   = Quat::identity();
    omega_      = {};
    gyro_bias_  = {};
    std::memset(P_, 0, sizeof(P_));
    for (int i = 0; i < 6; i++) P_[i][i] = 0.1;
    initialized_ = false;
}

void MEKF::propagate(const Vec3& gyro_meas, double dt) {
    if (!initialized_) return;

    // Bias-corrected angular rate
    omega_ = gyro_meas - gyro_bias_;

    // Quaternion kinematics: q_dot = 0.5 * q * [0, omega]
    attitude_ = (attitude_ + attitude_.qdot(omega_) * dt).normalized();

    // Covariance propagation: P = F*P*F^T + Q
    double F[6][6], Q[6][6];
    computeF(omega_, dt, F);
    computeQ(dt, Q);

    double temp[6][6]{}, P_new[6][6]{};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 6; k++)
                temp[i][j] += F[i][k] * P_[k][j];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            P_new[i][j] = Q[i][j];
            for (int k = 0; k < 6; k++)
                P_new[i][j] += temp[i][k] * F[j][k];  // F^T[k][j] = F[j][k]
        }
    std::memcpy(P_, P_new, sizeof(P_));
}

void MEKF::update(const Vec3& r1_eci, const Vec3& r2_eci,
                  const Vec3& b1_body, const Vec3& b2_body) {
    const Quat meas = triad(r1_eci, r2_eci, b1_body, b2_body);
    if (!initialized_) { initialize(meas); return; }

    // Innovation: error quaternion → rotation vector
    Quat q_err = meas * attitude_.conjugate();
    if (q_err.w < 0.0) q_err = -q_err;
    const Vec3 innovation = q_err.toRotVec();

    kalmanUpdate(innovation, att_meas_noise_ * att_meas_noise_);
}

void MEKF::updateStarTracker(const Quat& meas_attitude) {
    if (!initialized_) { initialize(meas_attitude); return; }

    Quat q_err = meas_attitude * attitude_.conjugate();
    if (q_err.w < 0.0) q_err = -q_err;
    const Vec3 innovation = q_err.toRotVec();

    kalmanUpdate(innovation, att_meas_noise_ * att_meas_noise_);
}

void MEKF::kalmanUpdate(const Vec3& innovation, double R_scalar) {
    // S = P[0:3, 0:3] + R*I  (H = [I | 0])
    double S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] = P_[i][j] + (i == j ? R_scalar : 0.0);

    // 3×3 matrix inverse (cofactors)
    const double det =
        S[0][0]*(S[1][1]*S[2][2]-S[1][2]*S[2][1])
       -S[0][1]*(S[1][0]*S[2][2]-S[1][2]*S[2][0])
       +S[0][2]*(S[1][0]*S[2][1]-S[1][1]*S[2][0]);
    if (std::abs(det) < 1e-15) return;

    double Si[3][3];
    Si[0][0] = (S[1][1]*S[2][2]-S[1][2]*S[2][1])/det;
    Si[0][1] = (S[0][2]*S[2][1]-S[0][1]*S[2][2])/det;
    Si[0][2] = (S[0][1]*S[1][2]-S[0][2]*S[1][1])/det;
    Si[1][0] = (S[1][2]*S[2][0]-S[1][0]*S[2][2])/det;
    Si[1][1] = (S[0][0]*S[2][2]-S[0][2]*S[2][0])/det;
    Si[1][2] = (S[0][2]*S[1][0]-S[0][0]*S[1][2])/det;
    Si[2][0] = (S[1][0]*S[2][1]-S[1][1]*S[2][0])/det;
    Si[2][1] = (S[0][1]*S[2][0]-S[0][0]*S[2][1])/det;
    Si[2][2] = (S[0][0]*S[1][1]-S[0][1]*S[1][0])/det;

    // Kalman gain: K (6×3) = P * H^T * S^-1  (H^T = [I; 0])
    double K[6][3]{};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                K[i][j] += P_[i][k] * Si[k][j];

    // Corrections
    Vec3 att_corr{}, bias_corr{};
    const double iv[3]{innovation.x, innovation.y, innovation.z};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            att_corr.x  += K[0][j] * iv[j];
            att_corr.y  += K[1][j] * iv[j];
            att_corr.z  += K[2][j] * iv[j];
            bias_corr.x += K[3][j] * iv[j];
            bias_corr.y += K[4][j] * iv[j];
            bias_corr.z += K[5][j] * iv[j];
        }
    att_corr  = {K[0][0]*iv[0]+K[0][1]*iv[1]+K[0][2]*iv[2],
                 K[1][0]*iv[0]+K[1][1]*iv[1]+K[1][2]*iv[2],
                 K[2][0]*iv[0]+K[2][1]*iv[1]+K[2][2]*iv[2]};
    bias_corr = {K[3][0]*iv[0]+K[3][1]*iv[1]+K[3][2]*iv[2],
                 K[4][0]*iv[0]+K[4][1]*iv[1]+K[4][2]*iv[2],
                 K[5][0]*iv[0]+K[5][1]*iv[1]+K[5][2]*iv[2]};

    // Multiplicative attitude correction
    attitude_ = (Quat::fromRotVec(att_corr) * attitude_).normalized();
    gyro_bias_ = gyro_bias_ + bias_corr;

    // Covariance update: P = (I - K*H) * P
    double P_new[6][6]{};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            P_new[i][j] = P_[i][j];
            for (int k = 0; k < 3; k++)
                P_new[i][j] -= K[i][k] * P_[k][j];
        }
    std::memcpy(P_, P_new, sizeof(P_));
}

Quat MEKF::triad(const Vec3& r1, const Vec3& r2,
                  const Vec3& b1, const Vec3& b2) {
    // Reference triad (ECI)
    const Vec3 t1_r = r1.normalized();
    const Vec3 t2_r = r1.cross(r2).normalized();
    const Vec3 t3_r = t1_r.cross(t2_r);

    // Body triad
    const Vec3 t1_b = b1.normalized();
    const Vec3 t2_b = b1.cross(b2).normalized();
    const Vec3 t3_b = t1_b.cross(t2_b);

    // R_body_to_eci = [t1_r t2_r t3_r] * [t1_b t2_b t3_b]^T
    // R[i][j] = sum_k t_r[k].comp(i) * t_b[k].comp(j)
    const Vec3* tr[3] = {&t1_r, &t2_r, &t3_r};
    const Vec3* tb[3] = {&t1_b, &t2_b, &t3_b};
    const double* trv[3][3] = {{&tr[0]->x,&tr[0]->y,&tr[0]->z},
                                {&tr[1]->x,&tr[1]->y,&tr[1]->z},
                                {&tr[2]->x,&tr[2]->y,&tr[2]->z}};
    const double* tbv[3][3] = {{&tb[0]->x,&tb[0]->y,&tb[0]->z},
                                {&tb[1]->x,&tb[1]->y,&tb[1]->z},
                                {&tb[2]->x,&tb[2]->y,&tb[2]->z}};

    // R_eci[:,col] = sum_k t_r[k] * t_b[k].comp(col)
    // cx (col 0) = sum_k t_r[k] * t_b[k].x
    Vec3 cx{}, cy{}, cz{};
    for (int k = 0; k < 3; k++) {
        cx.x += *trv[k][0] * *tbv[k][0];
        cx.y += *trv[k][1] * *tbv[k][0];
        cx.z += *trv[k][2] * *tbv[k][0];

        cy.x += *trv[k][0] * *tbv[k][1];
        cy.y += *trv[k][1] * *tbv[k][1];
        cy.z += *trv[k][2] * *tbv[k][1];

        cz.x += *trv[k][0] * *tbv[k][2];
        cz.y += *trv[k][1] * *tbv[k][2];
        cz.z += *trv[k][2] * *tbv[k][2];
    }
    return Quat::fromColAxes(cx, cy, cz);
}

void MEKF::computeF(const Vec3& omega, double dt, double F[6][6]) const {
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            F[i][j] = (i == j) ? 1.0 : 0.0;
    // Attitude error block: I - [omega×]*dt
    F[0][1] =  omega.z * dt;  F[0][2] = -omega.y * dt;
    F[1][0] = -omega.z * dt;  F[1][2] =  omega.x * dt;
    F[2][0] =  omega.y * dt;  F[2][1] = -omega.x * dt;
    // Bias coupling: -I*dt
    F[0][3] = -dt; F[1][4] = -dt; F[2][5] = -dt;
}

void MEKF::computeQ(double dt, double Q[6][6]) const {
    std::memset(Q, 0, 6*6*sizeof(double));
    const double gv = gyro_noise_  * gyro_noise_  * dt;
    const double bv = bias_noise_  * bias_noise_  * dt;
    Q[0][0] = gv; Q[1][1] = gv; Q[2][2] = gv;
    Q[3][3] = bv; Q[4][4] = bv; Q[5][5] = bv;
}
