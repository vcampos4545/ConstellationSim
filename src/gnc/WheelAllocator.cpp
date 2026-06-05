#include "gnc/WheelAllocator.h"
#include <cstring>
#include <cmath>
#include <algorithm>
#include <stdexcept>

WheelAllocator::WheelAllocator() {
    WheelConfig cfg;
    cfg.spin_axes = {{1,0,0}, {0,1,0}, {0,0,1}};
    cfg.max_torque_Nm = 0.2;
    *this = WheelAllocator(cfg);
}

WheelAllocator::WheelAllocator(const WheelConfig& cfg)
    : axes_(cfg.spin_axes), max_torque_Nm_(cfg.max_torque_Nm)
{
    if (axes_.empty()) throw std::runtime_error("WheelAllocator: no wheels configured");
    buildPseudoinverse();
}

// W (3×N): W[row][col] = axes_[col].comp(row)
// W^+ = W^T * (W*W^T)^-1   →  pinv_[i][j] = row i of W^+
void WheelAllocator::buildPseudoinverse() {
    const int N = static_cast<int>(axes_.size());
    // Compute WWT (3×3) = W * W^T
    double WWT[3][3]{};
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            for (int k = 0; k < N; k++) {
                const double* a = &axes_[k].x;
                WWT[r][c] += a[r] * a[c];
            }

    // Invert WWT (3×3)
    const double det =
        WWT[0][0]*(WWT[1][1]*WWT[2][2]-WWT[1][2]*WWT[2][1])
       -WWT[0][1]*(WWT[1][0]*WWT[2][2]-WWT[1][2]*WWT[2][0])
       +WWT[0][2]*(WWT[1][0]*WWT[2][1]-WWT[1][1]*WWT[2][0]);
    if (std::abs(det) < 1e-15)
        throw std::runtime_error("WheelAllocator: wheel axes are coplanar");

    double WWTi[3][3];
    WWTi[0][0] = (WWT[1][1]*WWT[2][2]-WWT[1][2]*WWT[2][1])/det;
    WWTi[0][1] = (WWT[0][2]*WWT[2][1]-WWT[0][1]*WWT[2][2])/det;
    WWTi[0][2] = (WWT[0][1]*WWT[1][2]-WWT[0][2]*WWT[1][1])/det;
    WWTi[1][0] = (WWT[1][2]*WWT[2][0]-WWT[1][0]*WWT[2][2])/det;
    WWTi[1][1] = (WWT[0][0]*WWT[2][2]-WWT[0][2]*WWT[2][0])/det;
    WWTi[1][2] = (WWT[0][2]*WWT[1][0]-WWT[0][0]*WWT[1][2])/det;
    WWTi[2][0] = (WWT[1][0]*WWT[2][1]-WWT[1][1]*WWT[2][0])/det;
    WWTi[2][1] = (WWT[0][1]*WWT[2][0]-WWT[0][0]*WWT[2][1])/det;
    WWTi[2][2] = (WWT[0][0]*WWT[1][1]-WWT[0][1]*WWT[1][0])/det;

    // W^+ = W^T * WWT^-1  →  pinv_[i][j] = sum_k W^T[i][k] * WWTi[k][j]
    //                                      = sum_k axes_[i].comp(k) * WWTi[k][j]
    pinv_.resize(N);
    for (int i = 0; i < N; i++) {
        const double* a = &axes_[i].x;
        for (int j = 0; j < 3; j++) {
            pinv_[i][j] = 0.0;
            for (int k = 0; k < 3; k++)
                pinv_[i][j] += a[k] * WWTi[k][j];
        }
    }
}

std::vector<double> WheelAllocator::allocate(const Vec3& tau_body) const {
    const double tv[3]{tau_body.x, tau_body.y, tau_body.z};
    std::vector<double> out(axes_.size());
    for (int i = 0; i < static_cast<int>(axes_.size()); i++) {
        double v = 0.0;
        for (int j = 0; j < 3; j++) v += pinv_[i][j] * tv[j];
        out[i] = std::clamp(v, -max_torque_Nm_, max_torque_Nm_);
    }
    return out;
}

Vec3 WheelAllocator::recompose(const std::vector<double>& wt) const {
    Vec3 tau{};
    for (int i = 0; i < static_cast<int>(axes_.size()); i++)
        tau += axes_[i] * wt[i];
    return tau;
}
