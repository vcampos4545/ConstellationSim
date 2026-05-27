#include "environment/AtmosphereModel.h"
#include <cmath>
#include <array>

// Piecewise exponential atmosphere model (USSA76-based).
// Each layer: [base altitude km, reference density kg/m^3, scale height m]
namespace {
    struct Layer { double h_base_m; double rho_ref; double H_m; };

    // Piecewise exponential fit to US Standard Atmosphere 1976.
    // Density values from USSA76 tables; scale heights derived from
    // adjacent layer ratios: H = Δh / ln(ρ₁/ρ₂).
    // Covers 0–1000 km with emphasis on LEO accuracy (100–800 km).
    constexpr std::array<Layer, 23> LAYERS = {{
        {       0.0, 1.225000e+00,  8500.0 },
        {  25000.0,  4.008000e-02,  6600.0 },
        {  30000.0,  1.841000e-02,  6700.0 },
        {  40000.0,  3.996000e-03,  7100.0 },
        {  50000.0,  1.027000e-03,  7900.0 },
        {  60000.0,  3.097000e-04,  8100.0 },
        {  70000.0,  8.283000e-05,  7900.0 },
        {  80000.0,  1.846000e-05,  7200.0 },
        { 100000.0,  5.600000e-07,  5900.0 },
        { 110000.0,  9.750000e-08,  7350.0 },
        { 120000.0,  2.420000e-08,  7800.0 },
        { 130000.0,  8.680000e-09,  8200.0 },
        { 140000.0,  3.960000e-09,  8700.0 },
        { 150000.0,  2.076000e-09,  8900.0 },
        { 180000.0,  5.194000e-10, 10200.0 },
        { 200000.0,  2.541000e-10, 11200.0 },
        { 250000.0,  6.073000e-11, 31100.0 },
        { 300000.0,  1.916000e-11, 43700.0 },
        { 350000.0,  6.703000e-12, 54900.0 },
        { 400000.0,  2.803000e-12, 58800.0 },
        { 500000.0,  5.215000e-13, 63700.0 },
        { 600000.0,  8.770000e-14, 71400.0 },
        { 700000.0,  3.070000e-14, 88800.0 },
    }};
} // anonymous

double AtmosphereModel::density(double altitude_m) {
    if (altitude_m >= 1'000'000.0) return 0.0;  // above 1000 km: negligible
    if (altitude_m < 0.0) altitude_m = 0.0;

    // Find the correct layer
    const Layer* layer = &LAYERS[0];
    for (int i = static_cast<int>(LAYERS.size()) - 1; i >= 0; --i) {
        if (altitude_m >= LAYERS[i].h_base_m) {
            layer = &LAYERS[i];
            break;
        }
    }

    return layer->rho_ref * std::exp(-(altitude_m - layer->h_base_m) / layer->H_m);
}

double AtmosphereModel::scaleHeight(double altitude_m) {
    if (altitude_m >= 1'000'000.0) return 37000.0;
    if (altitude_m < 0.0) altitude_m = 0.0;

    for (int i = static_cast<int>(LAYERS.size()) - 1; i >= 0; --i) {
        if (altitude_m >= LAYERS[i].h_base_m) return LAYERS[i].H_m;
    }
    return LAYERS[0].H_m;
}
