#pragma once
#include "core/math/Constants.h"

// Exponential atmosphere density model with altitude-varying scale heights.
// Piecewise exponential fit to USSA1976, adequate for LEO drag estimation.
namespace AtmosphereModel {

    // Density [kg/m^3] at altitude above Earth's surface [m].
    double density(double altitude_m);

    // Simplified scale height [m] at the given altitude [m].
    double scaleHeight(double altitude_m);

} // namespace AtmosphereModel
