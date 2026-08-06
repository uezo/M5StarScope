#pragma once

#include "starscope/types.hpp"

namespace starscope::geomagnetism {

constexpr double kWmm2025Epoch = 2025.0;
constexpr double kWmm2025ValidUntil = 2030.0;

struct MagneticField {
  double declinationDeg = 0.0;
  double horizontalIntensityNanoTesla = 0.0;
  bool valid = false;
};

// Converts UTC to a fractional calendar year suitable for WMM evaluation.
double decimalYear(const CivilTime& utc);

// Evaluates the degree/order-12 World Magnetic Model 2025. Declination is
// positive east and negative west. WMM2025 is valid for 2025.0 <= year < 2030.
MagneticField worldMagneticModel2025(double latitudeDeg, double longitudeDeg,
                                     double year,
                                     double altitudeKilometers = 0.0);

}  // namespace starscope::geomagnetism
