#include "starscope/geomagnetism.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace starscope::geomagnetism {
namespace {

constexpr int kMaximumDegree = 12;
constexpr int kCoefficientCount = 91;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kRadiansToDegrees = 180.0 / kPi;
constexpr double kWgs84SemiMajorKilometers = 6378.137;
constexpr double kWgs84SemiMinorKilometers = 6356.7523142;
constexpr double kMagneticReferenceRadiusKilometers = 6371.2;
constexpr double kWgs84EccentricitySquared =
    1.0 - (kWgs84SemiMinorKilometers * kWgs84SemiMinorKilometers) /
              (kWgs84SemiMajorKilometers * kWgs84SemiMajorKilometers);

struct Coefficient {
  float g;
  float h;
  float gAnnualChange;
  float hAnnualChange;
};

// NOAA/NCEI WMM2025 coefficients, epoch 2025.0, released 2024-11-13.
// Index is n * (n + 1) / 2 + m; element zero is unused.
constexpr Coefficient kCoefficients[kCoefficientCount] = {
    {0.0F, 0.0F, 0.0F, 0.0F},
    {-29351.8F, 0.0F, 12.0F, 0.0F},
    {-1410.8F, 4545.4F, 9.7F, -21.5F},
    {-2556.6F, 0.0F, -11.6F, 0.0F},
    {2951.1F, -3133.6F, -5.2F, -27.7F},
    {1649.3F, -815.1F, -8.0F, -12.1F},
    {1361.0F, 0.0F, -1.3F, 0.0F},
    {-2404.1F, -56.6F, -4.2F, 4.0F},
    {1243.8F, 237.5F, 0.4F, -0.3F},
    {453.6F, -549.5F, -15.6F, -4.1F},
    {895.0F, 0.0F, -1.6F, 0.0F},
    {799.5F, 278.6F, -2.4F, -1.1F},
    {55.7F, -133.9F, -6.0F, 4.1F},
    {-281.1F, 212.0F, 5.6F, 1.6F},
    {12.1F, -375.6F, -7.0F, -4.4F},
    {-233.2F, 0.0F, 0.6F, 0.0F},
    {368.9F, 45.4F, 1.4F, -0.5F},
    {187.2F, 220.2F, 0.0F, 2.2F},
    {-138.7F, -122.9F, 0.6F, 0.4F},
    {-142.0F, 43.0F, 2.2F, 1.7F},
    {20.9F, 106.1F, 0.9F, 1.9F},
    {64.4F, 0.0F, -0.2F, 0.0F},
    {63.8F, -18.4F, -0.4F, 0.3F},
    {76.9F, 16.8F, 0.9F, -1.6F},
    {-115.7F, 48.8F, 1.2F, -0.4F},
    {-40.9F, -59.8F, -0.9F, 0.9F},
    {14.9F, 10.9F, 0.3F, 0.7F},
    {-60.7F, 72.7F, 0.9F, 0.9F},
    {79.5F, 0.0F, 0.0F, 0.0F},
    {-77.0F, -48.9F, -0.1F, 0.6F},
    {-8.8F, -14.4F, -0.1F, 0.5F},
    {59.3F, -1.0F, 0.5F, -0.8F},
    {15.8F, 23.4F, -0.1F, 0.0F},
    {2.5F, -7.4F, -0.8F, -1.0F},
    {-11.1F, -25.1F, -0.8F, 0.6F},
    {14.2F, -2.3F, 0.8F, -0.2F},
    {23.2F, 0.0F, -0.1F, 0.0F},
    {10.8F, 7.1F, 0.2F, -0.2F},
    {-17.5F, -12.6F, 0.0F, 0.5F},
    {2.0F, 11.4F, 0.5F, -0.4F},
    {-21.7F, -9.7F, -0.1F, 0.4F},
    {16.9F, 12.7F, 0.3F, -0.5F},
    {15.0F, 0.7F, 0.2F, -0.6F},
    {-16.8F, -5.2F, 0.0F, 0.3F},
    {0.9F, 3.9F, 0.2F, 0.2F},
    {4.6F, 0.0F, 0.0F, 0.0F},
    {7.8F, -24.8F, -0.1F, -0.3F},
    {3.0F, 12.2F, 0.1F, 0.3F},
    {-0.2F, 8.3F, 0.3F, -0.3F},
    {-2.5F, -3.3F, -0.3F, 0.3F},
    {-13.1F, -5.2F, 0.0F, 0.2F},
    {2.4F, 7.2F, 0.3F, -0.1F},
    {8.6F, -0.6F, -0.1F, -0.2F},
    {-8.7F, 0.8F, 0.1F, 0.4F},
    {-12.9F, 10.0F, -0.1F, 0.1F},
    {-1.3F, 0.0F, 0.1F, 0.0F},
    {-6.4F, 3.3F, 0.0F, 0.0F},
    {0.2F, 0.0F, 0.1F, 0.0F},
    {2.0F, 2.4F, 0.1F, -0.2F},
    {-1.0F, 5.3F, 0.0F, 0.1F},
    {-0.6F, -9.1F, -0.3F, -0.1F},
    {-0.9F, 0.4F, 0.0F, 0.1F},
    {1.5F, -4.2F, -0.1F, 0.0F},
    {0.9F, -3.8F, -0.1F, -0.1F},
    {-2.7F, 0.9F, 0.0F, 0.2F},
    {-3.9F, -9.1F, 0.0F, 0.0F},
    {2.9F, 0.0F, 0.0F, 0.0F},
    {-1.5F, 0.0F, 0.0F, 0.0F},
    {-2.5F, 2.9F, 0.0F, 0.1F},
    {2.4F, -0.6F, 0.0F, 0.0F},
    {-0.6F, 0.2F, 0.0F, 0.1F},
    {-0.1F, 0.5F, -0.1F, 0.0F},
    {-0.6F, -0.3F, 0.0F, 0.0F},
    {-0.1F, -1.2F, 0.0F, 0.1F},
    {1.1F, -1.7F, -0.1F, 0.0F},
    {-1.0F, -2.9F, -0.1F, 0.0F},
    {-0.2F, -1.8F, -0.1F, 0.0F},
    {2.6F, -2.3F, -0.1F, 0.0F},
    {-2.0F, 0.0F, 0.0F, 0.0F},
    {-0.2F, -1.3F, 0.0F, 0.0F},
    {0.3F, 0.7F, 0.0F, 0.0F},
    {1.2F, 1.0F, 0.0F, -0.1F},
    {-1.3F, -1.4F, 0.0F, 0.1F},
    {0.6F, 0.0F, 0.0F, 0.0F},
    {0.6F, 0.6F, 0.1F, 0.0F},
    {0.5F, -0.1F, 0.0F, 0.0F},
    {-0.1F, 0.8F, 0.0F, 0.0F},
    {-0.4F, 0.1F, 0.0F, 0.0F},
    {-0.2F, -1.0F, -0.1F, 0.0F},
    {-1.3F, 0.1F, 0.0F, 0.0F},
    {-0.7F, 0.2F, -0.1F, -0.1F},
};

constexpr int coefficientIndex(int degree, int order) {
  return degree * (degree + 1) / 2 + order;
}

bool leapYear(int year) {
  return year % 400 == 0 || (year % 4 == 0 && year % 100 != 0);
}

}  // namespace

double decimalYear(const CivilTime& utc) {
  static constexpr int kDaysBeforeMonth[] = {0,   31,  59,  90,  120, 151,
                                             181, 212, 243, 273, 304, 334};
  if (utc.month < 1 || utc.month > 12 || utc.day < 1 || utc.day > 31) {
    return static_cast<double>(utc.year);
  }
  int dayOfYear = kDaysBeforeMonth[utc.month - 1] + utc.day;
  if (utc.month > 2 && leapYear(utc.year)) ++dayOfYear;
  const int daysInYear = leapYear(utc.year) ? 366 : 365;
  const double dayFraction =
      (utc.hour * 3600.0 + utc.minute * 60.0 + utc.second) / 86400.0;
  return utc.year + (dayOfYear - 1 + dayFraction) / daysInYear;
}

MagneticField worldMagneticModel2025(double latitudeDeg, double longitudeDeg,
                                     double year,
                                     double altitudeKilometers) {
  MagneticField result;
  if (!std::isfinite(latitudeDeg) || !std::isfinite(longitudeDeg) ||
      !std::isfinite(year) || !std::isfinite(altitudeKilometers) ||
      latitudeDeg < -90.0 || latitudeDeg > 90.0 ||
      year < kWmm2025Epoch || year >= kWmm2025ValidUntil) {
    return result;
  }

  latitudeDeg = std::max(-89.9999, std::min(89.9999, latitudeDeg));
  const double latitude = latitudeDeg * kDegreesToRadians;
  const double longitude = longitudeDeg * kDegreesToRadians;
  const double sinLatitude = std::sin(latitude);
  const double cosLatitude = std::cos(latitude);
  const double primeVerticalRadius =
      kWgs84SemiMajorKilometers /
      std::sqrt(1.0 - kWgs84EccentricitySquared * sinLatitude * sinLatitude);
  const double x =
      (primeVerticalRadius + altitudeKilometers) * cosLatitude;
  const double z =
      (primeVerticalRadius * (1.0 - kWgs84EccentricitySquared) +
       altitudeKilometers) *
      sinLatitude;
  const double radius = std::sqrt(x * x + z * z);
  const double geocentricLatitude = std::asin(z / radius);
  const double sinGeocentric = std::sin(geocentricLatitude);
  const double cosGeocentric = std::cos(geocentricLatitude);

  std::array<double, kMaximumDegree + 1> radiusPower{};
  const double radiusRatio = kMagneticReferenceRadiusKilometers / radius;
  radiusPower[0] = radiusRatio * radiusRatio;
  for (int degree = 1; degree <= kMaximumDegree; ++degree) {
    radiusPower[degree] = radiusPower[degree - 1] * radiusRatio;
  }

  std::array<double, kMaximumDegree + 1> cosLongitude{};
  std::array<double, kMaximumDegree + 1> sinLongitude{};
  cosLongitude[0] = 1.0;
  cosLongitude[1] = std::cos(longitude);
  sinLongitude[1] = std::sin(longitude);
  for (int order = 2; order <= kMaximumDegree; ++order) {
    cosLongitude[order] =
        cosLongitude[order - 1] * cosLongitude[1] -
        sinLongitude[order - 1] * sinLongitude[1];
    sinLongitude[order] =
        sinLongitude[order - 1] * cosLongitude[1] +
        cosLongitude[order - 1] * sinLongitude[1];
  }

  std::array<double, kCoefficientCount> legendre{};
  std::array<double, kCoefficientCount> legendreDerivative{};
  std::array<double, kCoefficientCount> schmidt{};
  legendre[0] = 1.0;
  schmidt[0] = 1.0;
  for (int degree = 1; degree <= kMaximumDegree; ++degree) {
    for (int order = 0; order <= degree; ++order) {
      const int index = coefficientIndex(degree, order);
      if (degree == order) {
        const int previous = coefficientIndex(degree - 1, order - 1);
        legendre[index] = cosGeocentric * legendre[previous];
        legendreDerivative[index] =
            cosGeocentric * legendreDerivative[previous] +
            sinGeocentric * legendre[previous];
      } else if (degree == 1 && order == 0) {
        legendre[index] = sinGeocentric;
        legendreDerivative[index] = -cosGeocentric;
      } else {
        const int previous = coefficientIndex(degree - 1, order);
        if (order > degree - 2) {
          legendre[index] = sinGeocentric * legendre[previous];
          legendreDerivative[index] =
              sinGeocentric * legendreDerivative[previous] -
              cosGeocentric * legendre[previous];
        } else {
          const int twoBack = coefficientIndex(degree - 2, order);
          const double recursion =
              ((degree - 1) * (degree - 1) - order * order) /
              static_cast<double>((2 * degree - 1) * (2 * degree - 3));
          legendre[index] = sinGeocentric * legendre[previous] -
                            recursion * legendre[twoBack];
          legendreDerivative[index] =
              sinGeocentric * legendreDerivative[previous] -
              cosGeocentric * legendre[previous] -
              recursion * legendreDerivative[twoBack];
        }
      }
    }
  }

  for (int degree = 1; degree <= kMaximumDegree; ++degree) {
    schmidt[coefficientIndex(degree, 0)] =
        schmidt[coefficientIndex(degree - 1, 0)] * (2 * degree - 1) /
        static_cast<double>(degree);
    for (int order = 1; order <= degree; ++order) {
      schmidt[coefficientIndex(degree, order)] =
          schmidt[coefficientIndex(degree, order - 1)] *
          std::sqrt((degree - order + 1) * (order == 1 ? 2.0 : 1.0) /
                    (degree + order));
    }
  }

  double north = 0.0;
  double east = 0.0;
  double down = 0.0;
  const double yearOffset = year - kWmm2025Epoch;
  for (int degree = 1; degree <= kMaximumDegree; ++degree) {
    for (int order = 0; order <= degree; ++order) {
      const int index = coefficientIndex(degree, order);
      const double normalizedLegendre = legendre[index] * schmidt[index];
      const double latitudeDerivative =
          -legendreDerivative[index] * schmidt[index];
      const auto& coefficient = kCoefficients[index];
      const double g = coefficient.g + coefficient.gAnnualChange * yearOffset;
      const double h = coefficient.h + coefficient.hAnnualChange * yearOffset;
      const double longitudeField =
          g * cosLongitude[order] + h * sinLongitude[order];
      const double longitudeDerivative =
          g * sinLongitude[order] - h * cosLongitude[order];
      down -= radiusPower[degree] * (degree + 1) * longitudeField *
              normalizedLegendre;
      east += radiusPower[degree] * order * longitudeDerivative *
              normalizedLegendre;
      north -= radiusPower[degree] * longitudeField * latitudeDerivative;
    }
  }
  if (std::fabs(cosGeocentric) > 1.0e-10) east /= cosGeocentric;

  const double latitudeDifference = geocentricLatitude - latitude;
  const double geodeticNorth =
      north * std::cos(latitudeDifference) -
      down * std::sin(latitudeDifference);
  result.declinationDeg =
      std::atan2(east, geodeticNorth) * kRadiansToDegrees;
  result.horizontalIntensityNanoTesla =
      std::sqrt(geodeticNorth * geodeticNorth + east * east);
  result.valid = std::isfinite(result.declinationDeg) &&
                 std::isfinite(result.horizontalIntensityNanoTesla) &&
                 result.horizontalIntensityNanoTesla > 0.0;
  return result;
}

}  // namespace starscope::geomagnetism
