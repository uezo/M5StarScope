#include "starscope/astronomy.hpp"

#include <algorithm>
#include <cmath>

namespace starscope::astronomy {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

double clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

double normalizeDegreesDouble(double value) {
  value = std::fmod(value, 360.0);
  return value < 0.0 ? value + 360.0 : value;
}

double sinDeg(double angleDeg) { return std::sin(angleDeg * kDegToRad); }
double cosDeg(double angleDeg) { return std::cos(angleDeg * kDegToRad); }

EquatorialCoordinate eclipticToEquatorial(double longitudeDeg,
                                          double latitudeDeg,
                                          double julianDate) {
  const double centuries = (julianDate - 2451545.0) / 36525.0;
  const double obliquityDeg = 23.439291 - 0.0130042 * centuries;
  const double longitude = longitudeDeg * kDegToRad;
  const double latitude = latitudeDeg * kDegToRad;
  const double obliquity = obliquityDeg * kDegToRad;
  const double x = std::cos(longitude) * std::cos(latitude);
  const double y = std::sin(longitude) * std::cos(latitude);
  const double z = std::sin(latitude);
  const double equatorialY = y * std::cos(obliquity) - z * std::sin(obliquity);
  const double equatorialZ = y * std::sin(obliquity) + z * std::cos(obliquity);
  return {normalizeDegreesDouble(std::atan2(equatorialY, x) * kRadToDeg),
          std::asin(clamp(equatorialZ, -1.0, 1.0)) * kRadToDeg};
}

struct OrbitalElements {
  double ascendingNodeDeg;
  double inclinationDeg;
  double perihelionDeg;
  double semiMajorAxisAu;
  double eccentricity;
  double meanAnomalyDeg;
};

struct EclipticPosition {
  double x;
  double y;
  double z;
  double radius;
  double longitudeDeg;
  double latitudeDeg;
};

EclipticPosition heliocentricPosition(const OrbitalElements& elements) {
  const double meanAnomaly =
      normalizeDegreesDouble(elements.meanAnomalyDeg) * kDegToRad;
  double eccentricAnomaly = meanAnomaly;
  for (int i = 0; i < 6; ++i) {
    eccentricAnomaly -=
        (eccentricAnomaly -
         elements.eccentricity * std::sin(eccentricAnomaly) - meanAnomaly) /
        (1.0 - elements.eccentricity * std::cos(eccentricAnomaly));
  }
  const double orbitalX = elements.semiMajorAxisAu *
                          (std::cos(eccentricAnomaly) - elements.eccentricity);
  const double orbitalY =
      elements.semiMajorAxisAu *
      std::sqrt(1.0 - elements.eccentricity * elements.eccentricity) *
      std::sin(eccentricAnomaly);
  const double radius = std::sqrt(orbitalX * orbitalX + orbitalY * orbitalY);
  const double trueAnomalyDeg = std::atan2(orbitalY, orbitalX) * kRadToDeg;
  const double argumentDeg = trueAnomalyDeg + elements.perihelionDeg;
  const double nodeCos = cosDeg(elements.ascendingNodeDeg);
  const double nodeSin = sinDeg(elements.ascendingNodeDeg);
  const double argumentCos = cosDeg(argumentDeg);
  const double argumentSin = sinDeg(argumentDeg);
  const double inclinationCos = cosDeg(elements.inclinationDeg);
  const double inclinationSin = sinDeg(elements.inclinationDeg);

  EclipticPosition result;
  result.x = radius *
             (nodeCos * argumentCos -
              nodeSin * argumentSin * inclinationCos);
  result.y = radius *
             (nodeSin * argumentCos +
              nodeCos * argumentSin * inclinationCos);
  result.z = radius * argumentSin * inclinationSin;
  result.radius = radius;
  result.longitudeDeg =
      normalizeDegreesDouble(std::atan2(result.y, result.x) * kRadToDeg);
  result.latitudeDeg =
      std::atan2(result.z,
                 std::sqrt(result.x * result.x + result.y * result.y)) *
      kRadToDeg;
  return result;
}

void applyEclipticCorrections(EclipticPosition& position,
                              double longitudeCorrectionDeg,
                              double latitudeCorrectionDeg) {
  position.longitudeDeg = normalizeDegreesDouble(
      position.longitudeDeg + longitudeCorrectionDeg);
  position.latitudeDeg += latitudeCorrectionDeg;
  position.x = position.radius * cosDeg(position.longitudeDeg) *
               cosDeg(position.latitudeDeg);
  position.y = position.radius * sinDeg(position.longitudeDeg) *
               cosDeg(position.latitudeDeg);
  position.z = position.radius * sinDeg(position.latitudeDeg);
}

}  // namespace

float normalizeDegrees(float value) {
  value = std::fmod(value, 360.0F);
  return value < 0.0F ? value + 360.0F : value;
}

float signedAngleDifference(float targetDeg, float referenceDeg) {
  float delta = normalizeDegrees(targetDeg) - normalizeDegrees(referenceDeg);
  if (delta > 180.0F) delta -= 360.0F;
  if (delta < -180.0F) delta += 360.0F;
  return delta;
}

double julianDateUtc(const CivilTime& localTime, int utcOffsetMinutes) {
  int year = localTime.year;
  int month = localTime.month;
  const double fractionalDay =
      (localTime.hour - utcOffsetMinutes / 60.0 +
       localTime.minute / 60.0 + localTime.second / 3600.0) /
      24.0;
  double day = localTime.day + fractionalDay;

  if (month <= 2) {
    --year;
    month += 12;
  }
  const int century = year / 100;
  const int correction = 2 - century + century / 4;
  return std::floor(365.25 * (year + 4716)) +
         std::floor(30.6001 * (month + 1)) + day + correction - 1524.5;
}

CivilTime civilTimeUtc(double julianDate) {
  // Round to the nearest second before applying the standard inverse-Julian
  // calendar conversion. Advancing the Julian value first naturally handles
  // 23:59:59.5 rolling into the following day.
  const double roundedJulianDate = julianDate + 0.5 / 86400.0;
  const double shifted = roundedJulianDate + 0.5;
  const double integralDay = std::floor(shifted);
  const double fractionalDay = shifted - integralDay;

  double adjustedDay = integralDay;
  if (integralDay >= 2299161.0) {
    const double alpha =
        std::floor((integralDay - 1867216.25) / 36524.25);
    adjustedDay = integralDay + 1.0 + alpha - std::floor(alpha / 4.0);
  }
  const double b = adjustedDay + 1524.0;
  const int c = static_cast<int>(std::floor((b - 122.1) / 365.25));
  const int d = static_cast<int>(std::floor(365.25 * c));
  const int e = static_cast<int>(std::floor((b - d) / 30.6001));
  const int day =
      static_cast<int>(b - d - std::floor(30.6001 * e));
  const int month = e < 14 ? e - 1 : e - 13;
  const int year = month > 2 ? c - 4716 : c - 4715;

  const int secondsOfDay =
      static_cast<int>(std::floor(fractionalDay * 86400.0 + 1e-6));
  return {year, month, day, secondsOfDay / 3600,
          (secondsOfDay % 3600) / 60, secondsOfDay % 60};
}

double greenwichMeanSiderealDeg(double julianDate) {
  const double d = julianDate - 2451545.0;
  const double t = d / 36525.0;
  double value = 280.46061837 + 360.98564736629 * d +
                 0.000387933 * t * t - t * t * t / 38710000.0;
  value = std::fmod(value, 360.0);
  return value < 0.0 ? value + 360.0 : value;
}

double localSiderealDeg(double julianDate, double longitudeDeg) {
  double value = greenwichMeanSiderealDeg(julianDate) + longitudeDeg;
  value = std::fmod(value, 360.0);
  return value < 0.0 ? value + 360.0 : value;
}

SolarSystemEphemeris solarSystemEphemeris(double julianDate) {
  // Paul Schlyter's compact orbital elements, with the principal lunar
  // perturbations. Epoch day zero is 2000 Jan 0.0 UT (JD 2451543.5).
  const double days = julianDate - 2451543.5;

  const double sunPerihelionDeg = 282.9404 + 4.70935e-5 * days;
  const double sunEccentricity = 0.016709 - 1.151e-9 * days;
  const double sunMeanAnomalyDeg =
      normalizeDegreesDouble(356.0470 + 0.9856002585 * days);
  const double sunMeanAnomaly = sunMeanAnomalyDeg * kDegToRad;
  double sunEccentricAnomaly = sunMeanAnomaly;
  for (int i = 0; i < 4; ++i) {
    sunEccentricAnomaly -=
        (sunEccentricAnomaly -
         sunEccentricity * std::sin(sunEccentricAnomaly) - sunMeanAnomaly) /
        (1.0 - sunEccentricity * std::cos(sunEccentricAnomaly));
  }
  const double sunX = std::cos(sunEccentricAnomaly) - sunEccentricity;
  const double sunY = std::sqrt(1.0 - sunEccentricity * sunEccentricity) *
                      std::sin(sunEccentricAnomaly);
  const double sunTrueAnomalyDeg = std::atan2(sunY, sunX) * kRadToDeg;
  const double sunLongitudeDeg = normalizeDegreesDouble(
      sunTrueAnomalyDeg + sunPerihelionDeg);
  const double sunDistanceAu = std::sqrt(sunX * sunX + sunY * sunY);
  const double sunGeocentricX = sunDistanceAu * cosDeg(sunLongitudeDeg);
  const double sunGeocentricY = sunDistanceAu * sinDeg(sunLongitudeDeg);

  const double ascendingNodeDeg =
      normalizeDegreesDouble(125.1228 - 0.0529538083 * days);
  constexpr double inclinationDeg = 5.1454;
  const double lunarPerigeeDeg =
      normalizeDegreesDouble(318.0634 + 0.1643573223 * days);
  constexpr double lunarEccentricity = 0.054900;
  const double lunarMeanAnomalyDeg =
      normalizeDegreesDouble(115.3654 + 13.0649929509 * days);
  const double lunarMeanAnomaly = lunarMeanAnomalyDeg * kDegToRad;
  double lunarEccentricAnomaly = lunarMeanAnomaly;
  for (int i = 0; i < 5; ++i) {
    lunarEccentricAnomaly -=
        (lunarEccentricAnomaly -
         lunarEccentricity * std::sin(lunarEccentricAnomaly) -
         lunarMeanAnomaly) /
        (1.0 - lunarEccentricity * std::cos(lunarEccentricAnomaly));
  }
  const double lunarX = 60.2666 *
                        (std::cos(lunarEccentricAnomaly) - lunarEccentricity);
  const double lunarY =
      60.2666 * std::sqrt(1.0 - lunarEccentricity * lunarEccentricity) *
      std::sin(lunarEccentricAnomaly);
  const double lunarTrueAnomalyDeg = std::atan2(lunarY, lunarX) * kRadToDeg;
  const double argumentDeg = lunarTrueAnomalyDeg + lunarPerigeeDeg;
  const double nodeCos = cosDeg(ascendingNodeDeg);
  const double nodeSin = sinDeg(ascendingNodeDeg);
  const double argumentCos = cosDeg(argumentDeg);
  const double argumentSin = sinDeg(argumentDeg);
  const double inclinationCos = cosDeg(inclinationDeg);
  const double inclinationSin = sinDeg(inclinationDeg);
  const double eclipticX =
      nodeCos * argumentCos - nodeSin * argumentSin * inclinationCos;
  const double eclipticY =
      nodeSin * argumentCos + nodeCos * argumentSin * inclinationCos;
  const double eclipticZ = argumentSin * inclinationSin;
  double lunarLongitudeDeg =
      normalizeDegreesDouble(std::atan2(eclipticY, eclipticX) * kRadToDeg);
  double lunarLatitudeDeg = std::atan2(
      eclipticZ, std::sqrt(eclipticX * eclipticX + eclipticY * eclipticY)) *
                            kRadToDeg;

  const double sunMeanLongitudeDeg = normalizeDegreesDouble(
      sunMeanAnomalyDeg + sunPerihelionDeg);
  const double lunarMeanLongitudeDeg = normalizeDegreesDouble(
      ascendingNodeDeg + lunarPerigeeDeg + lunarMeanAnomalyDeg);
  const double elongationDeg =
      normalizeDegreesDouble(lunarMeanLongitudeDeg - sunMeanLongitudeDeg);
  const double argumentLatitudeDeg =
      normalizeDegreesDouble(lunarMeanLongitudeDeg - ascendingNodeDeg);

  lunarLongitudeDeg +=
      -1.274 * sinDeg(lunarMeanAnomalyDeg - 2.0 * elongationDeg) +
      0.658 * sinDeg(2.0 * elongationDeg) -
      0.186 * sinDeg(sunMeanAnomalyDeg) -
      0.059 * sinDeg(2.0 * lunarMeanAnomalyDeg - 2.0 * elongationDeg) -
      0.057 * sinDeg(lunarMeanAnomalyDeg - 2.0 * elongationDeg +
                     sunMeanAnomalyDeg) +
      0.053 * sinDeg(lunarMeanAnomalyDeg + 2.0 * elongationDeg) +
      0.046 * sinDeg(2.0 * elongationDeg - sunMeanAnomalyDeg) +
      0.041 * sinDeg(lunarMeanAnomalyDeg - sunMeanAnomalyDeg) -
      0.035 * sinDeg(elongationDeg) -
      0.031 * sinDeg(lunarMeanAnomalyDeg + sunMeanAnomalyDeg) -
      0.015 * sinDeg(2.0 * argumentLatitudeDeg - 2.0 * elongationDeg) +
      0.011 * sinDeg(lunarMeanAnomalyDeg - 4.0 * elongationDeg);
  lunarLatitudeDeg +=
      -0.173 * sinDeg(argumentLatitudeDeg - 2.0 * elongationDeg) -
      0.055 * sinDeg(lunarMeanAnomalyDeg - argumentLatitudeDeg -
                     2.0 * elongationDeg) -
      0.046 * sinDeg(lunarMeanAnomalyDeg + argumentLatitudeDeg -
                     2.0 * elongationDeg) +
      0.033 * sinDeg(argumentLatitudeDeg + 2.0 * elongationDeg) +
      0.017 * sinDeg(2.0 * lunarMeanAnomalyDeg + argumentLatitudeDeg);
  lunarLongitudeDeg = normalizeDegreesDouble(lunarLongitudeDeg);

  const OrbitalElements mercury{
      48.3313 + 3.24587e-5 * days,
      7.0047 + 5.00e-8 * days,
      29.1241 + 1.01444e-5 * days,
      0.387098,
      0.205635 + 5.59e-10 * days,
      168.6562 + 4.0923344368 * days,
  };
  const OrbitalElements venus{
      76.6799 + 2.46590e-5 * days,
      3.3946 + 2.75e-8 * days,
      54.8910 + 1.38374e-5 * days,
      0.723330,
      0.006773 - 1.302e-9 * days,
      48.0052 + 1.6021302244 * days,
  };
  const OrbitalElements mars{
      49.5574 + 2.11081e-5 * days,
      1.8497 - 1.78e-8 * days,
      286.5016 + 2.92961e-5 * days,
      1.523688,
      0.093405 + 2.516e-9 * days,
      18.6021 + 0.5240207766 * days,
  };
  const OrbitalElements jupiter{
      100.4542 + 2.76854e-5 * days,
      1.3030 - 1.557e-7 * days,
      273.8777 + 1.64505e-5 * days,
      5.20256,
      0.048498 + 4.469e-9 * days,
      19.8950 + 0.0830853001 * days,
  };
  const OrbitalElements saturn{
      113.6634 + 2.38980e-5 * days,
      2.4886 - 1.081e-7 * days,
      339.3939 + 2.97661e-5 * days,
      9.55475,
      0.055546 - 9.499e-9 * days,
      316.9670 + 0.0334442282 * days,
  };

  EclipticPosition planetPositions[kNakedEyePlanetCount] = {
      heliocentricPosition(mercury), heliocentricPosition(venus),
      heliocentricPosition(mars), heliocentricPosition(jupiter),
      heliocentricPosition(saturn)};

  const double jupiterMeanAnomalyDeg =
      normalizeDegreesDouble(jupiter.meanAnomalyDeg);
  const double saturnMeanAnomalyDeg =
      normalizeDegreesDouble(saturn.meanAnomalyDeg);
  const double jupiterLongitudeCorrectionDeg =
      -0.332 * sinDeg(2.0 * jupiterMeanAnomalyDeg -
                      5.0 * saturnMeanAnomalyDeg - 67.6) -
      0.056 * sinDeg(2.0 * jupiterMeanAnomalyDeg -
                     2.0 * saturnMeanAnomalyDeg + 21.0) +
      0.042 * sinDeg(3.0 * jupiterMeanAnomalyDeg -
                     5.0 * saturnMeanAnomalyDeg + 21.0) -
      0.036 * sinDeg(jupiterMeanAnomalyDeg -
                     2.0 * saturnMeanAnomalyDeg) +
      0.022 * cosDeg(jupiterMeanAnomalyDeg - saturnMeanAnomalyDeg) +
      0.023 * sinDeg(2.0 * jupiterMeanAnomalyDeg -
                     3.0 * saturnMeanAnomalyDeg + 52.0) -
      0.016 * sinDeg(jupiterMeanAnomalyDeg -
                     5.0 * saturnMeanAnomalyDeg - 69.0);
  applyEclipticCorrections(
      planetPositions[static_cast<std::size_t>(PlanetKind::Jupiter)],
      jupiterLongitudeCorrectionDeg, 0.0);

  const double saturnLongitudeCorrectionDeg =
      0.812 * sinDeg(2.0 * jupiterMeanAnomalyDeg -
                     5.0 * saturnMeanAnomalyDeg - 67.6) -
      0.229 * cosDeg(2.0 * jupiterMeanAnomalyDeg -
                     4.0 * saturnMeanAnomalyDeg - 2.0) +
      0.119 * sinDeg(jupiterMeanAnomalyDeg -
                     2.0 * saturnMeanAnomalyDeg - 3.0) +
      0.046 * sinDeg(2.0 * jupiterMeanAnomalyDeg -
                     6.0 * saturnMeanAnomalyDeg - 69.0) +
      0.014 * sinDeg(jupiterMeanAnomalyDeg -
                     3.0 * saturnMeanAnomalyDeg + 32.0);
  const double saturnLatitudeCorrectionDeg =
      -0.020 * cosDeg(2.0 * jupiterMeanAnomalyDeg -
                      4.0 * saturnMeanAnomalyDeg - 2.0) +
      0.018 * sinDeg(2.0 * jupiterMeanAnomalyDeg -
                     6.0 * saturnMeanAnomalyDeg - 49.0);
  applyEclipticCorrections(
      planetPositions[static_cast<std::size_t>(PlanetKind::Saturn)],
      saturnLongitudeCorrectionDeg, saturnLatitudeCorrectionDeg);

  SolarSystemEphemeris result;
  result.sun = eclipticToEquatorial(sunLongitudeDeg, 0.0, julianDate);
  result.moon =
      eclipticToEquatorial(lunarLongitudeDeg, lunarLatitudeDeg, julianDate);
  const double cosineElongation =
      cosDeg(lunarLatitudeDeg) * cosDeg(lunarLongitudeDeg - sunLongitudeDeg);
  result.moonIlluminatedFraction = static_cast<float>(
      clamp((1.0 - cosineElongation) * 0.5, 0.0, 1.0));
  for (std::size_t i = 0; i < kNakedEyePlanetCount; ++i) {
    const EclipticPosition& heliocentric = planetPositions[i];
    const double geocentricX = heliocentric.x + sunGeocentricX;
    const double geocentricY = heliocentric.y + sunGeocentricY;
    const double geocentricZ = heliocentric.z;
    const double geocentricDistance = std::sqrt(
        geocentricX * geocentricX + geocentricY * geocentricY +
        geocentricZ * geocentricZ);
    const double geocentricLongitudeDeg = normalizeDegreesDouble(
        std::atan2(geocentricY, geocentricX) * kRadToDeg);
    const double geocentricLatitudeDeg =
        std::atan2(geocentricZ,
                   std::sqrt(geocentricX * geocentricX +
                             geocentricY * geocentricY)) *
        kRadToDeg;
    result.planets[i].kind = static_cast<PlanetKind>(i);
    result.planets[i].equatorial = eclipticToEquatorial(
        geocentricLongitudeDeg, geocentricLatitudeDeg, julianDate);
    result.planets[i].distanceAu =
        static_cast<float>(geocentricDistance);
  }
  return result;
}

Vec3 equatorialToWorld(double rightAscensionDeg, double declinationDeg,
                       double julianDate, const Location& location) {
  const double hourAngle =
      (localSiderealDeg(julianDate, location.longitudeDeg) -
       rightAscensionDeg) *
      kDegToRad;
  const double declination = declinationDeg * kDegToRad;
  const double latitude = location.latitudeDeg * kDegToRad;

  const double cosDeclination = std::cos(declination);
  const double east = -cosDeclination * std::sin(hourAngle);
  const double north = std::sin(declination) * std::cos(latitude) -
                       cosDeclination * std::cos(hourAngle) *
                           std::sin(latitude);
  const double up = std::sin(declination) * std::sin(latitude) +
                    cosDeclination * std::cos(hourAngle) *
                        std::cos(latitude);

  return {static_cast<float>(east), static_cast<float>(north),
          static_cast<float>(up)};
}

HorizontalCoordinate equatorialToHorizontal(double rightAscensionDeg,
                                             double declinationDeg,
                                             double julianDate,
                                             const Location& location) {
  const Vec3 world = equatorialToWorld(rightAscensionDeg, declinationDeg,
                                       julianDate, location);

  HorizontalCoordinate result;
  result.azimuthDeg = normalizeDegrees(
      static_cast<float>(std::atan2(world.x, world.y) * kRadToDeg));
  result.altitudeDeg =
      static_cast<float>(std::asin(clamp(world.z, -1.0, 1.0)) * kRadToDeg);
  return result;
}

float angularSeparationDeg(float azimuthA, float altitudeA, float azimuthB,
                           float altitudeB) {
  const double altA = altitudeA * kDegToRad;
  const double altB = altitudeB * kDegToRad;
  const double deltaAz = (azimuthA - azimuthB) * kDegToRad;
  const double cosine = std::sin(altA) * std::sin(altB) +
                        std::cos(altA) * std::cos(altB) * std::cos(deltaAz);
  return static_cast<float>(std::acos(clamp(cosine, -1.0, 1.0)) * kRadToDeg);
}

Rgb8 colorFromBv(float bv) {
  // Start with a B-V temperature conversion and black-body approximation, then
  // deliberately exaggerate the distance from the brightest channel. Real
  // stars are subtler, but the stronger chroma survives the small AMOLED pixels
  // and makes spectral differences useful through the viewing tube.
  bv = std::max(-0.4F, std::min(2.0F, bv));
  const double temperature =
      4600.0 * (1.0 / (0.92 * bv + 1.7) + 1.0 / (0.92 * bv + 0.62));
  const double t = temperature / 100.0;
  double red;
  double green;
  double blue;
  if (t <= 66.0) {
    red = 255.0;
    green = 99.4708025861 * std::log(t) - 161.1195681661;
    blue = t <= 19.0 ? 0.0
                     : 138.5177312231 * std::log(t - 10.0) - 305.0447927307;
  } else {
    red = 329.698727446 * std::pow(t - 60.0, -0.1332047592);
    green = 288.1221695283 * std::pow(t - 60.0, -0.0755148492);
    blue = 255.0;
  }
  red = clamp(red, 0.0, 255.0);
  green = clamp(green, 0.0, 255.0);
  blue = clamp(blue, 0.0, 255.0);
  const double brightest = std::max(red, std::max(green, blue));
  const auto channel = [brightest](double value) {
    constexpr double kColorContrast = 1.8;
    return static_cast<std::uint8_t>(clamp(
        brightest - (brightest - value) * kColorContrast, 0.0, 255.0));
  };
  return {channel(red), channel(green), channel(blue)};
}

}  // namespace starscope::astronomy
