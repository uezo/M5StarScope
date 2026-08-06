#pragma once

#include "starscope/types.hpp"

namespace starscope::astronomy {

double julianDateUtc(const CivilTime& localTime, int utcOffsetMinutes);
CivilTime civilTimeUtc(double julianDate);
double greenwichMeanSiderealDeg(double julianDate);
double localSiderealDeg(double julianDate, double longitudeDeg);

// Lightweight geocentric ephemeris suitable for the 45-degree scope view.
// The Sun and naked-eye planets are accurate to a small fraction of a degree
// for this use. Lunar perturbation terms keep the Moon to roughly a degree
// without a large external library.
SolarSystemEphemeris solarSystemEphemeris(double julianDate);

HorizontalCoordinate equatorialToHorizontal(double rightAscensionDeg,
                                             double declinationDeg,
                                             double julianDate,
                                             const Location& location);

// Returns the same apparent direction directly in the local east/north/up
// frame. This avoids converting through azimuth/altitude when a renderer only
// needs a unit vector.
Vec3 equatorialToWorld(double rightAscensionDeg, double declinationDeg,
                       double julianDate, const Location& location);

float angularSeparationDeg(float azimuthA, float altitudeA, float azimuthB,
                           float altitudeB);

Rgb8 colorFromBv(float bv);
float normalizeDegrees(float value);
float signedAngleDifference(float targetDeg, float referenceDeg);

}  // namespace starscope::astronomy
