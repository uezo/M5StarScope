#pragma once

#include <cstddef>
#include <cstdint>

namespace starscope {

constexpr float kInvalidAccuracy = -1.0F;

struct CivilTime {
  int year = 2026;
  int month = 1;
  int day = 1;
  int hour = 0;
  int minute = 0;
  int second = 0;
};

struct Location {
  double latitudeDeg = 35.681236;
  double longitudeDeg = 139.767125;
  int utcOffsetMinutes = 540;
  const char* name = "Tokyo";
};

struct HorizontalCoordinate {
  float azimuthDeg = 0.0F;
  float altitudeDeg = -90.0F;
};

struct EquatorialCoordinate {
  double rightAscensionDeg = 0.0;
  double declinationDeg = 0.0;
};

enum class PlanetKind : std::uint8_t {
  Mercury,
  Venus,
  Mars,
  Jupiter,
  Saturn,
};

constexpr std::size_t kNakedEyePlanetCount = 5;

struct PlanetEphemeris {
  PlanetKind kind = PlanetKind::Mercury;
  EquatorialCoordinate equatorial;
  float distanceAu = 0.0F;
};

struct SolarSystemEphemeris {
  EquatorialCoordinate sun;
  EquatorialCoordinate moon;
  float moonIlluminatedFraction = 0.0F;
  PlanetEphemeris planets[kNakedEyePlanetCount];
};

struct Vec3 {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

// Unit quaternion rotating a vector from sensor coordinates into the local
// east/north/up world frame.
struct Quaternion {
  float w = 1.0F;
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

struct OrientationSample {
  Quaternion sensorToWorld;
  float yawDeg = 0.0F;
  float pitchDeg = 0.0F;
  float rollDeg = 0.0F;
  float magneticMicroTesla = 0.0F;
  float accuracy = kInvalidAccuracy;
  std::uint32_t timestampMs = 0;
  bool valid = false;
  bool calibrated = false;
};

struct StarView {
  std::uint32_t hipId = 0;
  const char* name = nullptr;
  float magnitude = 0.0F;
  float colorIndex = 0.65F;
  HorizontalCoordinate horizontal;
};

struct Rgb8 {
  std::uint8_t red = 255;
  std::uint8_t green = 255;
  std::uint8_t blue = 255;
};

}  // namespace starscope
