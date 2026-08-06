#include "starscope/fusion.hpp"

#include <algorithm>
#include <cmath>

#include "starscope/quaternion.hpp"

namespace starscope {
namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kDegToRad = kPi / 180.0F;

}  // namespace

void PoseFusion::reset() {
  sensorToWorld_ = {};
  previousTimestampUs_ = 0;
  valid_ = false;
}

bool PoseFusion::absoluteReference(const Vec3& accelerationG,
                                   const Vec3& magneticMicroTesla,
                                   Quaternion& output) const {
  const float acceleration = quaternion::length(accelerationG);
  const float magnetic = quaternion::length(magneticMicroTesla);
  if (acceleration < 0.70F || acceleration > 1.30F || magnetic < 5.0F ||
      magnetic > 1000.0F) {
    return false;
  }

  // M5Unified reports approximately +1 g along the outward/up axis at rest.
  const Vec3 up = quaternion::normalized(accelerationG);
  const float verticalMagnetic = quaternion::dot(magneticMicroTesla, up);
  const Vec3 horizontalMagnetic{
      magneticMicroTesla.x - up.x * verticalMagnetic,
      magneticMicroTesla.y - up.y * verticalMagnetic,
      magneticMicroTesla.z - up.z * verticalMagnetic};
  const Vec3 north = quaternion::normalized(horizontalMagnetic);
  if (quaternion::length(north) < 0.5F) return false;
  const Vec3 east = quaternion::normalized(quaternion::cross(north, up));

  // Rows of a sensor-to-world ENU rotation matrix are the world axes as seen
  // in sensor coordinates.
  output = quaternion::fromRotationRows(east, north, up);
  return true;
}

bool PoseFusion::update(const Vec3& gyroDegPerSecond,
                        const Vec3& accelerationG,
                        const Vec3& magneticMicroTesla,
                        std::uint32_t timestampUs) {
  Quaternion absolute;
  const bool hasAbsolute =
      absoluteReference(accelerationG, magneticMicroTesla, absolute);

  if (!valid_) {
    previousTimestampUs_ = timestampUs;
    if (!hasAbsolute) return false;
    sensorToWorld_ = absolute;
    valid_ = true;
    return true;
  }

  const std::uint32_t elapsedUs = timestampUs - previousTimestampUs_;
  previousTimestampUs_ = timestampUs;
  const float dt = std::min(0.1F, elapsedUs * 1.0e-6F);
  if (dt <= 0.0F) return valid_;

  const Quaternion angularVelocity{0.0F, gyroDegPerSecond.x * kDegToRad,
                                   gyroDegPerSecond.y * kDegToRad,
                                   gyroDegPerSecond.z * kDegToRad};
  const Quaternion derivative =
      quaternion::multiply(sensorToWorld_, angularVelocity);
  sensorToWorld_ = quaternion::normalized(
      {sensorToWorld_.w + derivative.w * 0.5F * dt,
       sensorToWorld_.x + derivative.x * 0.5F * dt,
       sensorToWorld_.y + derivative.y * 0.5F * dt,
       sensorToWorld_.z + derivative.z * 0.5F * dt});

  if (hasAbsolute) {
    // About 0.65 s time constant: responsive enough for a hand-held tube but
    // still much less jittery than raw magnetometer Euler angles.
    const float correction = 1.0F - std::exp(-dt / 0.65F);
    sensorToWorld_ =
        quaternion::nlerp(sensorToWorld_, absolute, correction);
  }
  return valid_;
}

}  // namespace starscope
