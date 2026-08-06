#include "starscope/quaternion.hpp"

#include <algorithm>
#include <cmath>

#include "starscope/astronomy.hpp"

namespace starscope::quaternion {
namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kDegToRad = kPi / 180.0F;
constexpr float kRadToDeg = 180.0F / kPi;

float clamp(float value, float low, float high) {
  return std::max(low, std::min(value, high));
}

}  // namespace

float dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}

float length(const Vec3& value) { return std::sqrt(dot(value, value)); }

Vec3 normalized(const Vec3& value) {
  const float size = length(value);
  if (size < 1.0e-6F) return {};
  return {value.x / size, value.y / size, value.z / size};
}

Quaternion normalized(const Quaternion& value) {
  const float size = std::sqrt(value.w * value.w + value.x * value.x +
                               value.y * value.y + value.z * value.z);
  if (size < 1.0e-6F) return {};
  return {value.w / size, value.x / size, value.y / size, value.z / size};
}

Quaternion conjugate(const Quaternion& value) {
  return {value.w, -value.x, -value.y, -value.z};
}

Quaternion multiply(const Quaternion& a, const Quaternion& b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
          a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

Quaternion nlerp(const Quaternion& from, const Quaternion& to, float amount) {
  amount = clamp(amount, 0.0F, 1.0F);
  float sign = from.w * to.w + from.x * to.x + from.y * to.y +
                       from.z * to.z <
                   0.0F
                   ? -1.0F
                   : 1.0F;
  return normalized({from.w + (sign * to.w - from.w) * amount,
                     from.x + (sign * to.x - from.x) * amount,
                     from.y + (sign * to.y - from.y) * amount,
                     from.z + (sign * to.z - from.z) * amount});
}

Quaternion fromEulerDeg(float yawDeg, float pitchDeg, float rollDeg) {
  const float yaw = yawDeg * kDegToRad * 0.5F;
  const float pitch = pitchDeg * kDegToRad * 0.5F;
  const float roll = rollDeg * kDegToRad * 0.5F;
  const float cy = std::cos(yaw);
  const float sy = std::sin(yaw);
  const float cp = std::cos(pitch);
  const float sp = std::sin(pitch);
  const float cr = std::cos(roll);
  const float sr = std::sin(roll);
  return normalized({cr * cp * cy + sr * sp * sy,
                     sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy});
}

Quaternion fromPointingDeg(float azimuthDeg, float altitudeDeg,
                           float rollDeg) {
  const float azimuth = azimuthDeg * kDegToRad;
  const float altitude = altitudeDeg * kDegToRad;
  const float roll = rollDeg * kDegToRad;
  const Vec3 forward{std::sin(azimuth) * std::cos(altitude),
                     std::cos(azimuth) * std::cos(altitude),
                     std::sin(altitude)};
  const Vec3 levelRight{std::cos(azimuth), -std::sin(azimuth), 0.0F};
  const Vec3 levelUp = cross(levelRight, forward);
  const Vec3 right{levelRight.x * std::cos(roll) + levelUp.x * std::sin(roll),
                   levelRight.y * std::cos(roll) + levelUp.y * std::sin(roll),
                   levelRight.z * std::cos(roll) + levelUp.z * std::sin(roll)};
  const Vec3 left{-right.x, -right.y, -right.z};
  const Vec3 up = cross(forward, left);
  // Columns are the local +X/+Y/+Z axes expressed in world ENU.
  return fromRotationRows({forward.x, left.x, up.x},
                          {forward.y, left.y, up.y},
                          {forward.z, left.z, up.z});
}

Quaternion fromRotationRows(const Vec3& row0, const Vec3& row1,
                            const Vec3& row2) {
  const float trace = row0.x + row1.y + row2.z;
  Quaternion result;
  if (trace > 0.0F) {
    const float s = std::sqrt(trace + 1.0F) * 2.0F;
    result = {0.25F * s, (row2.y - row1.z) / s,
              (row0.z - row2.x) / s, (row1.x - row0.y) / s};
  } else if (row0.x > row1.y && row0.x > row2.z) {
    const float s = std::sqrt(1.0F + row0.x - row1.y - row2.z) * 2.0F;
    result = {(row2.y - row1.z) / s, 0.25F * s,
              (row0.y + row1.x) / s, (row0.z + row2.x) / s};
  } else if (row1.y > row2.z) {
    const float s = std::sqrt(1.0F + row1.y - row0.x - row2.z) * 2.0F;
    result = {(row0.z - row2.x) / s, (row0.y + row1.x) / s,
              0.25F * s, (row1.z + row2.y) / s};
  } else {
    const float s = std::sqrt(1.0F + row2.z - row0.x - row1.y) * 2.0F;
    result = {(row1.x - row0.y) / s, (row0.z + row2.x) / s,
              (row1.z + row2.y) / s, 0.25F * s};
  }
  return normalized(result);
}

Vec3 rotate(const Quaternion& rotation, const Vec3& value) {
  const Quaternion q = normalized(rotation);
  const Quaternion vector{0.0F, value.x, value.y, value.z};
  const Quaternion result = multiply(multiply(q, vector), conjugate(q));
  return {result.x, result.y, result.z};
}

void toEulerDeg(const Quaternion& sensorToWorld, float& yawDeg,
                float& pitchDeg, float& rollDeg) {
  const Vec3 forward = rotate(sensorToWorld, {1.0F, 0.0F, 0.0F});
  const Vec3 right = rotate(sensorToWorld, {0.0F, -1.0F, 0.0F});
  yawDeg = astronomy::normalizeDegrees(
      std::atan2(forward.x, forward.y) * kRadToDeg);
  pitchDeg = std::asin(clamp(forward.z, -1.0F, 1.0F)) * kRadToDeg;
  const Vec3 levelRight =
      normalized(Vec3{forward.y, -forward.x, 0.0F});
  const Vec3 levelUp = normalized(cross(levelRight, forward));
  rollDeg = std::atan2(dot(right, levelUp), dot(right, levelRight)) * kRadToDeg;
}

}  // namespace starscope::quaternion
