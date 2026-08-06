#include "starscope/scope_projection.hpp"

#include <algorithm>
#include <cmath>

#include "starscope/quaternion.hpp"

namespace starscope {
namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kDegToRad = kPi / 180.0F;
constexpr float kRadToDeg = 180.0F / kPi;

}  // namespace

Quaternion applyWorldHeadingCorrection(const Quaternion& sensorToWorld,
                                       float correctionDeg) {
  // ENU azimuth grows clockwise when viewed from above, opposite to the
  // mathematical positive rotation around +Z.
  const float halfAngle = -correctionDeg * kDegToRad * 0.5F;
  const Quaternion correction{std::cos(halfAngle), 0.0F, 0.0F,
                              std::sin(halfAngle)};
  return quaternion::normalized(
      quaternion::multiply(correction, sensorToWorld));
}

ScopeBasis scopeBasis(const Quaternion& sensorToWorld,
                      const Quaternion& tubeToSensor) {
  const Quaternion tubeToWorld = quaternion::multiply(
      quaternion::normalized(sensorToWorld),
      quaternion::normalized(tubeToSensor));
  return {quaternion::rotate(tubeToWorld, {1.0F, 0.0F, 0.0F}),
          quaternion::rotate(tubeToWorld, {0.0F, -1.0F, 0.0F}),
          quaternion::rotate(tubeToWorld, {0.0F, 0.0F, 1.0F})};
}

Vec3 horizontalToWorld(const HorizontalCoordinate& horizontal) {
  const float azimuth = horizontal.azimuthDeg * kDegToRad;
  const float altitude = horizontal.altitudeDeg * kDegToRad;
  const float horizontalRadius = std::cos(altitude);
  return {horizontalRadius * std::sin(azimuth),
          horizontalRadius * std::cos(azimuth), std::sin(altitude)};
}

ProjectedPoint projectIntoScope(const Vec3& worldDirection,
                                const ScopeBasis& scope, float fieldOfViewDeg,
                                bool circular) {
  ProjectedPoint result;
  const Vec3 direction = quaternion::normalized(worldDirection);
  const float forward = quaternion::dot(direction, scope.forward);
  result.angularDistanceDeg =
      std::acos(std::max(-1.0F, std::min(1.0F, forward))) * kRadToDeg;
  if (forward <= 0.0F || fieldOfViewDeg <= 1.0F || fieldOfViewDeg >= 179.0F) {
    return result;
  }
  const float tangent = std::tan(fieldOfViewDeg * kDegToRad * 0.5F);
  result.normalizedX = quaternion::dot(direction, scope.right) /
                       (forward * tangent);
  result.normalizedY = quaternion::dot(direction, scope.up) /
                       (forward * tangent);
  const float radiusSquared = result.normalizedX * result.normalizedX +
                              result.normalizedY * result.normalizedY;
  result.visible = circular ? radiusSquared <= 1.0F
                            : std::fabs(result.normalizedX) <= 1.0F &&
                                  std::fabs(result.normalizedY) <= 1.0F;
  return result;
}

}  // namespace starscope
