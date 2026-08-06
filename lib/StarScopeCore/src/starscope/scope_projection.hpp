#pragma once

#include "starscope/types.hpp"

namespace starscope {

struct ScopeBasis {
  Vec3 forward;
  Vec3 right;
  Vec3 up;
};

struct ProjectedPoint {
  float normalizedX = 0.0F;
  float normalizedY = 0.0F;
  float angularDistanceDeg = 180.0F;
  bool visible = false;
};

Quaternion applyWorldHeadingCorrection(const Quaternion& sensorToWorld,
                                       float correctionDeg);
ScopeBasis scopeBasis(const Quaternion& sensorToWorld,
                      const Quaternion& tubeToSensor = {});
Vec3 horizontalToWorld(const HorizontalCoordinate& horizontal);
ProjectedPoint projectIntoScope(const Vec3& worldDirection,
                                const ScopeBasis& scope, float fieldOfViewDeg,
                                bool circular = true);

}  // namespace starscope
