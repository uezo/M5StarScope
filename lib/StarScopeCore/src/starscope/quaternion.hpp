#pragma once

#include "starscope/types.hpp"

namespace starscope::quaternion {

float dot(const Vec3& a, const Vec3& b);
Vec3 cross(const Vec3& a, const Vec3& b);
float length(const Vec3& value);
Vec3 normalized(const Vec3& value);

Quaternion normalized(const Quaternion& value);
Quaternion conjugate(const Quaternion& value);
Quaternion multiply(const Quaternion& a, const Quaternion& b);
Quaternion nlerp(const Quaternion& from, const Quaternion& to, float amount);
Quaternion fromEulerDeg(float yawDeg, float pitchDeg, float rollDeg);
Quaternion fromPointingDeg(float azimuthDeg, float altitudeDeg,
                           float rollDeg = 0.0F);
Quaternion fromRotationRows(const Vec3& row0, const Vec3& row1,
                            const Vec3& row2);
Vec3 rotate(const Quaternion& rotation, const Vec3& value);

// Aerospace-style diagnostic angles for a sensor whose +X axis is forward,
// +Y is right, and +Z is up. Rendering uses the quaternion directly.
void toEulerDeg(const Quaternion& sensorToWorld, float& yawDeg,
                float& pitchDeg, float& rollDeg);

}  // namespace starscope::quaternion
