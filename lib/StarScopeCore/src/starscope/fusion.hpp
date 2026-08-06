#pragma once

#include <cstdint>

#include "starscope/types.hpp"

namespace starscope {

// Gyroscope integration with an absolute accelerometer/magnetometer reference.
// It avoids Euler-angle singularities while keeping magnetic heading drift
// bounded. Inputs use degrees/second, g, and microtesla respectively.
class PoseFusion {
 public:
  void reset();
  bool update(const Vec3& gyroDegPerSecond, const Vec3& accelerationG,
              const Vec3& magneticMicroTesla, std::uint32_t timestampUs);

  Quaternion sensorToMagneticWorld() const { return sensorToWorld_; }
  bool valid() const { return valid_; }

 private:
  bool absoluteReference(const Vec3& accelerationG,
                         const Vec3& magneticMicroTesla,
                         Quaternion& output) const;

  Quaternion sensorToWorld_;
  std::uint32_t previousTimestampUs_ = 0;
  bool valid_ = false;
};

}  // namespace starscope
