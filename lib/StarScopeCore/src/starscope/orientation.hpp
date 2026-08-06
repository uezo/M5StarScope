#pragma once

#include "starscope/quaternion.hpp"
#include "starscope/types.hpp"

namespace starscope {

class OrientationProvider {
 public:
  virtual ~OrientationProvider() = default;
  virtual void begin() = 0;
  virtual void update(std::uint32_t nowMs) = 0;
  virtual OrientationSample latest() const = 0;
  virtual const char* name() const = 0;
};

class EmulatedOrientationProvider final : public OrientationProvider {
 public:
  void begin() override { sample_.valid = true; }
  void update(std::uint32_t nowMs) override {
    sample_.timestampMs = nowMs;
    sample_.valid = true;
    sample_.calibrated = true;
    sample_.accuracy = 1.0F;
  }
  OrientationSample latest() const override { return sample_; }
  const char* name() const override { return "EMU"; }

  void set(float yawDeg, float pitchDeg, float rollDeg) {
    sample_.yawDeg = yawDeg;
    sample_.pitchDeg = pitchDeg;
    sample_.rollDeg = rollDeg;
    sample_.sensorToWorld =
        quaternion::fromPointingDeg(yawDeg, pitchDeg, rollDeg);
  }

 private:
  OrientationSample sample_;
};

}  // namespace starscope
