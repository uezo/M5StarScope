#include <Arduino.h>
#include <M5Unified.h>
#include <Preferences.h>

#include <algorithm>
#include <cmath>

#include "starscope/fusion.hpp"
#include "starscope/protocol.hpp"
#include "starscope/quaternion.hpp"
#include "starscope/types.hpp"

namespace {

using starscope::OrientationSample;
using starscope::PoseFusion;
using starscope::Quaternion;
using starscope::Vec3;
using starscope::protocol::OrientationPacket;

constexpr int kUartRxPin = 2;
constexpr int kUartTxPin = 1;
constexpr std::uint32_t kUartBaud = 115200;
constexpr std::uint32_t kTransmitIntervalMs = 20;
constexpr std::uint32_t kDisplayIntervalMs = 100;
constexpr std::uint32_t kCalibrationDurationMs = 20000;
constexpr std::uint32_t kSetupHoldMs = 2000;
constexpr std::uint32_t kSetupRequestPulseMs = 350;
constexpr float kQuaternionScale = 16384.0F;

HardwareSerial orientationSerial(1);
Preferences preferences;

struct CalibrationData {
  Vec3 minimum;
  Vec3 maximum;
  bool valid = false;
};

class MagnetometerCalibration {
 public:
  void begin() {
    preferences.begin("scope-mag", false);
    data_.minimum.x = preferences.getFloat("minx", 0.0F);
    data_.minimum.y = preferences.getFloat("miny", 0.0F);
    data_.minimum.z = preferences.getFloat("minz", 0.0F);
    data_.maximum.x = preferences.getFloat("maxx", 0.0F);
    data_.maximum.y = preferences.getFloat("maxy", 0.0F);
    data_.maximum.z = preferences.getFloat("maxz", 0.0F);
    data_.valid = preferences.getBool("valid", false) && hasEnoughRange();
  }

  void start(std::uint32_t nowMs) {
    data_.minimum = {100000.0F, 100000.0F, 100000.0F};
    data_.maximum = {-100000.0F, -100000.0F, -100000.0F};
    startedAtMs_ = nowMs;
    calibrating_ = true;
    data_.valid = false;
  }

  void add(const Vec3& field, std::uint32_t nowMs) {
    if (!calibrating_) return;
    data_.minimum.x = std::min(data_.minimum.x, field.x);
    data_.minimum.y = std::min(data_.minimum.y, field.y);
    data_.minimum.z = std::min(data_.minimum.z, field.z);
    data_.maximum.x = std::max(data_.maximum.x, field.x);
    data_.maximum.y = std::max(data_.maximum.y, field.y);
    data_.maximum.z = std::max(data_.maximum.z, field.z);
    if (nowMs - startedAtMs_ >= kCalibrationDurationMs) finish();
  }

  Vec3 apply(const Vec3& field) const {
    if (!data_.valid) return field;
    const Vec3 offset{(data_.minimum.x + data_.maximum.x) * 0.5F,
                      (data_.minimum.y + data_.maximum.y) * 0.5F,
                      (data_.minimum.z + data_.maximum.z) * 0.5F};
    const Vec3 radius{(data_.maximum.x - data_.minimum.x) * 0.5F,
                      (data_.maximum.y - data_.minimum.y) * 0.5F,
                      (data_.maximum.z - data_.minimum.z) * 0.5F};
    const float average = (radius.x + radius.y + radius.z) / 3.0F;
    return {(field.x - offset.x) * average / radius.x,
            (field.y - offset.y) * average / radius.y,
            (field.z - offset.z) * average / radius.z};
  }

  bool calibrating() const { return calibrating_; }
  bool valid() const { return data_.valid; }

  float progress(std::uint32_t nowMs) const {
    if (!calibrating_) return data_.valid ? 1.0F : 0.0F;
    return std::min(1.0F,
                    (nowMs - startedAtMs_) /
                        static_cast<float>(kCalibrationDurationMs));
  }

  float accuracy() const {
    if (!data_.valid) return 0.0F;
    const float x = data_.maximum.x - data_.minimum.x;
    const float y = data_.maximum.y - data_.minimum.y;
    const float z = data_.maximum.z - data_.minimum.z;
    const float minimumSpan = std::min(x, std::min(y, z));
    const float maximumSpan = std::max(x, std::max(y, z));
    const float coverage = std::min(1.0F, minimumSpan / 60.0F);
    const float symmetry = maximumSpan > 0.0F ? minimumSpan / maximumSpan : 0.0F;
    return std::max(0.0F, std::min(1.0F, coverage * symmetry));
  }

 private:
  bool hasEnoughRange() const {
    return data_.maximum.x - data_.minimum.x > 15.0F &&
           data_.maximum.y - data_.minimum.y > 15.0F &&
           data_.maximum.z - data_.minimum.z > 15.0F;
  }

  void finish() {
    calibrating_ = false;
    data_.valid = hasEnoughRange();
    preferences.putBool("valid", data_.valid);
    if (!data_.valid) return;
    preferences.putFloat("minx", data_.minimum.x);
    preferences.putFloat("miny", data_.minimum.y);
    preferences.putFloat("minz", data_.minimum.z);
    preferences.putFloat("maxx", data_.maximum.x);
    preferences.putFloat("maxy", data_.maximum.y);
    preferences.putFloat("maxz", data_.maximum.z);
  }

  CalibrationData data_;
  bool calibrating_ = false;
  std::uint32_t startedAtMs_ = 0;
};

MagnetometerCalibration calibration;
PoseFusion fusion;
OrientationSample latest;
std::uint16_t sequence = 0;
std::uint32_t lastTransmitMs = 0;
std::uint32_t lastDisplayMs = 0;
bool imuAvailable = false;
bool setupLongPressHandled = false;
bool setupRequestActive = false;
std::uint32_t setupRequestStartedMs = 0;

bool setupRequestPending(std::uint32_t nowMs) {
  if (setupRequestActive &&
      nowMs - setupRequestStartedMs >= kSetupRequestPulseMs) {
    setupRequestActive = false;
  }
  return setupRequestActive;
}

std::int16_t encodeQuaternion(float value) {
  return static_cast<std::int16_t>(std::lround(
      std::max(-1.999F, std::min(1.999F, value)) * kQuaternionScale));
}

void transmit(const OrientationSample& sample) {
  OrientationPacket packet;
  packet.flags = (sample.valid ? starscope::protocol::kFlagValid : 0) |
                 (sample.calibrated ? starscope::protocol::kFlagCalibrated : 0) |
                 (calibration.calibrating()
                      ? starscope::protocol::kFlagCalibrating
                      : 0) |
                 (setupRequestPending(millis())
                      ? starscope::protocol::kFlagSetupToggle
                      : 0);
  packet.sequence = sequence++;
  packet.timestampMs = sample.timestampMs;
  packet.quaternionW = encodeQuaternion(sample.sensorToWorld.w);
  packet.quaternionX = encodeQuaternion(sample.sensorToWorld.x);
  packet.quaternionY = encodeQuaternion(sample.sensorToWorld.y);
  packet.quaternionZ = encodeQuaternion(sample.sensorToWorld.z);
  packet.magneticDeciMicroTesla = static_cast<std::uint16_t>(std::lround(
      std::max(0.0F, std::min(6553.5F, sample.magneticMicroTesla)) * 10.0F));
  packet.accuracyPercent = static_cast<std::uint8_t>(std::lround(
      std::max(0.0F, std::min(1.0F, sample.accuracy)) * 100.0F));
  starscope::protocol::finalize(packet);
  orientationSerial.write(reinterpret_cast<const std::uint8_t*>(&packet),
                          sizeof(packet));
}

void drawStatus(std::uint32_t nowMs) {
  auto& display = M5.Display;
  display.startWrite();
  display.fillScreen(TFT_BLACK);
  display.setTextDatum(top_center);
  display.setTextColor(latest.valid ? TFT_GREEN : TFT_RED, TFT_BLACK);
  display.drawString(setupRequestPending(nowMs)
                         ? "SETUP TOGGLE"
                         : calibration.calibrating() ? "ROTATE ALL AXES"
                                                     : "M5 STARSCOPE",
                     64, 3, 2);

  display.setTextDatum(top_left);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.setCursor(7, 24);
  display.printf("AZ %6.1f\n", latest.yawDeg);
  display.printf("AL %6.1f\n", latest.pitchDeg);
  display.printf("RO %6.1f\n", latest.rollDeg);
  display.printf("B  %5.1f uT\n", latest.magneticMicroTesla);
  display.printf("Q  %3d%%", static_cast<int>(latest.accuracy * 100.0F));

  const int width = static_cast<int>(112.0F * calibration.progress(nowMs));
  display.drawRect(8, 112, 112, 8, TFT_DARKGREY);
  display.fillRect(9, 113, std::max(0, width - 2), 6,
                   calibration.valid() ? TFT_GREEN : TFT_ORANGE);
  display.setTextDatum(bottom_center);
  display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  display.drawString("TAP RECAL / HOLD SETUP", 64, 127, 1);
  display.endWrite();
}

}  // namespace

void setup() {
  auto config = M5.config();
  config.clear_display = true;
  config.output_power = false;
  M5.begin(config);
  M5.Display.setRotation(0);
  M5.Display.setBrightness(80);
  M5.BtnA.setHoldThresh(kSetupHoldMs);
  orientationSerial.begin(kUartBaud, SERIAL_8N1, kUartRxPin, kUartTxPin);
  calibration.begin();
  imuAvailable = M5.Imu.begin();
  if (!calibration.valid()) calibration.start(millis());
}

void loop() {
  M5.update();
  const std::uint32_t nowMs = millis();

  if (M5.BtnA.wasPressed()) setupLongPressHandled = false;

  if (!setupLongPressHandled && M5.BtnA.pressedFor(kSetupHoldMs)) {
    setupLongPressHandled = true;
    setupRequestActive = true;
    setupRequestStartedMs = nowMs;
  }

  if (M5.BtnA.wasReleased() && !setupLongPressHandled) {
    calibration.start(nowMs);
    fusion.reset();
  }

  if (M5.Imu.update()) {
    const auto data = M5.Imu.getImuData();
    const Vec3 acceleration{data.accel.x, data.accel.y, data.accel.z};
    const Vec3 gyro{data.gyro.x, data.gyro.y, data.gyro.z};
    const Vec3 rawMagnetic{data.mag.x, data.mag.y, data.mag.z};
    calibration.add(rawMagnetic, nowMs);
    const Vec3 magnetic = calibration.apply(rawMagnetic);
    latest.valid = imuAvailable &&
                   fusion.update(gyro, acceleration, magnetic, micros());
    latest.sensorToWorld = fusion.sensorToMagneticWorld();
    starscope::quaternion::toEulerDeg(latest.sensorToWorld, latest.yawDeg,
                                     latest.pitchDeg, latest.rollDeg);
    latest.magneticMicroTesla = starscope::quaternion::length(magnetic);
    latest.accuracy = calibration.accuracy();
    latest.timestampMs = nowMs;
    latest.calibrated = calibration.valid();
  }

  if (nowMs - lastTransmitMs >= kTransmitIntervalMs) {
    lastTransmitMs = nowMs;
    transmit(latest);
  }
  if (nowMs - lastDisplayMs >= kDisplayIntervalMs) {
    lastDisplayMs = nowMs;
    drawStatus(nowMs);
  }
  delay(2);
}
