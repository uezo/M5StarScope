#include "starscope/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "starscope/quaternion.hpp"

namespace starscope::protocol {

std::uint16_t crc16Ccitt(const std::uint8_t* data, std::size_t length) {
  std::uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < length; ++i) {
    crc ^= static_cast<std::uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                            : static_cast<std::uint16_t>(crc << 1U);
    }
  }
  return crc;
}

void finalize(OrientationPacket& packet) {
  packet.magic = kMagic;
  packet.version = kVersion;
  packet.crc = 0;
  packet.crc = crc16Ccitt(reinterpret_cast<const std::uint8_t*>(&packet),
                         sizeof(packet) - sizeof(packet.crc));
}

bool isValid(const OrientationPacket& packet) {
  if (packet.magic != kMagic || packet.version != kVersion) return false;
  return packet.crc ==
         crc16Ccitt(reinterpret_cast<const std::uint8_t*>(&packet),
                    sizeof(packet) - sizeof(packet.crc));
}

OrientationSample toSample(const OrientationPacket& packet,
                           std::uint32_t receivedAtMs) {
  OrientationSample sample;
  constexpr float kQuaternionScale = 16384.0F;
  sample.sensorToWorld = quaternion::normalized(
      {packet.quaternionW / kQuaternionScale,
       packet.quaternionX / kQuaternionScale,
       packet.quaternionY / kQuaternionScale,
       packet.quaternionZ / kQuaternionScale});
  quaternion::toEulerDeg(sample.sensorToWorld, sample.yawDeg, sample.pitchDeg,
                         sample.rollDeg);
  sample.magneticMicroTesla = packet.magneticDeciMicroTesla / 10.0F;
  sample.accuracy = packet.accuracyPercent / 100.0F;
  sample.timestampMs = receivedAtMs;
  sample.valid = (packet.flags & kFlagValid) != 0;
  sample.calibrated = (packet.flags & kFlagCalibrated) != 0;
  return sample;
}

bool OrientationPacketParser::push(std::uint8_t byte,
                                   OrientationPacket& completed) {
  const std::uint8_t magicLow = static_cast<std::uint8_t>(kMagic & 0xFFU);
  const std::uint8_t magicHigh = static_cast<std::uint8_t>(kMagic >> 8U);
  if (length_ == 0 && byte != magicLow) return false;
  if (length_ == 1 && byte != magicHigh) {
    length_ = byte == magicLow ? 1 : 0;
    buffer_[0] = byte;
    return false;
  }
  buffer_[length_++] = byte;
  if (length_ != sizeof(OrientationPacket)) return false;

  std::memcpy(&completed, buffer_, sizeof(completed));
  const bool valid = isValid(completed);
  reset();
  return valid;
}

void OrientationPacketParser::reset() { length_ = 0; }

}  // namespace starscope::protocol
