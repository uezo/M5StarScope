#pragma once

#include <cstddef>
#include <cstdint>

#include "starscope/types.hpp"

namespace starscope::protocol {

constexpr std::uint16_t kMagic = 0x5353;  // "SS" on the wire, little endian.
constexpr std::uint8_t kVersion = 2;
constexpr std::uint8_t kFlagValid = 1U << 0;
constexpr std::uint8_t kFlagCalibrated = 1U << 1;
constexpr std::uint8_t kFlagCalibrating = 1U << 2;
constexpr std::uint8_t kFlagSetupToggle = 1U << 3;

#pragma pack(push, 1)
struct OrientationPacket {
  std::uint16_t magic = kMagic;
  std::uint8_t version = kVersion;
  std::uint8_t flags = 0;
  std::uint16_t sequence = 0;
  std::uint32_t timestampMs = 0;
  std::int16_t quaternionW = 16384;
  std::int16_t quaternionX = 0;
  std::int16_t quaternionY = 0;
  std::int16_t quaternionZ = 0;
  std::uint16_t magneticDeciMicroTesla = 0;
  std::uint8_t accuracyPercent = 0;
  std::uint8_t reserved = 0;
  std::uint16_t crc = 0;
};
#pragma pack(pop)

static_assert(sizeof(OrientationPacket) == 24,
              "UART packet layout must remain stable");

std::uint16_t crc16Ccitt(const std::uint8_t* data, std::size_t length);
void finalize(OrientationPacket& packet);
bool isValid(const OrientationPacket& packet);
OrientationSample toSample(const OrientationPacket& packet,
                           std::uint32_t receivedAtMs);

class OrientationPacketParser {
 public:
  bool push(std::uint8_t byte, OrientationPacket& completed);
  void reset();

 private:
  std::uint8_t buffer_[sizeof(OrientationPacket)]{};
  std::size_t length_ = 0;
};

}  // namespace starscope::protocol
