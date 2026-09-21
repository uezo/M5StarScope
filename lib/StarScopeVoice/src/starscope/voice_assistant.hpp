#pragma once

#include <cstddef>
#include <cstdint>

namespace starscope {

// Writes one complete JSON value (normally an object) into destination and
// returns its byte length. Returning zero omits sky_context for that invoke.
using VoiceSkyContextWriter = std::size_t (*)(
    char* destination, std::size_t capacity, std::uint32_t nowMs,
    void* userData);

// Same JSON writer contract, but called synchronously on the caller's task at
// pressPushToTalk(), before queueing the recording command. The returned JSON
// is owned by that recording until upload/discard; never read live UI state
// from the voice worker. Capacity is 8 KiB including the terminating NUL.
// Returning zero sends an explicit invalid view instead of reusing an old one.
using VoiceViewContextWriter = VoiceSkyContextWriter;

struct VoiceAssistantConfig {
  const char* wifiSsid = nullptr;
  const char* wifiPassword = nullptr;
  const char* webSocketHost = nullptr;
  std::uint16_t webSocketPort = 443;
  const char* webSocketPath = "/ws";
  const char* apiKey = nullptr;
  bool allowInsecureTls = false;
  const char* rootCa = nullptr;
  // M5Unified speaker level: 0 is muted and 255 is maximum. The default is
  // intentionally louder than the previous fixed level of 160.
  std::uint8_t playbackVolume = 217;
  VoiceSkyContextWriter skyContextWriter = nullptr;
  void* skyContextUserData = nullptr;
  std::uint32_t skyContextIntervalMs = 10U * 60U * 1000U;
  VoiceViewContextWriter viewContextWriter = nullptr;
  void* viewContextUserData = nullptr;
};

enum class VoiceAssistantState : std::uint8_t {
  Offline,
  Ready,
  TentativeRecording,
  Recording,
  FinishingRecording,
  Sending,
  Receiving,
  Playing,
  Error,
};

class VoiceAssistant {
 public:
  VoiceAssistant();
  ~VoiceAssistant();

  VoiceAssistant(const VoiceAssistant&) = delete;
  VoiceAssistant& operator=(const VoiceAssistant&) = delete;

  void begin(const VoiceAssistantConfig& config, std::uint32_t nowMs);
  void update(std::uint32_t nowMs);

  // Start recording immediately so the first second is not clipped. If the
  // press is released before commitPushToTalk(), releasePushToTalk() discards
  // the audio and returns false so the caller can perform its short action.
  void pressPushToTalk(std::uint32_t nowMs);
  void commitPushToTalk();
  bool releasePushToTalk(std::uint32_t nowMs);
  // Forces the next successful invoke to carry a fresh sky_context.
  void invalidateSkyContext();

  VoiceAssistantState state() const;
  const char* statusLabel() const;
  bool isOnline() const;
  bool isRecording() const;
  bool isBusy() const;

 private:
  class Impl;
  Impl* impl_ = nullptr;
};

}  // namespace starscope
