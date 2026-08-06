#include "starscope/voice_assistant.hpp"

#include <Arduino.h>
#include <M5Unified.h>
#include <WebSocketsClient.h>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <mbedtls/base64.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace starscope {
namespace {

constexpr std::uint32_t kSampleRate = 16000;
constexpr std::uint32_t kMaximumRecordingSeconds = 20;
constexpr std::size_t kMaximumSamples =
    kSampleRate * kMaximumRecordingSeconds;
constexpr std::size_t kMaximumCaptureBytes =
    kMaximumSamples * sizeof(std::int16_t);
constexpr std::size_t kRecordBlockSamples = 512;
constexpr std::uint32_t kWifiRetryMs = 15000;
constexpr std::uint32_t kWebSocketReconnectMs = 5000;
constexpr std::uint32_t kResponseTimeoutMs = 120000;
constexpr std::uint32_t kHeartbeatIntervalMs = 15000;
constexpr std::uint32_t kHeartbeatTimeoutMs = 3000;
constexpr std::size_t kCommandQueueCapacity = 12;
constexpr std::uint32_t kWorkerStackBytes = 16 * 1024;
constexpr UBaseType_t kWorkerPriority = 1;
constexpr BaseType_t kWorkerCore = 0;
constexpr std::size_t kBase64EncodeInputBlock = 3072;
constexpr std::size_t kCodecYieldBytes = 24 * 1024;
// A magnitude-3 sky context is normally 3-8 KiB. Keep a strict upper bound
// while leaving enough room for escaped location/star names and invoke fields.
constexpr std::size_t kInvokeJsonOverhead = 16 * 1024;
constexpr std::size_t kMaximumBase64CaptureBytes =
    4 * ((kMaximumCaptureBytes + 2) / 3);
constexpr std::size_t kInvokeBufferBytes =
    kMaximumBase64CaptureBytes + kInvokeJsonOverhead;
// 32.8 seconds of mono PCM16 at 16 kHz. The buffer is allocated once in PSRAM
// and provides backpressure headroom while the server emits a response faster
// than the speaker can consume it.
constexpr std::size_t kPlaybackBufferBytes = 1024 * 1024;
constexpr std::size_t kPlaybackStartBytes = 4096;
constexpr std::size_t kPlaybackSubmitBytes = 4096;
// The server is configured for 2048-byte PCM frames. Leave room for larger
// diagnostic settings without permitting an unbounded WebSocket allocation.
constexpr std::size_t kDecodeScratchBytes = 16 * 1024;
// AIAvatarKit responses are normally a few KiB, but WebSocket intermediaries
// are allowed to fragment one logical text message across multiple frames.
// Reuse the already allocated invoke buffer for bounded reassembly after an
// upload has completed instead of allocating on the receive path.
constexpr std::size_t kMaximumInboundTextBytes = 64 * 1024;

static_assert(kBase64EncodeInputBlock % 3 == 0);
static_assert(kPlaybackBufferBytes % sizeof(std::int16_t) == 0);

enum class VoiceCommandType : std::uint8_t {
  PressPushToTalk,
  CommitPushToTalk,
  ReleasePushToTalk,
};

struct VoiceCommand {
  VoiceCommandType type;
  std::uint32_t timestampMs;
};

void* allocatePsram(std::size_t size) {
  void* pointer = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  return pointer ? pointer : std::malloc(size);
}

const char* findBounded(const char* begin, const char* end,
                        const char* token) {
  const std::size_t tokenLength = std::strlen(token);
  if (tokenLength == 0 || static_cast<std::size_t>(end - begin) < tokenLength) {
    return nullptr;
  }
  const char* found = std::search(begin, end, token, token + tokenLength);
  return found == end ? nullptr : found;
}

bool jsonStringSpan(const char* json, std::size_t jsonLength, const char* key,
                    const char*& value, std::size_t& valueLength) {
  char quotedKey[64];
  const int quotedLength =
      std::snprintf(quotedKey, sizeof(quotedKey), "\"%s\"", key);
  if (quotedLength <= 0 ||
      static_cast<std::size_t>(quotedLength) >= sizeof(quotedKey)) {
    return false;
  }

  const char* end = json + jsonLength;
  const char* cursor = findBounded(json, end, quotedKey);
  if (!cursor) return false;
  cursor += quotedLength;
  while (cursor < end && (*cursor == ' ' || *cursor == '\t')) ++cursor;
  if (cursor == end || *cursor++ != ':') return false;
  while (cursor < end && (*cursor == ' ' || *cursor == '\t')) ++cursor;
  if (cursor == end || *cursor++ != '"') return false;

  value = cursor;
  bool escaped = false;
  while (cursor < end) {
    if (!escaped && *cursor == '"') {
      valueLength = static_cast<std::size_t>(cursor - value);
      return true;
    }
    if (!escaped && *cursor == '\\') {
      escaped = true;
    } else {
      escaped = false;
    }
    ++cursor;
  }
  return false;
}

bool jsonUnsigned(const char* json, std::size_t jsonLength, const char* key,
                  std::uint32_t& value) {
  char quotedKey[64];
  const int quotedLength =
      std::snprintf(quotedKey, sizeof(quotedKey), "\"%s\"", key);
  if (quotedLength <= 0 ||
      static_cast<std::size_t>(quotedLength) >= sizeof(quotedKey)) {
    return false;
  }

  const char* end = json + jsonLength;
  const char* cursor = findBounded(json, end, quotedKey);
  if (!cursor) return false;
  cursor += quotedLength;
  while (cursor < end && (*cursor == ' ' || *cursor == '\t')) ++cursor;
  if (cursor == end || *cursor++ != ':') return false;
  while (cursor < end && (*cursor == ' ' || *cursor == '\t')) ++cursor;
  if (cursor == end || *cursor < '0' || *cursor > '9') return false;

  std::uint32_t parsed = 0;
  while (cursor < end && *cursor >= '0' && *cursor <= '9') {
    parsed = parsed * 10U + static_cast<std::uint32_t>(*cursor - '0');
    ++cursor;
  }
  value = parsed;
  return true;
}

void generateUuid(char* destination, std::size_t capacity) {
  if (!destination || capacity < 37) return;
  std::uint8_t bytes[16];
  for (auto& byte : bytes) byte = static_cast<std::uint8_t>(esp_random());
  bytes[6] = static_cast<std::uint8_t>((bytes[6] & 0x0fU) | 0x40U);
  bytes[8] = static_cast<std::uint8_t>((bytes[8] & 0x3fU) | 0x80U);
  std::snprintf(
      destination, capacity,
      "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-"
      "%02x%02x%02x%02x%02x%02x",
      bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5],
      bytes[6], bytes[7], bytes[8], bytes[9], bytes[10], bytes[11],
      bytes[12], bytes[13], bytes[14], bytes[15]);
}

}  // namespace

class VoiceAssistant::Impl {
 public:
  ~Impl() {
    shutdownRequested_.store(true, std::memory_order_release);
    if (taskHandle_) {
      for (int attempt = 0; attempt < 200 && taskHandle_; ++attempt) delay(1);
      if (taskHandle_) {
        vTaskDelete(taskHandle_);
        taskHandle_ = nullptr;
      }
    }
    if (commandQueue_) {
      vQueueDelete(commandQueue_);
      commandQueue_ = nullptr;
    }
    freeBuffers();
  }

  void begin(const VoiceAssistantConfig& config, std::uint32_t nowMs) {
    if (taskHandle_) return;
    config_ = config;
    nowMs_ = nowMs;
    state_.store(VoiceAssistantState::Offline, std::memory_order_release);
    online_.store(false, std::memory_order_release);
    playbackActive_.store(false, std::memory_order_release);
    commandQueue_ = xQueueCreate(kCommandQueueCapacity, sizeof(VoiceCommand));
    if (!commandQueue_) {
      publishError("VOICE TASK");
      return;
    }
    const BaseType_t created = xTaskCreatePinnedToCore(
        taskEntry, "starscope_voice", kWorkerStackBytes, this,
        kWorkerPriority, &taskHandle_, kWorkerCore);
    if (created != pdPASS) {
      vQueueDelete(commandQueue_);
      commandQueue_ = nullptr;
      taskHandle_ = nullptr;
      publishError("VOICE TASK");
    }
  }

  // All voice work is isolated on the worker task. The public update hook is a
  // no-op so the optional module can be attached without burdening the scope
  // rendering loop.
  void update(std::uint32_t) {}

  void pressPushToTalk(std::uint32_t nowMs) {
    if (uiButtonActive_) return;
    if (!enqueueCommand(VoiceCommandType::PressPushToTalk, nowMs)) return;
    uiButtonActive_ = true;
    uiPttCommitted_ = false;
  }

  void commitPushToTalk() {
    if (!uiButtonActive_ || uiPttCommitted_) return;
    if (!enqueueCommand(VoiceCommandType::CommitPushToTalk, millis())) return;
    uiPttCommitted_ = true;
  }

  bool releasePushToTalk(std::uint32_t nowMs) {
    if (!uiButtonActive_) return false;
    const bool committed = uiPttCommitted_;
    uiButtonActive_ = false;
    uiPttCommitted_ = false;
    if (!enqueueCommand(VoiceCommandType::ReleasePushToTalk, nowMs)) {
      Serial.println("[VOICE] command queue full on release");
    }
    return committed;
  }

  void invalidateSkyContext() {
    skyContextRevision_.fetch_add(1U, std::memory_order_acq_rel);
  }

  VoiceAssistantState state() const {
    return state_.load(std::memory_order_acquire);
  }

  const char* statusLabel() const {
    switch (state()) {
      case VoiceAssistantState::Offline: return "VOICE OFF";
      case VoiceAssistantState::Ready: return "VOICE";
      case VoiceAssistantState::TentativeRecording: return "HOLD";
      case VoiceAssistantState::Recording: return "REC";
      case VoiceAssistantState::FinishingRecording: return "REC END";
      case VoiceAssistantState::Sending: return "SEND";
      case VoiceAssistantState::Receiving:
        return playbackActive_.load(std::memory_order_acquire) ? "AI" : "THINK";
      case VoiceAssistantState::Playing: return "AI";
      case VoiceAssistantState::Error:
        return errorLabel_.load(std::memory_order_acquire);
    }
    return "VOICE";
  }

  bool isOnline() const {
    return online_.load(std::memory_order_acquire);
  }

  bool isRecording() const {
    const VoiceAssistantState current = state();
    return current == VoiceAssistantState::TentativeRecording ||
           current == VoiceAssistantState::Recording ||
           current == VoiceAssistantState::FinishingRecording;
  }

  bool isBusy() const {
    const VoiceAssistantState current = state();
    return current == VoiceAssistantState::TentativeRecording ||
           current == VoiceAssistantState::Recording ||
           current == VoiceAssistantState::FinishingRecording ||
           current == VoiceAssistantState::Sending ||
           current == VoiceAssistantState::Receiving ||
           current == VoiceAssistantState::Playing;
  }

 private:
  static void taskEntry(void* argument) {
    static_cast<Impl*>(argument)->runWorker();
  }

  void runWorker() {
    Serial.printf("[VOICE] worker started on core %d, priority=%u\n",
                  xPortGetCoreID(),
                  static_cast<unsigned>(uxTaskPriorityGet(nullptr)));
    if (!beginWorker()) {
      taskHandle_ = nullptr;
      vTaskDelete(nullptr);
      return;
    }

    while (!shutdownRequested_.load(std::memory_order_acquire)) {
      processCommands();
      updateWorker(millis());
      vTaskDelay(pdMS_TO_TICKS(2));
    }
    webSocket_.disconnect();
    stopAndClearPlayback();
    stopCapture();
    taskHandle_ = nullptr;
    vTaskDelete(nullptr);
  }

  bool beginWorker() {
    if (!allocateBuffers()) {
      publishError("NO PSRAM");
      return false;
    }
    Serial.printf(
        "[VOICE] fixed buffers capture=%uKB invoke=%uKB playback=%uKB "
        "scratch=%uKB, free_psram=%u\n",
        static_cast<unsigned>(kMaximumCaptureBytes / 1024),
        static_cast<unsigned>(kInvokeBufferBytes / 1024),
        static_cast<unsigned>(kPlaybackBufferBytes / 1024),
        static_cast<unsigned>(kDecodeScratchBytes / 1024),
        static_cast<unsigned>(ESP.getFreePsram()));

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);
    WiFi.begin(config_.wifiSsid, config_.wifiPassword);
    lastWifiAttemptMs_ = nowMs_;
    if (M5.Mic.isRunning()) M5.Mic.end();
    M5.Speaker.stop();
    if (M5.Speaker.isRunning()) M5.Speaker.end();
    M5.Speaker.setVolume(config_.playbackVolume);
    return true;
  }

  bool allocateBuffers() {
    captureBuffer_ = static_cast<std::int16_t*>(
        allocatePsram(kMaximumCaptureBytes));
    invokeBuffer_ = static_cast<char*>(allocatePsram(kInvokeBufferBytes));
    playbackBuffer_ = static_cast<std::uint8_t*>(
        allocatePsram(kPlaybackBufferBytes));
    decodeScratch_ = static_cast<std::uint8_t*>(
        allocatePsram(kDecodeScratchBytes));
    if (captureBuffer_ && invokeBuffer_ && playbackBuffer_ && decodeScratch_) {
      return true;
    }
    freeBuffers();
    return false;
  }

  void freeBuffers() {
    std::free(captureBuffer_);
    std::free(invokeBuffer_);
    std::free(playbackBuffer_);
    std::free(decodeScratch_);
    captureBuffer_ = nullptr;
    invokeBuffer_ = nullptr;
    playbackBuffer_ = nullptr;
    decodeScratch_ = nullptr;
  }

  void updateWorker(std::uint32_t nowMs) {
    nowMs_ = nowMs;
    updateWifi();
    if (webSocketStarted_ && WiFi.status() == WL_CONNECTED) {
      webSocket_.loop();
    }

    const VoiceAssistantState current = state();
    if (current == VoiceAssistantState::TentativeRecording ||
        current == VoiceAssistantState::Recording ||
        current == VoiceAssistantState::FinishingRecording) {
      updateRecording();
    }

    if (hasPlaybackWork()) updatePlayback();

    // updateRecording() can encode and send a complete invoke. That work uses
    // fresh millis() values and may therefore set lastNetworkActivityMs_ later
    // than the nowMs captured at the beginning of this worker iteration. Using
    // the stale value here would underflow uint32_t and cause an immediate
    // false TIMEOUT directly after every successful send.
    const std::uint32_t timeoutNowMs = millis();
    nowMs_ = timeoutNowMs;
    const VoiceAssistantState afterWork = state();
    if ((afterWork == VoiceAssistantState::Receiving ||
         afterWork == VoiceAssistantState::Playing) &&
        timeoutNowMs - lastNetworkActivityMs_ > kResponseTimeoutMs) {
      fail("TIMEOUT");
    }
  }

  bool enqueueCommand(VoiceCommandType type, std::uint32_t timestampMs) {
    if (!commandQueue_ || !taskHandle_) return false;
    const VoiceCommand command{type, timestampMs};
    if (xQueueSend(commandQueue_, &command, 0) == pdTRUE) return true;
    Serial.println("[VOICE] command queue full");
    return false;
  }

  void processCommands() {
    VoiceCommand command{};
    while (xQueueReceive(commandQueue_, &command, 0) == pdTRUE) {
      switch (command.type) {
        case VoiceCommandType::PressPushToTalk:
          handlePressPushToTalk(command.timestampMs);
          break;
        case VoiceCommandType::CommitPushToTalk:
          handleCommitPushToTalk();
          break;
        case VoiceCommandType::ReleasePushToTalk:
          handleReleasePushToTalk(command.timestampMs);
          break;
      }
    }
  }

  void handlePressPushToTalk(std::uint32_t nowMs) {
    if (buttonActive_) return;
    nowMs_ = nowMs;
    buttonActive_ = true;
    pttCommitted_ = false;
    finishRequested_ = false;
    cancelLocalResponse();
    startCapture();
  }

  void handleCommitPushToTalk() {
    if (!buttonActive_ || pttCommitted_) return;
    pttCommitted_ = true;
    if (state() == VoiceAssistantState::TentativeRecording) {
      state_ = VoiceAssistantState::Recording;
    }
  }

  void handleReleasePushToTalk(std::uint32_t nowMs) {
    if (!buttonActive_) return;
    nowMs_ = nowMs;
    buttonActive_ = false;
    if (!pttCommitted_) {
      stopCapture();
      setIdleState();
      return;
    }
    releaseRequestedMs_ = nowMs;
    requestFinishRecording();
  }

  void updateWifi() {
    if (WiFi.status() != WL_CONNECTED) {
      online_.store(false, std::memory_order_release);
      if (state() == VoiceAssistantState::Ready) {
        state_ = VoiceAssistantState::Offline;
      }
      if (nowMs_ - lastWifiAttemptMs_ >= kWifiRetryMs) {
        lastWifiAttemptMs_ = nowMs_;
        WiFi.reconnect();
      }
      return;
    }

    if (!webSocketStarted_) startWebSocket();
    online_.store(webSocketConnected_ && serverReady_,
                  std::memory_order_release);
  }

  void startWebSocket() {
    webSocketStarted_ = true;
    if (!config_.webSocketHost || !config_.webSocketHost[0] ||
        !config_.webSocketPath || !config_.webSocketPath[0]) {
      publishError("BAD WS URL");
      return;
    }

    webSocket_.onEvent(
        [this](WStype_t type, std::uint8_t* payload, std::size_t length) {
          onWebSocketEvent(type, payload, length);
        });
    webSocket_.setReconnectInterval(kWebSocketReconnectMs);
    webSocket_.enableHeartbeat(kHeartbeatIntervalMs, kHeartbeatTimeoutMs, 2);
    if (config_.apiKey && config_.apiKey[0]) {
      std::snprintf(authorizationHeader_, sizeof(authorizationHeader_),
                    "Bearer %s", config_.apiKey);
      webSocket_.setAuthorization(authorizationHeader_);
    }

    Serial.printf("[VOICE] WebSocket begin %s://%s:%u%s\n",
                  config_.webSocketPort == 443 ? "wss" : "ws",
                  config_.webSocketHost, config_.webSocketPort,
                  config_.webSocketPath);
    if (config_.webSocketPort == 443) {
      if (config_.rootCa && config_.rootCa[0]) {
        webSocket_.beginSslWithCA(config_.webSocketHost,
                                  config_.webSocketPort,
                                  config_.webSocketPath, config_.rootCa);
      } else if (config_.allowInsecureTls) {
        webSocket_.beginSSL(config_.webSocketHost,
                            config_.webSocketPort,
                            config_.webSocketPath);
      } else {
        publishError("TLS CA");
      }
    } else {
      webSocket_.begin(config_.webSocketHost, config_.webSocketPort,
                       config_.webSocketPath);
    }
  }

  void onWebSocketEvent(WStype_t type, std::uint8_t* payload,
                        std::size_t length) {
    switch (type) {
      case WStype_CONNECTED:
        webSocketConnected_ = true;
        serverReady_ = false;
        skyContextRequiredForConnection_ = true;
        generateUuid(sessionId_, sizeof(sessionId_));
        Serial.printf("[VOICE] WebSocket connected session=%s\n", sessionId_);
        if (!sendSessionStart()) publishError("WS START");
        break;

      case WStype_DISCONNECTED:
        if (webSocketConnected_) Serial.println("[VOICE] WebSocket disconnected");
        webSocketConnected_ = false;
        serverReady_ = false;
        online_.store(false, std::memory_order_release);
        if (!isRecording()) {
          stopAndClearPlayback();
          state_ = VoiceAssistantState::Offline;
        }
        break;

      case WStype_TEXT:
        processWebSocketText(reinterpret_cast<const char*>(payload), length);
        break;

      case WStype_FRAGMENT_TEXT_START:
        fragmentedTextActive_ = true;
        fragmentedTextLength_ = 0;
        Serial.printf("[VOICE] WS RX fragment start bytes=%u\n",
                      static_cast<unsigned>(length));
        if (!appendTextFragment(payload, length)) fail("WS RX LARGE");
        break;

      case WStype_FRAGMENT:
        if (fragmentedTextActive_ && !appendTextFragment(payload, length)) {
          fail("WS RX LARGE");
        }
        break;

      case WStype_FRAGMENT_FIN:
        if (fragmentedTextActive_ && appendTextFragment(payload, length)) {
          Serial.printf("[VOICE] WS RX fragment final total=%u\n",
                        static_cast<unsigned>(fragmentedTextLength_));
          processWebSocketText(invokeBuffer_, fragmentedTextLength_);
        } else if (fragmentedTextActive_) {
          fail("WS RX LARGE");
        }
        fragmentedTextActive_ = false;
        fragmentedTextLength_ = 0;
        break;

      case WStype_BIN:
      case WStype_FRAGMENT_BIN_START:
        Serial.printf("[VOICE] unexpected WS binary frame bytes=%u\n",
                      static_cast<unsigned>(length));
        break;

      case WStype_ERROR:
        Serial.println("[VOICE] WebSocket transport error");
        break;

      default:
        break;
    }
  }

  bool sendSessionStart() {
    char message[512];
    const int written = contextId_[0]
        ? std::snprintf(
              message, sizeof(message),
              "{\"type\":\"start\",\"session_id\":\"%s\","
              "\"user_id\":\"m5starscope\",\"context_id\":\"%s\"}",
              sessionId_, contextId_)
        : std::snprintf(
              message, sizeof(message),
              "{\"type\":\"start\",\"session_id\":\"%s\","
              "\"user_id\":\"m5starscope\",\"context_id\":null}",
              sessionId_);
    return written > 0 && static_cast<std::size_t>(written) < sizeof(message) &&
           webSocket_.sendTXT(message, static_cast<std::size_t>(written));
  }

  bool appendTextFragment(const std::uint8_t* payload, std::size_t length) {
    if (!fragmentedTextActive_ || !invokeBuffer_ || !payload ||
        length > kMaximumInboundTextBytes - fragmentedTextLength_) {
      return false;
    }
    std::memcpy(invokeBuffer_ + fragmentedTextLength_, payload, length);
    fragmentedTextLength_ += length;
    invokeBuffer_[fragmentedTextLength_] = '\0';
    return true;
  }

  void processWebSocketText(const char* json, std::size_t jsonLength) {
    lastNetworkActivityMs_ = nowMs_;
    const char* value = nullptr;
    std::size_t valueLength = 0;
    if (jsonStringSpan(json, jsonLength, "context_id", value, valueLength) &&
        valueLength > 0) {
      const std::size_t copyLength =
          std::min(valueLength, sizeof(contextId_) - 1);
      std::memcpy(contextId_, value, copyLength);
      contextId_[copyLength] = '\0';
    }

    if (!jsonStringSpan(json, jsonLength, "type", value, valueLength)) {
      Serial.printf("[VOICE] WS RX malformed text bytes=%u\n",
                    static_cast<unsigned>(jsonLength));
      return;
    }
    char messageType[24]{};
    const std::size_t typeLength =
        std::min(valueLength, sizeof(messageType) - 1);
    std::memcpy(messageType, value, typeLength);

    if (std::strcmp(messageType, "connected") == 0) {
      serverReady_ = true;
      online_.store(true, std::memory_order_release);
      if (!isBusy()) state_ = VoiceAssistantState::Ready;
      Serial.println("[VOICE] WebSocket session ready");
      return;
    }

    if (std::strcmp(messageType, "accepted") == 0) {
      if (!awaitingResponse_) return;
      responseAccepted_ = true;
      state_ = VoiceAssistantState::Receiving;
      Serial.printf("[VOICE][LATENCY] release_to_accepted=%u ms\n",
                    static_cast<unsigned>(nowMs_ - releaseRequestedMs_));
      return;
    }

    if (std::strcmp(messageType, "start") == 0) {
      if (!awaitingResponse_) return;
      responseAccepted_ = true;
      state_ = VoiceAssistantState::Receiving;
      Serial.printf("[VOICE] WS RX start bytes=%u\n",
                    static_cast<unsigned>(jsonLength));
      return;
    }

    if (std::strcmp(messageType, "chunk") == 0) {
      if (!awaitingResponse_ || !responseAccepted_) return;
      if (!updatePcmFormat(json, jsonLength)) {
        fail("PCM FORMAT");
        return;
      }
      if (jsonStringSpan(json, jsonLength, "audio_data", value,
                         valueLength) && valueLength > 0) {
        if (!enqueueBase64Pcm(value, valueLength)) {
          fail("AUDIO FULL");
        }
      }
      return;
    }

    if (std::strcmp(messageType, "final") == 0) {
      if (!awaitingResponse_) return;
      // AIAvatarWebSocketServer closes the previous transaction with a
      // synthetic final immediately before accepting the next transaction.
      // The wire response does not expose transaction_id, so the only safe
      // correlation point is accepted: a final received before accepted
      // belongs to the previous response and must not complete the new invoke.
      if (!responseAccepted_) {
        Serial.println("[VOICE] WS RX stale final before accepted");
        return;
      }
      finishReceiving();
      return;
    }

    if (std::strcmp(messageType, "canceled") == 0 ||
        std::strcmp(messageType, "cancelled") == 0) {
      Serial.println("[VOICE] response canceled");
      stopAndClearPlayback();
      resetResponseState();
      setIdleState();
      return;
    }

    if (std::strcmp(messageType, "stop") == 0) {
      Serial.printf("[VOICE] WS RX stop awaiting=%d\n",
                    awaitingResponse_ ? 1 : 0);
      stopAndClearPlayback();
      if (!awaitingResponse_) setIdleState();
      return;
    }

    if (std::strcmp(messageType, "error") == 0) {
      fail("AI ERROR");
    }
  }

  bool updatePcmFormat(const char* json, std::size_t jsonLength) {
    std::uint32_t sampleRate = playbackSampleRate_;
    std::uint32_t channels = playbackChannels_;
    std::uint32_t sampleWidth = sizeof(std::int16_t);
    jsonUnsigned(json, jsonLength, "sample_rate", sampleRate);
    jsonUnsigned(json, jsonLength, "channels", channels);
    jsonUnsigned(json, jsonLength, "sample_width", sampleWidth);
    std::uint32_t bitsPerSample = sampleWidth * 8U;
    jsonUnsigned(json, jsonLength, "bits_per_sample", bitsPerSample);
    if (sampleRate < 8000 || sampleRate > 96000 ||
        (channels != 1 && channels != 2) || bitsPerSample != 16) {
      Serial.printf("[VOICE] unsupported PCM %u Hz, %u ch, %u bit\n",
                    static_cast<unsigned>(sampleRate),
                    static_cast<unsigned>(channels),
                    static_cast<unsigned>(bitsPerSample));
      return false;
    }
    playbackSampleRate_ = sampleRate;
    playbackChannels_ = static_cast<std::uint8_t>(channels);
    return true;
  }

  bool enqueueBase64Pcm(const char* base64, std::size_t base64Length) {
    const std::size_t required = ((base64Length + 3) / 4) * 3;
    if (required > kDecodeScratchBytes) {
      Serial.printf("[VOICE] PCM frame too large b64=%u\n",
                    static_cast<unsigned>(base64Length));
      return false;
    }
    std::size_t decodedLength = 0;
    if (mbedtls_base64_decode(
            decodeScratch_, kDecodeScratchBytes, &decodedLength,
            reinterpret_cast<const std::uint8_t*>(base64),
            base64Length) != 0 || decodedLength == 0 ||
        decodedLength % sizeof(std::int16_t) != 0) {
      return false;
    }
    if (decodedLength >= 12 &&
        std::memcmp(decodeScratch_, "RIFF", 4) == 0 &&
        std::memcmp(decodeScratch_ + 8, "WAVE", 4) == 0) {
      Serial.println("[VOICE] server returned WAV; enable PCM chunking");
      return false;
    }
    if (!enqueuePlaybackBytes(decodeScratch_, decodedLength)) return false;

    if (!firstAudioReceived_) {
      firstAudioReceived_ = true;
      Serial.printf("[VOICE][LATENCY] release_to_first_audio=%u ms\n",
                    static_cast<unsigned>(nowMs_ - releaseRequestedMs_));
    }
    playbackActive_.store(true, std::memory_order_release);
    Serial.printf("[VOICE] PCM queued bytes=%u buffered=%u\n",
                  static_cast<unsigned>(decodedLength),
                  static_cast<unsigned>(playbackBytes_));
    return true;
  }

  bool enqueuePlaybackBytes(const std::uint8_t* data, std::size_t length) {
    if (!data || length == 0 || length > kPlaybackBufferBytes - playbackBytes_) {
      return false;
    }
    const std::size_t first =
        std::min(length, kPlaybackBufferBytes - playbackTail_);
    std::memcpy(playbackBuffer_ + playbackTail_, data, first);
    if (length > first) {
      std::memcpy(playbackBuffer_, data + first, length - first);
    }
    playbackTail_ = (playbackTail_ + length) % kPlaybackBufferBytes;
    playbackBytes_ += length;
    return true;
  }

  void startCapture() {
    stopCapture();
    captureSamples_ = 0;
    finishRequested_ = false;
    M5.Speaker.stop();
    if (M5.Speaker.isRunning()) M5.Speaker.end();
    if (!M5.Mic.begin()) {
      fail("NO MIC");
      return;
    }
    state_ = VoiceAssistantState::TentativeRecording;
  }

  void updateRecording() {
    if (!captureBuffer_) {
      fail("NO BUFFER");
      return;
    }
    if (!finishRequested_) {
      while (captureSamples_ < kMaximumSamples && M5.Mic.isRecording() < 2) {
        const std::size_t block = std::min(
            kRecordBlockSamples, kMaximumSamples - captureSamples_);
        if (!M5.Mic.record(captureBuffer_ + captureSamples_, block,
                           kSampleRate, false)) {
          fail("MIC ERROR");
          stopCapture();
          return;
        }
        captureSamples_ += block;
      }
      if (captureSamples_ >= kMaximumSamples) {
        pttCommitted_ = true;
        releaseRequestedMs_ = nowMs_;
        requestFinishRecording();
      }
    }

    if (finishRequested_ && M5.Mic.isRecording() == 0) {
      if (M5.Mic.isRunning()) M5.Mic.end();
      sendCapture();
    }
  }

  void requestFinishRecording() {
    if (finishRequested_) return;
    finishRequested_ = true;
    state_ = VoiceAssistantState::FinishingRecording;
  }

  void stopCapture() {
    if (M5.Mic.isRunning()) M5.Mic.end();
    captureSamples_ = 0;
    finishRequested_ = false;
  }

  void logCaptureStats() const {
    if (!captureBuffer_ || captureSamples_ == 0) return;
    std::uint64_t squareSum = 0;
    std::uint32_t peak = 0;
    for (std::size_t index = 0; index < captureSamples_; ++index) {
      const std::int32_t sample = captureBuffer_[index];
      const std::uint32_t absolute = static_cast<std::uint32_t>(
          sample < 0 ? -sample : sample);
      peak = std::max(peak, absolute);
      squareSum += static_cast<std::uint64_t>(sample * sample);
      if (index && (index % (kCodecYieldBytes / sizeof(std::int16_t))) == 0) {
        vTaskDelay(1);
      }
    }
    const double rms =
        std::sqrt(static_cast<double>(squareSum) / captureSamples_);
    Serial.printf("[VOICE] recorded %u samples (%u ms), peak=%u rms=%.0f\n",
                  static_cast<unsigned>(captureSamples_),
                  static_cast<unsigned>(captureSamples_ * 1000 / kSampleRate),
                  static_cast<unsigned>(peak), rms);
  }

  bool skyContextDue(std::uint32_t nowMs) const {
    if (!config_.skyContextWriter) return false;
    if (skyContextRequiredForConnection_ || !hasSentSkyContext_) return true;
    if (skyContextRevision_.load(std::memory_order_acquire) !=
        lastSkyContextRevisionSent_) {
      return true;
    }
    return config_.skyContextIntervalMs == 0 ||
           nowMs - lastSkyContextSentMs_ >= config_.skyContextIntervalMs;
  }

  bool appendInvokeBytes(std::size_t& messageLength, const char* bytes,
                         std::size_t length) {
    if (!bytes || messageLength >= kInvokeBufferBytes ||
        length >= kInvokeBufferBytes - messageLength) {
      return false;
    }
    std::memcpy(invokeBuffer_ + messageLength, bytes, length);
    messageLength += length;
    invokeBuffer_[messageLength] = '\0';
    return true;
  }

  bool appendInvokeText(std::size_t& messageLength, const char* text) {
    return appendInvokeBytes(messageLength, text, std::strlen(text));
  }

  bool buildInvokeMessage(std::size_t& messageLength,
                          bool& includedSkyContext,
                          std::uint32_t& includedSkyContextRevision) {
    includedSkyContext = false;
    includedSkyContextRevision = 0;
    const int headerLength = contextId_[0]
        ? std::snprintf(
              invokeBuffer_, kInvokeJsonOverhead,
              "{\"type\":\"invoke\",\"session_id\":\"%s\","
              "\"user_id\":\"m5starscope\",\"context_id\":\"%s\","
              "\"text\":\"\",\"audio_data\":\"",
              sessionId_, contextId_)
        : std::snprintf(
              invokeBuffer_, kInvokeJsonOverhead,
              "{\"type\":\"invoke\",\"session_id\":\"%s\","
              "\"user_id\":\"m5starscope\",\"context_id\":null,"
              "\"text\":\"\",\"audio_data\":\"",
              sessionId_);
    if (headerLength <= 0 ||
        static_cast<std::size_t>(headerLength) >= kInvokeJsonOverhead) {
      return false;
    }

    const std::size_t captureBytes =
        captureSamples_ * sizeof(std::int16_t);
    std::size_t inputOffset = 0;
    std::size_t encodedLength = 0;
    std::size_t bytesSinceYield = 0;
    while (inputOffset < captureBytes) {
      const std::size_t inputLength = std::min(
          kBase64EncodeInputBlock, captureBytes - inputOffset);
      const std::size_t outputCapacity = 4 * ((inputLength + 2) / 3);
      std::size_t outputLength = 0;
      if (mbedtls_base64_encode(
              reinterpret_cast<std::uint8_t*>(invokeBuffer_) + headerLength +
                  encodedLength,
              outputCapacity + 1, &outputLength,
              reinterpret_cast<const std::uint8_t*>(captureBuffer_) +
                  inputOffset,
              inputLength) != 0 || outputLength > outputCapacity) {
        return false;
      }
      inputOffset += inputLength;
      encodedLength += outputLength;
      bytesSinceYield += inputLength;
      if (bytesSinceYield >= kCodecYieldBytes) {
        bytesSinceYield = 0;
        vTaskDelay(1);
      }
    }

    static constexpr char metadataPrefix[] =
        "\",\"files\":[],\"allow_merge\":false,"
        "\"wait_in_queue\":false,\"metadata\":{"
        "\"audio_format\":{\"codec\":\"pcm16\","
        "\"sample_rate\":16000,\"channels\":1,"
        "\"bits_per_sample\":16}";
    messageLength =
        static_cast<std::size_t>(headerLength) + encodedLength;
    if (!appendInvokeText(messageLength, metadataPrefix)) return false;

    const std::uint32_t contextNowMs = millis();
    if (skyContextDue(contextNowMs)) {
      const std::size_t withoutSkyContext = messageLength;
      static constexpr char skyContextKey[] = ",\"sky_context\":";
      if (appendInvokeText(messageLength, skyContextKey)) {
        const std::size_t reservedClosingBytes = 2;
        const std::size_t available =
            kInvokeBufferBytes - messageLength - reservedClosingBytes - 1;
        includedSkyContextRevision =
            skyContextRevision_.load(std::memory_order_acquire);
        const std::size_t contextLength = config_.skyContextWriter(
            invokeBuffer_ + messageLength, available, contextNowMs,
            config_.skyContextUserData);
        if (contextLength > 0 && contextLength <= available) {
          messageLength += contextLength;
          invokeBuffer_[messageLength] = '\0';
          includedSkyContext = true;
        } else {
          messageLength = withoutSkyContext;
          invokeBuffer_[messageLength] = '\0';
          includedSkyContextRevision = 0;
          Serial.println("[VOICE] sky_context unavailable; sending audio only");
        }
      }
    }

    return appendInvokeText(messageLength, "}}");
  }

  void sendCapture() {
    if (!captureBuffer_ || captureSamples_ == 0) {
      fail("NO AUDIO");
      stopCapture();
      return;
    }
    if (!serverReady_ || !webSocketConnected_) {
      stopCapture();
      fail("VOICE OFF");
      return;
    }

    logCaptureStats();
    state_ = VoiceAssistantState::Sending;
    const std::uint32_t encodeStartedMs = millis();
    std::size_t messageLength = 0;
    bool includedSkyContext = false;
    std::uint32_t includedSkyContextRevision = 0;
    if (!buildInvokeMessage(messageLength, includedSkyContext,
                            includedSkyContextRevision)) {
      stopCapture();
      fail("ENCODE ERR");
      return;
    }
    const std::uint32_t encodedMs = millis();

    resetResponseState();
    awaitingResponse_ = true;
    responseAccepted_ = false;
    responseComplete_ = false;
    firstAudioReceived_ = false;
    firstPlaybackStarted_ = false;
    lastNetworkActivityMs_ = encodedMs;
    const bool sent = webSocket_.sendTXT(
        reinterpret_cast<const std::uint8_t*>(invokeBuffer_), messageLength);
    const std::uint32_t sentMs = millis();
    if (sent && includedSkyContext) {
      hasSentSkyContext_ = true;
      skyContextRequiredForConnection_ = false;
      lastSkyContextSentMs_ = sentMs;
      lastSkyContextRevisionSent_ = includedSkyContextRevision;
    }
    Serial.printf(
        "[VOICE] invoke samples=%u bytes=%u sent=%d sky_context=%d\n"
        "[VOICE][LATENCY] encode=%u ms send=%u ms release_to_send=%u ms\n",
        static_cast<unsigned>(captureSamples_),
        static_cast<unsigned>(messageLength), sent ? 1 : 0,
        includedSkyContext ? 1 : 0,
        static_cast<unsigned>(encodedMs - encodeStartedMs),
        static_cast<unsigned>(sentMs - encodedMs),
        static_cast<unsigned>(sentMs - releaseRequestedMs_));
    stopCapture();
    if (!sent) {
      fail("WS SEND");
      return;
    }
    state_ = VoiceAssistantState::Receiving;
  }

  bool hasPlaybackWork() const {
    return activePlaybackBytes_ > 0 || playbackBytes_ > 0 ||
           M5.Speaker.isPlaying();
  }

  void updatePlayback() {
    if (activePlaybackBytes_ > 0) {
      if (nowMs_ - playbackSubmittedMs_ < 20 || M5.Speaker.isPlaying()) return;
      playbackHead_ =
          (playbackHead_ + activePlaybackBytes_) % kPlaybackBufferBytes;
      playbackBytes_ -= activePlaybackBytes_;
      activePlaybackBytes_ = 0;
    }

    if (playbackBytes_ > 0 &&
        (playbackStarted_ || responseComplete_ ||
         playbackBytes_ >= kPlaybackStartBytes)) {
      if (!speakerRunning_) {
        if (M5.Mic.isRunning()) M5.Mic.end();
        if (!M5.Speaker.begin()) {
          fail("SPEAKER ERR");
          return;
        }
        M5.Speaker.setVolume(config_.playbackVolume);
        speakerRunning_ = true;
      }

      const std::size_t frameBytes =
          playbackChannels_ * sizeof(std::int16_t);
      std::size_t submitBytes = std::min(
          {playbackBytes_, kPlaybackSubmitBytes,
           kPlaybackBufferBytes - playbackHead_});
      submitBytes -= submitBytes % frameBytes;
      if (submitBytes > 0) {
        const auto* samples = reinterpret_cast<const std::int16_t*>(
            playbackBuffer_ + playbackHead_);
        if (!M5.Speaker.playRaw(samples,
                                submitBytes / sizeof(std::int16_t),
                                playbackSampleRate_,
                                playbackChannels_ == 2, 1, 0, false)) {
          fail("SPEAKER ERR");
          return;
        }
        activePlaybackBytes_ = submitBytes;
        playbackSubmittedMs_ = nowMs_;
        playbackStarted_ = true;
        playbackActive_.store(true, std::memory_order_release);
        if (!firstPlaybackStarted_) {
          firstPlaybackStarted_ = true;
          Serial.printf("[VOICE][LATENCY] release_to_playback=%u ms\n",
                        static_cast<unsigned>(nowMs_ - releaseRequestedMs_));
        }
      }
      return;
    }

    if (responseComplete_ && activePlaybackBytes_ == 0 &&
        playbackBytes_ == 0 && !M5.Speaker.isPlaying()) {
      finishResponse();
    }
  }

  void clearPlaybackRing() {
    playbackHead_ = 0;
    playbackTail_ = 0;
    playbackBytes_ = 0;
    activePlaybackBytes_ = 0;
    playbackStarted_ = false;
    playbackActive_.store(false, std::memory_order_release);
  }

  void stopAndClearPlayback() {
    M5.Speaker.stop();
    for (int attempt = 0; attempt < 100 && M5.Speaker.isPlaying(); ++attempt) {
      vTaskDelay(1);
    }
    if (M5.Speaker.isRunning()) M5.Speaker.end();
    speakerRunning_ = false;
    clearPlaybackRing();
  }

  void finishReceiving() {
    responseComplete_ = true;
    awaitingResponse_ = false;
    if (hasPlaybackWork()) {
      state_ = VoiceAssistantState::Playing;
      playbackActive_.store(true, std::memory_order_release);
    } else {
      finishResponse();
    }
  }

  void finishResponse() {
    if (M5.Speaker.isRunning()) M5.Speaker.end();
    speakerRunning_ = false;
    clearPlaybackRing();
    resetResponseState();
    setIdleState();
  }

  void cancelLocalResponse() {
    stopAndClearPlayback();
    resetResponseState();
    setIdleState();
  }

  void resetResponseState() {
    awaitingResponse_ = false;
    responseAccepted_ = false;
    responseComplete_ = false;
    firstAudioReceived_ = false;
    firstPlaybackStarted_ = false;
    playbackSampleRate_ = kSampleRate;
    playbackChannels_ = 1;
  }

  void setIdleState() {
    const bool connected = webSocketConnected_ && serverReady_;
    online_.store(connected, std::memory_order_release);
    state_ = connected ? VoiceAssistantState::Ready
                       : VoiceAssistantState::Offline;
  }

  void publishError(const char* message) {
    errorLabel_.store(message ? message : "ERR", std::memory_order_release);
    state_.store(VoiceAssistantState::Error, std::memory_order_release);
  }

  void fail(const char* message) {
    Serial.printf("[VOICE] ERROR: %s\n", message);
    stopAndClearPlayback();
    resetResponseState();
    publishError(message);
  }

  VoiceAssistantConfig config_;
  std::atomic<VoiceAssistantState> state_{VoiceAssistantState::Offline};
  std::atomic<bool> online_{false};
  std::atomic<bool> playbackActive_{false};
  std::atomic<const char*> errorLabel_{"ERR"};
  std::atomic<bool> shutdownRequested_{false};
  std::atomic<std::uint32_t> skyContextRevision_{1};
  QueueHandle_t commandQueue_ = nullptr;
  TaskHandle_t taskHandle_ = nullptr;
  bool uiButtonActive_ = false;
  bool uiPttCommitted_ = false;

  WebSocketsClient webSocket_;
  bool webSocketStarted_ = false;
  bool webSocketConnected_ = false;
  bool serverReady_ = false;
  char authorizationHeader_[192]{};
  char sessionId_[40]{};
  char contextId_[80]{};

  std::uint32_t nowMs_ = 0;
  std::uint32_t lastWifiAttemptMs_ = 0;
  std::uint32_t lastNetworkActivityMs_ = 0;
  std::uint32_t releaseRequestedMs_ = 0;
  bool buttonActive_ = false;
  bool pttCommitted_ = false;
  bool finishRequested_ = false;
  bool awaitingResponse_ = false;
  bool responseAccepted_ = false;
  bool responseComplete_ = false;
  bool firstAudioReceived_ = false;
  bool firstPlaybackStarted_ = false;
  bool skyContextRequiredForConnection_ = true;
  bool hasSentSkyContext_ = false;
  std::uint32_t lastSkyContextSentMs_ = 0;
  std::uint32_t lastSkyContextRevisionSent_ = 0;

  std::int16_t* captureBuffer_ = nullptr;
  std::size_t captureSamples_ = 0;
  char* invokeBuffer_ = nullptr;
  std::uint8_t* decodeScratch_ = nullptr;
  std::uint8_t* playbackBuffer_ = nullptr;
  std::size_t playbackHead_ = 0;
  std::size_t playbackTail_ = 0;
  std::size_t playbackBytes_ = 0;
  std::size_t activePlaybackBytes_ = 0;
  std::uint32_t playbackSubmittedMs_ = 0;
  std::uint32_t playbackSampleRate_ = kSampleRate;
  std::uint8_t playbackChannels_ = 1;
  bool playbackStarted_ = false;
  bool speakerRunning_ = false;
  bool fragmentedTextActive_ = false;
  std::size_t fragmentedTextLength_ = 0;
};

VoiceAssistant::VoiceAssistant() : impl_(new Impl()) {}
VoiceAssistant::~VoiceAssistant() { delete impl_; }

void VoiceAssistant::begin(const VoiceAssistantConfig& config,
                           std::uint32_t nowMs) {
  impl_->begin(config, nowMs);
}

void VoiceAssistant::update(std::uint32_t nowMs) { impl_->update(nowMs); }

void VoiceAssistant::pressPushToTalk(std::uint32_t nowMs) {
  impl_->pressPushToTalk(nowMs);
}

void VoiceAssistant::commitPushToTalk() { impl_->commitPushToTalk(); }

bool VoiceAssistant::releasePushToTalk(std::uint32_t nowMs) {
  return impl_->releasePushToTalk(nowMs);
}

void VoiceAssistant::invalidateSkyContext() { impl_->invalidateSkyContext(); }

VoiceAssistantState VoiceAssistant::state() const { return impl_->state(); }
const char* VoiceAssistant::statusLabel() const { return impl_->statusLabel(); }
bool VoiceAssistant::isOnline() const { return impl_->isOnline(); }
bool VoiceAssistant::isRecording() const { return impl_->isRecording(); }
bool VoiceAssistant::isBusy() const { return impl_->isBusy(); }

}  // namespace starscope
