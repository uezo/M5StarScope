#include <Arduino.h>
#include <DNSServer.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp32-hal-psram.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <cstdarg>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "starscope/astronomy.hpp"
#include "starscope/city_catalog.hpp"
#include "starscope/geomagnetism.hpp"
#include "starscope/orientation.hpp"
#include "starscope/protocol.hpp"
#include "starscope/quaternion.hpp"
#include "starscope/scope_projection.hpp"
#include "starscope/star_catalog.hpp"
#include "starscope/types.hpp"
#include "starscope/voice_assistant.hpp"

namespace {

using starscope::CivilTime;
using starscope::CityCatalog;
using starscope::EmulatedOrientationProvider;
using starscope::Location;
using starscope::OrientationProvider;
using starscope::OrientationSample;
using starscope::Quaternion;
using starscope::ScopeBasis;
using starscope::StarCatalog;
using starscope::Vec3;
using starscope::VoiceAssistant;
using starscope::VoiceAssistantConfig;

constexpr int kScreenWidth = 466;
constexpr int kScreenHeight = 466;
constexpr int kCenter = 233;
constexpr int kScopeRadius = 222;
constexpr int kUartRxPin = 11;
constexpr int kUartTxPin = 10;
constexpr std::uint32_t kUartBaud = 115200;
constexpr std::uint32_t kOrientationTimeoutMs = 600;
constexpr std::uint32_t kFrameIntervalMs = 40;
constexpr UBaseType_t kScopeTaskPriority = 2;
constexpr std::uint32_t kButtonHoldMs = 1000;
constexpr std::uint32_t kMovingBodyUpdateMs = 30000;
constexpr std::uint8_t kNormalDisplayBrightness = 96;
constexpr std::uint8_t kNightDisplayBrightness = 32;
// Six stars per 25 fps frame refreshes all 8,921 catalogue entries in roughly
// one minute without a periodic all-at-once pause.
constexpr std::size_t kSkyStarsPerFrame = 6;
constexpr float kFieldOfViewDeg = 45.0F;
constexpr float kVoiceSkyMagnitudeLimit = 3.0F;
constexpr std::uint32_t kVoiceSkyContextIntervalMs = 10U * 60U * 1000U;
constexpr std::uint32_t kConfigurationRestartDelayMs = 800;
constexpr char kDefaultVoiceWebSocketHost[] = "192.168.1.100";
constexpr std::uint16_t kDefaultVoiceWebSocketPort = 8000;
constexpr char kDefaultVoiceWebSocketPath[] = "/ws";
constexpr std::uint8_t kDefaultVoiceVolumePercent = 85;
constexpr std::size_t kCitySearchResultLimit = 20;

// The initial bracket contract: Atom sensor +X points down the tube, +Z points
// away from the tube. Change these offsets after the physical bracket is tested.
constexpr float kMountYawDeg = 0.0F;
constexpr float kMountPitchDeg = 0.0F;
constexpr float kMountRollDeg = 0.0F;

struct Gesture {
  bool released = false;
  bool tap = false;
  int startX = 0;
  int startY = 0;
  int deltaX = 0;
  int deltaY = 0;
};

class GestureTracker {
 public:
  Gesture update() {
    Gesture event;
    const auto touch = M5.Touch.getDetail();
    if (touch.wasPressed()) {
      active_ = true;
      startX_ = touch.x;
      startY_ = touch.y;
    }
    if (active_ && touch.wasReleased()) {
      active_ = false;
      event.released = true;
      event.startX = startX_;
      event.startY = startY_;
      event.deltaX = touch.x - startX_;
      event.deltaY = touch.y - startY_;
      event.tap = std::abs(event.deltaX) < 18 && std::abs(event.deltaY) < 18;
    }
    return event;
  }

 private:
  bool active_ = false;
  int startX_ = 0;
  int startY_ = 0;
};

class AtomQuaternionProvider final : public OrientationProvider {
 public:
  AtomQuaternionProvider() : serial_(1) {}

  void begin() override {
    serial_.begin(kUartBaud, SERIAL_8N1, kUartRxPin, kUartTxPin);
  }

  void update(std::uint32_t nowMs) override {
    starscope::protocol::OrientationPacket packet;
    while (serial_.available() > 0) {
      if (parser_.push(static_cast<std::uint8_t>(serial_.read()), packet)) {
        const bool setupToggle =
            (packet.flags & starscope::protocol::kFlagSetupToggle) != 0;
        if (setupToggle && !setupToggleLevel_) setupTogglePending_ = true;
        setupToggleLevel_ = setupToggle;
        sample_ = starscope::protocol::toSample(packet, nowMs);
        lastPacketMs_ = nowMs;
      }
    }
    if (!connected(nowMs)) sample_.valid = false;
  }

  OrientationSample latest() const override { return sample_; }
  const char* name() const override { return "ATOM"; }
  bool connected(std::uint32_t nowMs) const {
    return lastPacketMs_ != 0 && nowMs - lastPacketMs_ <= kOrientationTimeoutMs;
  }

  bool consumeSetupToggle() {
    const bool pending = setupTogglePending_;
    setupTogglePending_ = false;
    return pending;
  }

 private:
  HardwareSerial serial_;
  starscope::protocol::OrientationPacketParser parser_;
  OrientationSample sample_;
  std::uint32_t lastPacketMs_ = 0;
  bool setupToggleLevel_ = false;
  bool setupTogglePending_ = false;
};

class ScopeOrientationProvider final : public OrientationProvider {
 public:
  void begin() override {
    atom_.begin();
    emulator_.begin();
    emulator_.set(180.0F, 25.0F, 0.0F);
  }

  void update(std::uint32_t nowMs) override {
    nowMs_ = nowMs;
    atom_.update(nowMs);
    emulator_.update(nowMs);
  }

  OrientationSample latest() const override {
    const auto atom = atom_.latest();
    return atom_.connected(nowMs_) && atom.valid ? atom : emulator_.latest();
  }

  const char* name() const override {
    const auto atom = atom_.latest();
    return atom_.connected(nowMs_) && atom.valid ? atom_.name()
                                                 : emulator_.name();
  }

  bool emulating() const { return name()[0] == 'E'; }

  bool consumeSetupToggle() { return atom_.consumeSetupToggle(); }

  void adjustEmulator(float azimuthDelta, float altitudeDelta,
                      float rollDelta = 0.0F) {
    auto current = emulator_.latest();
    current.yawDeg = starscope::astronomy::normalizeDegrees(
        current.yawDeg + azimuthDelta);
    current.pitchDeg =
        std::max(-85.0F, std::min(90.0F, current.pitchDeg + altitudeDelta));
    current.rollDeg = starscope::astronomy::signedAngleDifference(
        current.rollDeg + rollDelta, 0.0F);
    emulator_.set(current.yawDeg, current.pitchDeg, current.rollDeg);
  }

 private:
  AtomQuaternionProvider atom_;
  EmulatedOrientationProvider emulator_;
  std::uint32_t nowMs_ = 0;
};

// Kept separate from the observation context so a future "Sydney from Tokyo"
// mode changes the simulated sky without changing physical compass correction.
struct PhysicalPoseSettings {
  float trueNorthCorrectionDeg = -7.5F;
};

struct ObservationContext {
  Location location;
  char locationName[48] = "Tokyo";
  float limitingMagnitude = 6.5F;
};

class SettingsStore {
 public:
  void begin() {
    preferences_.begin("m5starscope", false);
    observation_.location.latitudeDeg = preferences_.getDouble("lat", 35.681236);
    observation_.location.longitudeDeg = preferences_.getDouble("lon", 139.767125);
    observation_.location.utcOffsetMinutes = preferences_.getInt("offset", 540);
    preferences_.getString("place", observation_.locationName,
                           sizeof(observation_.locationName));
    observation_.location.name = observation_.locationName;
    observation_.limitingMagnitude = preferences_.getFloat("limit", 6.5F);
    physical_.trueNorthCorrectionDeg = preferences_.getFloat("north", -7.5F);
    preferences_.getString("ssid", wifiSsid_, sizeof(wifiSsid_));
    preferences_.getString("wifi-pass", wifiPassword_, sizeof(wifiPassword_));
    preferences_.getString("ws-host", voiceWebSocketHost_,
                           sizeof(voiceWebSocketHost_));
    if (!voiceWebSocketHost_[0]) {
      std::snprintf(voiceWebSocketHost_, sizeof(voiceWebSocketHost_), "%s",
                    kDefaultVoiceWebSocketHost);
    }
    const std::uint32_t savedPort = preferences_.getUInt(
        "ws-port", kDefaultVoiceWebSocketPort);
    voiceWebSocketPort_ = savedPort >= 1U && savedPort <= 65535U
                              ? static_cast<std::uint16_t>(savedPort)
                              : kDefaultVoiceWebSocketPort;
    preferences_.getString("ws-path", voiceWebSocketPath_,
                           sizeof(voiceWebSocketPath_));
    if (!voiceWebSocketPath_[0]) {
      std::snprintf(voiceWebSocketPath_, sizeof(voiceWebSocketPath_), "%s",
                    kDefaultVoiceWebSocketPath);
    }
    const std::uint8_t savedVoiceVolume = preferences_.getUChar(
        "voice-vol", kDefaultVoiceVolumePercent);
    voiceVolumePercent_ = savedVoiceVolume <= 100
                              ? savedVoiceVolume
                              : kDefaultVoiceVolumePercent;
    configurationMode_ = preferences_.getBool("cfgmode", false);
  }

  const ObservationContext& observation() const { return observation_; }
  const PhysicalPoseSettings& physical() const { return physical_; }
  const char* wifiSsid() const { return wifiSsid_; }
  const char* wifiPassword() const { return wifiPassword_; }
  const char* voiceWebSocketHost() const { return voiceWebSocketHost_; }
  std::uint16_t voiceWebSocketPort() const { return voiceWebSocketPort_; }
  const char* voiceWebSocketPath() const { return voiceWebSocketPath_; }
  std::uint8_t voiceVolumePercent() const { return voiceVolumePercent_; }
  bool configurationMode() const { return configurationMode_; }

  void setLocation(const Location& location) {
    observation_.location.latitudeDeg = location.latitudeDeg;
    observation_.location.longitudeDeg = location.longitudeDeg;
    observation_.location.utcOffsetMinutes = location.utcOffsetMinutes;
    std::snprintf(observation_.locationName, sizeof(observation_.locationName),
                  "%s", location.name ? location.name : "Custom");
    observation_.location.name = observation_.locationName;
    saveLocation();
  }

  void setTrueNorthCorrection(float value) {
    physical_.trueNorthCorrectionDeg =
        std::max(-180.0F, std::min(180.0F, value));
    preferences_.putFloat("north", physical_.trueNorthCorrectionDeg);
  }

  void setLimitingMagnitude(float value) {
    observation_.limitingMagnitude = value;
    preferences_.putFloat("limit", observation_.limitingMagnitude);
  }

  void setWifiCredentials(const char* ssid, const char* password) {
    std::snprintf(wifiSsid_, sizeof(wifiSsid_), "%s", ssid ? ssid : "");
    std::snprintf(wifiPassword_, sizeof(wifiPassword_), "%s",
                  password ? password : "");
    preferences_.putString("ssid", wifiSsid_);
    preferences_.putString("wifi-pass", wifiPassword_);
  }

  void setVoiceWebSocket(const char* host, std::uint16_t port,
                         const char* path) {
    std::snprintf(voiceWebSocketHost_, sizeof(voiceWebSocketHost_), "%s",
                  host ? host : "");
    voiceWebSocketPort_ = port;
    std::snprintf(voiceWebSocketPath_, sizeof(voiceWebSocketPath_), "%s",
                  path ? path : "/ws");
    preferences_.putString("ws-host", voiceWebSocketHost_);
    preferences_.putUInt("ws-port", voiceWebSocketPort_);
    preferences_.putString("ws-path", voiceWebSocketPath_);
  }

  void setVoiceVolumePercent(std::uint8_t value) {
    voiceVolumePercent_ = std::min<std::uint8_t>(value, 100);
    preferences_.putUChar("voice-vol", voiceVolumePercent_);
  }

  void setConfigurationMode(bool enabled) {
    configurationMode_ = enabled;
    preferences_.putBool("cfgmode", enabled);
  }

  void cycleMagnitude() {
    // Move from the full catalogue toward progressively brighter stars, then
    // wrap back to the full magnitude-6.5 view.
    if (observation_.limitingMagnitude >= 6.0F) {
      observation_.limitingMagnitude = 5.5F;
    } else if (observation_.limitingMagnitude >= 5.0F) {
      observation_.limitingMagnitude = 4.0F;
    } else if (observation_.limitingMagnitude >= 3.5F) {
      observation_.limitingMagnitude = 3.0F;
    } else if (observation_.limitingMagnitude >= 2.5F) {
      observation_.limitingMagnitude = 2.0F;
    } else if (observation_.limitingMagnitude >= 1.5F) {
      observation_.limitingMagnitude = 1.0F;
    } else {
      observation_.limitingMagnitude = 6.5F;
    }
    preferences_.putFloat("limit", observation_.limitingMagnitude);
  }

 private:
  void saveLocation() {
    preferences_.putDouble("lat", observation_.location.latitudeDeg);
    preferences_.putDouble("lon", observation_.location.longitudeDeg);
    preferences_.putInt("offset", observation_.location.utcOffsetMinutes);
    preferences_.putString("place", observation_.locationName);
  }

  Preferences preferences_;
  PhysicalPoseSettings physical_;
  ObservationContext observation_;
  char wifiSsid_[33]{};
  char wifiPassword_[65]{};
  char voiceWebSocketHost_[128]{};
  std::uint16_t voiceWebSocketPort_ = 8000;
  char voiceWebSocketPath_[128]{};
  std::uint8_t voiceVolumePercent_ = kDefaultVoiceVolumePercent;
  bool configurationMode_ = false;
};

struct PackedSkyVector {
  std::int16_t east = 0;
  std::int16_t north = 0;
  std::int16_t up = -32767;
  std::uint16_t color = TFT_WHITE;
  std::uint8_t radius = 1;
  std::uint8_t reserved = 0;
};

StarCatalog starCatalog;
CityCatalog cityCatalog;
ScopeOrientationProvider orientation;
VoiceAssistant voiceAssistant;
SettingsStore settings;
GestureTracker gestures;
M5Canvas scopeCanvas(&M5.Display);
PackedSkyVector* skyVectors = nullptr;

CivilTime hudUtc;
std::uint32_t lastFrameMs = 0;
std::uint32_t lastHudRtcReadMs = 0;
std::size_t skyUpdateCursor = 0;
double julianDateAnchor = 0.0;
std::uint32_t julianDateAnchorMs = 0;
bool skyCacheValid = false;
bool showHud = true;
bool redNightMode = false;
bool configurationMode = false;
bool configurationTransitionPending = false;
std::uint32_t configurationRestartAtMs = 0;
char configurationApSsid[32]{};
char configurationApPassword[24]{};
DNSServer configurationDnsServer;
WebServer configurationWebServer(80);
Vec3 sunWorld;
Vec3 moonWorld;
Vec3 planetWorld[starscope::kNakedEyePlanetCount];
float moonIlluminatedFraction = 0.0F;
std::uint32_t lastMovingBodyUpdateMs = 0;

struct VoiceSkySnapshot {
  double julianDate = 2451545.0;
  double latitudeDeg = 35.681236;
  double longitudeDeg = 139.767125;
  int utcOffsetMinutes = 540;
  std::uint32_t capturedAtMs = 0;
  char locationName[48] = "Tokyo";
};

portMUX_TYPE voiceSkySnapshotMux = portMUX_INITIALIZER_UNLOCKED;
VoiceSkySnapshot voiceSkySnapshot;

std::uint16_t displayColor(std::uint16_t color) {
  if (!redNightMode || color == TFT_BLACK) return color;

  // Preserve the original value/brightness ordering while discarding hue.
  // Capping red below full-scale plus the lower panel brightness protects dark
  // adaptation better than simply replacing every color with TFT_RED.
  const std::uint16_t red8 = ((color >> 11U) & 0x1FU) * 255U / 31U;
  const std::uint16_t green8 = ((color >> 5U) & 0x3FU) * 255U / 63U;
  const std::uint16_t blue8 = (color & 0x1FU) * 255U / 31U;
  const std::uint16_t value = std::max(red8, std::max(green8, blue8));
  std::uint16_t red5 = (value * 26U + 127U) / 255U;
  if (value > 0 && red5 == 0) red5 = 1;
  return static_cast<std::uint16_t>(red5 << 11U);
}

void setRedNightMode(bool enabled) {
  redNightMode = enabled;
  M5.Display.setBrightness(enabled ? kNightDisplayBrightness
                                   : kNormalDisplayBrightness);
}

CivilTime readRtcUtc() {
  m5::rtc_datetime_t value;
  M5.Rtc.getDateTime(&value);
  return {value.date.year, value.date.month, value.date.date, value.time.hours,
          value.time.minutes, value.time.seconds};
}

void writeRtcUtc(const CivilTime& time) {
  m5::rtc_datetime_t value;
  value.date.year = time.year;
  value.date.month = time.month;
  value.date.date = time.day;
  value.time.hours = time.hour;
  value.time.minutes = time.minute;
  value.time.seconds = time.second;
  M5.Rtc.setDateTime(value);
}

int daysInMonth(int year, int month) {
  static constexpr int days[] = {31, 28, 31, 30, 31, 30,
                                 31, 31, 30, 31, 30, 31};
  if (month == 2 && (year % 400 == 0 || (year % 4 == 0 && year % 100 != 0))) {
    return 29;
  }
  return days[std::max(1, std::min(12, month)) - 1];
}

String htmlEscape(const char* value) {
  String escaped;
  if (!value) return escaped;
  escaped.reserve(std::strlen(value) + 16);
  for (const char* cursor = value; *cursor; ++cursor) {
    switch (*cursor) {
      case '&': escaped += F("&amp;"); break;
      case '<': escaped += F("&lt;"); break;
      case '>': escaped += F("&gt;"); break;
      case '"': escaped += F("&quot;"); break;
      case '\'': escaped += F("&#39;"); break;
      default: escaped += *cursor; break;
    }
  }
  return escaped;
}

void appendJsonString(String& destination, const char* value) {
  destination += '"';
  if (value) {
    for (const char* cursor = value; *cursor; ++cursor) {
      switch (*cursor) {
        case '"': destination += F("\\\""); break;
        case '\\': destination += F("\\\\"); break;
        case '\b': destination += F("\\b"); break;
        case '\f': destination += F("\\f"); break;
        case '\n': destination += F("\\n"); break;
        case '\r': destination += F("\\r"); break;
        case '\t': destination += F("\\t"); break;
        default:
          if (static_cast<unsigned char>(*cursor) >= 0x20U) {
            destination += *cursor;
          }
          break;
      }
    }
  }
  destination += '"';
}

bool parseDoubleSetting(const char* name, double minimum, double maximum,
                        double& result) {
  if (!configurationWebServer.hasArg(name)) return false;
  const String text = configurationWebServer.arg(name);
  char* end = nullptr;
  result = std::strtod(text.c_str(), &end);
  return end && *end == '\0' && std::isfinite(result) && result >= minimum &&
         result <= maximum;
}

bool parseIntSetting(const char* name, int minimum, int maximum, int& result) {
  if (!configurationWebServer.hasArg(name)) return false;
  const String text = configurationWebServer.arg(name);
  char* end = nullptr;
  const long value = std::strtol(text.c_str(), &end, 10);
  if (!end || *end != '\0' || value < minimum || value > maximum) return false;
  result = static_cast<int>(value);
  return true;
}

bool allowedMagnitude(float value) {
  static constexpr float values[] = {1.0F, 2.0F, 3.0F,
                                     4.0F, 5.5F, 6.5F};
  for (float candidate : values) {
    if (std::fabs(value - candidate) < 0.01F) return true;
  }
  return false;
}

void drawConfigurationMessage(const char* heading, const char* detail) {
  auto& display = M5.Display;
  display.fillScreen(TFT_BLACK);
  display.setTextDatum(middle_center);
  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.drawString(heading, kCenter, 165, 4);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.drawString(detail, kCenter, 235, 2);
}

void scheduleConfigurationRestart(bool enable, std::uint32_t nowMs) {
  settings.setConfigurationMode(enable);
  configurationTransitionPending = true;
  configurationRestartAtMs = nowMs + kConfigurationRestartDelayMs;
  drawConfigurationMessage(enable ? "ENTERING SETUP" : "EXITING SETUP",
                           "Restarting...");
}

static const char kConfigurationPage[] PROGMEM = R"HTML(
<!doctype html><html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>M5 StarScope Setup</title><style>
:root{color-scheme:dark;font-family:system-ui,sans-serif}body{margin:0;background:#090b11;color:#eef2ff}
main{max-width:640px;margin:auto;padding:24px}h1{color:#79d9ff;font-size:1.7rem}fieldset{border:1px solid #354052;border-radius:12px;margin:18px 0;padding:16px}
legend{color:#9edfff;padding:0 8px}label{display:block;margin:12px 0 5px}input,select,button{box-sizing:border-box;width:100%;font:inherit;border-radius:8px;border:1px solid #526078;background:#151a24;color:#fff;padding:11px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}.note{color:#aeb8c9;font-size:.9rem}.primary{background:#086c91;border:0;font-weight:700}.secondary{background:#1b4c62}.exit{background:#44232b;border-color:#86505e}.city-results{display:grid;gap:8px;margin-top:8px}.city-option{text-align:left;background:#192536;padding:9px 11px}.city-option small{display:block;color:#aeb8c9;margin-top:2px}.city-status{min-height:1.3em}@media(max-width:520px){.grid{grid-template-columns:1fr}}
</style></head><body><main><h1>M5 StarScope Setup</h1>
<p class="note">Save settings here, then hold the Atom button for 2 seconds to return to the scope.</p>
<form method="post" action="/save" onsubmit="return prepareTime()">
<fieldset><legend>Wi-Fi for voice assistant</legend>
<label for="ssid">SSID</label><input id="ssid" name="ssid" maxlength="32" value="{{SSID}}">
<label for="wifi_password">Password</label><input id="wifi_password" name="wifi_password" type="password" maxlength="64" placeholder="Leave blank to keep the current password">
</fieldset><fieldset><legend>Voice WebSocket server</legend>
<label for="ws_host">Host</label><input id="ws_host" name="ws_host" maxlength="127" value="{{WS_HOST}}" placeholder="192.168.1.100" required>
<div class="grid"><div><label for="ws_port">Port</label><input id="ws_port" name="ws_port" type="number" min="1" max="65535" value="{{WS_PORT}}" required></div>
<div><label for="ws_path">Path</label><input id="ws_path" name="ws_path" maxlength="127" value="{{WS_PATH}}" placeholder="/ws" required></div></div>
<p class="note">Enter the host name or IP address without ws://. Use a plain WebSocket endpoint on the same trusted LAN, typically port 8000.</p>
<label for="voice_volume">Playback volume: <output id="voice_volume_value">{{VOICE_VOLUME}}%</output></label>
<input id="voice_volume" name="voice_volume" type="range" min="0" max="100" step="5" value="{{VOICE_VOLUME}}" oninput="document.getElementById('voice_volume_value').value=this.value+'%'">
</fieldset><fieldset><legend>Observation location</legend>
<label for="city_search">Search the city catalogue</label><input id="city_search" type="search" autocomplete="off" placeholder="Tokyo, Japan, JP...">
<p id="city_status" class="note city-status" aria-live="polite">Enter at least 2 letters, then choose a city.</p><div id="city_results" class="city-results"></div>
<p class="note">Choosing a city fills the fields below with its standard UTC offset. Adjust the offset manually when daylight saving time is in effect.</p>
<label for="place">Location name</label><input id="place" name="place" maxlength="47" value="{{PLACE}}" required>
<div class="grid"><div><label for="lat">Latitude</label><input id="lat" name="lat" type="number" min="-90" max="90" step="0.000001" value="{{LAT}}" required></div>
<div><label for="lon">Longitude</label><input id="lon" name="lon" type="number" min="-180" max="180" step="0.000001" value="{{LON}}" required></div></div>
<label for="utc_offset">UTC offset in minutes</label><input id="utc_offset" name="utc_offset" type="number" min="-840" max="840" value="{{OFFSET}}" required>
</fieldset><fieldset><legend>Display and heading</legend>
<label for="north">Magnetic declination / true-north correction (degrees)</label><div class="grid"><input id="north" name="north" type="number" min="-180" max="180" step="0.1" value="{{NORTH}}" required><button id="calculate_north" class="secondary" type="button">Calculate from coordinates</button></div>
<p id="north_status" class="note city-status" aria-live="polite">WMM2025 can estimate this from the physical location. Manual adjustment remains available.</p>
<label for="limit">Limiting magnitude</label><select id="limit" name="limit"><option>1.0</option><option>2.0</option><option>3.0</option><option>4.0</option><option>5.5</option><option>6.5</option></select>
</fieldset><p class="note">Saving also sets the RTC from this phone's current UTC time.</p>
<input type="hidden" id="utc_year" name="utc_year"><input type="hidden" id="utc_month" name="utc_month"><input type="hidden" id="utc_day" name="utc_day"><input type="hidden" id="utc_hour" name="utc_hour"><input type="hidden" id="utc_minute" name="utc_minute"><input type="hidden" id="utc_second" name="utc_second">
<p><button class="primary" type="submit">Save settings</button></p></form>
<form method="post" action="/exit"><button class="exit" type="submit">Exit setup mode</button></form>
</main><script>
document.getElementById('limit').value='{{LIMIT}}';
if(!document.getElementById('utc_offset').value){document.getElementById('utc_offset').value=String(-new Date().getTimezoneOffset())}
const citySearch=document.getElementById('city_search'),cityStatus=document.getElementById('city_status'),cityResults=document.getElementById('city_results'),northStatus=document.getElementById('north_status');let cityTimer=0,cityRequest=0,declinationRequest=0;
function phoneDecimalYear(){const now=new Date(),year=now.getUTCFullYear(),start=Date.UTC(year,0,1),end=Date.UTC(year+1,0,1);return year+(now.getTime()-start)/(end-start)}
async function calculateDeclination(){const latitude=document.getElementById('lat').valueAsNumber,longitude=document.getElementById('lon').valueAsNumber,year=phoneDecimalYear(),request=++declinationRequest;if(!Number.isFinite(latitude)||latitude < -90||latitude > 90||!Number.isFinite(longitude)||longitude < -180||longitude > 180){northStatus.textContent='Enter valid coordinates before calculating.';return}northStatus.textContent='Calculating WMM2025 estimate...';try{const response=await fetch('/declination?lat='+encodeURIComponent(latitude)+'&lon='+encodeURIComponent(longitude)+'&year='+encodeURIComponent(year),{cache:'no-store'});if(!response.ok)throw new Error();const data=await response.json();if(request!==declinationRequest)return;document.getElementById('north').value=data.declination.toFixed(1);const direction=data.declination>=0?'east':'west';northStatus.textContent='WMM2025 estimate for '+data.year.toFixed(2)+': '+data.declination.toFixed(1)+' degrees '+direction+'.'+(data.caution?' Compass accuracy may be reduced near the magnetic poles.':'') }catch(error){if(request===declinationRequest)northStatus.textContent='Could not calculate a WMM2025 estimate for this date. Keep or enter the correction manually.'}}
function chooseCity(city){document.getElementById('place').value=city.name;document.getElementById('lat').value=city.lat.toFixed(5);document.getElementById('lon').value=city.lon.toFixed(5);document.getElementById('utc_offset').value=String(city.offset);citySearch.value=city.name+', '+city.country;cityResults.replaceChildren();cityStatus.textContent='Selected '+city.name+', '+city.country+'.';calculateDeclination()}
async function searchCities(){const query=citySearch.value.trim();const request=++cityRequest;cityResults.replaceChildren();if(query.length<2){cityStatus.textContent='Enter at least 2 letters, then choose a city.';return}cityStatus.textContent='Searching...';try{const response=await fetch('/cities?q='+encodeURIComponent(query),{cache:'no-store'});if(!response.ok)throw new Error();const data=await response.json();if(request!==cityRequest)return;for(const city of data.results){const button=document.createElement('button');button.type='button';button.className='city-option';button.textContent=city.name;const detail=document.createElement('small');detail.textContent=city.country+' ('+city.code+') · '+city.lat.toFixed(5)+', '+city.lon.toFixed(5)+' · UTC '+(city.offset>=0?'+':'')+(city.offset/60);button.append(detail);button.addEventListener('click',()=>chooseCity(city));cityResults.append(button)}cityStatus.textContent=data.results.length?(data.more?'More matches available; keep typing to narrow the list.':'Choose a matching city.'):'No matching city found. You can still enter the fields manually.'}catch(error){if(request===cityRequest)cityStatus.textContent='City search failed. You can still enter the fields manually.'}}
citySearch.addEventListener('input',()=>{++cityRequest;clearTimeout(cityTimer);cityTimer=setTimeout(searchCities,180)});citySearch.addEventListener('keydown',event=>{if(event.key==='Enter')event.preventDefault()});
document.getElementById('calculate_north').addEventListener('click',calculateDeclination);
function prepareTime(){const d=new Date();for(const [id,v] of [['utc_year',d.getUTCFullYear()],['utc_month',d.getUTCMonth()+1],['utc_day',d.getUTCDate()],['utc_hour',d.getUTCHours()],['utc_minute',d.getUTCMinutes()],['utc_second',d.getUTCSeconds()]])document.getElementById(id).value=String(v);return true}
</script></body></html>)HTML";

void sendConfigurationPage() {
  String page = FPSTR(kConfigurationPage);
  const auto& observation = settings.observation();
  page.replace("{{SSID}}", htmlEscape(settings.wifiSsid()));
  page.replace("{{WS_HOST}}", htmlEscape(settings.voiceWebSocketHost()));
  page.replace("{{WS_PORT}}", String(settings.voiceWebSocketPort()));
  page.replace("{{WS_PATH}}", htmlEscape(settings.voiceWebSocketPath()));
  page.replace("{{VOICE_VOLUME}}", String(settings.voiceVolumePercent()));
  page.replace("{{PLACE}}", htmlEscape(observation.locationName));
  page.replace("{{LAT}}", String(observation.location.latitudeDeg, 6));
  page.replace("{{LON}}", String(observation.location.longitudeDeg, 6));
  page.replace("{{OFFSET}}", String(observation.location.utcOffsetMinutes));
  page.replace("{{NORTH}}", String(settings.physical().trueNorthCorrectionDeg, 1));
  page.replace("{{LIMIT}}", String(observation.limitingMagnitude, 1));
  configurationWebServer.sendHeader("Cache-Control", "no-store");
  configurationWebServer.send(200, "text/html; charset=utf-8", page);
}

void sendConfigurationError(const char* message) {
  String page = F("<!doctype html><meta name=viewport content='width=device-width'>"
                  "<title>Invalid settings</title><h1>Invalid settings</h1><p>");
  page += htmlEscape(message);
  page += F("</p><p><a href='/'>Back</a></p>");
  configurationWebServer.send(400, "text/html; charset=utf-8", page);
}

void sendCitySearchResults() {
  String query = configurationWebServer.arg("q");
  query.trim();
  if (query.length() < 2 || query.length() > 64) {
    configurationWebServer.send(
        400, "application/json; charset=utf-8",
        F("{\"error\":\"Query must contain 2 to 64 characters.\"}"));
    return;
  }

  String response;
  response.reserve(4096);
  response = F("{\"results\":[");
  std::size_t resultCount = 0;
  bool more = false;
  for (std::size_t countryIndex = 0;
       countryIndex < cityCatalog.countryCount() && !more; ++countryIndex) {
    const auto& country = cityCatalog.country(countryIndex);
    for (std::size_t cityIndex = 0; cityIndex < country.cityCount; ++cityIndex) {
      const auto& city = cityCatalog.city(country, cityIndex);
      if (!cityCatalog.matches(country, city, query.c_str())) continue;
      if (resultCount >= kCitySearchResultLimit) {
        more = true;
        break;
      }
      if (resultCount++) response += ',';
      response += F("{\"name\":");
      appendJsonString(response, cityCatalog.cityName(city));
      response += F(",\"country\":");
      appendJsonString(response, cityCatalog.countryName(country));
      response += F(",\"code\":");
      appendJsonString(response, country.code);
      response += F(",\"lat\":");
      response += String(city.latitudeE5 / 100000.0, 5);
      response += F(",\"lon\":");
      response += String(city.longitudeE5 / 100000.0, 5);
      response += F(",\"offset\":");
      response += String(city.standardUtcOffsetMinutes);
      response += '}';
    }
  }
  response += F("],\"more\":");
  response += more ? F("true}") : F("false}");
  configurationWebServer.sendHeader("Cache-Control", "no-store");
  configurationWebServer.send(200, "application/json; charset=utf-8", response);
}

void sendDeclinationEstimate() {
  double latitude = 0.0;
  double longitude = 0.0;
  double year = 0.0;
  if (!parseDoubleSetting("lat", -90.0, 90.0, latitude) ||
      !parseDoubleSetting("lon", -180.0, 180.0, longitude) ||
      !parseDoubleSetting("year", starscope::geomagnetism::kWmm2025Epoch,
                          starscope::geomagnetism::kWmm2025ValidUntil - 1.0e-6,
                          year)) {
    configurationWebServer.send(
        400, "application/json; charset=utf-8",
        F("{\"error\":\"Coordinates or date are outside the WMM2025 range.\"}"));
    return;
  }

  const auto field = starscope::geomagnetism::worldMagneticModel2025(
      latitude, longitude, year);
  if (!field.valid) {
    configurationWebServer.send(
        422, "application/json; charset=utf-8",
        F("{\"error\":\"WMM2025 could not estimate this location.\"}"));
    return;
  }

  String response;
  response.reserve(160);
  response = F("{\"declination\":");
  response += String(field.declinationDeg, 2);
  response += F(",\"year\":");
  response += String(year, 6);
  response += F(",\"caution\":");
  response += field.horizontalIntensityNanoTesla < 6000.0 ? F("true}")
                                                            : F("false}");
  configurationWebServer.sendHeader("Cache-Control", "no-store");
  configurationWebServer.send(200, "application/json; charset=utf-8", response);
}

void handleConfigurationSave() {
  double latitude = 0.0;
  double longitude = 0.0;
  double north = 0.0;
  double magnitude = 0.0;
  int utcOffset = 0;
  int webSocketPort = 0;
  int voiceVolume = 0;
  if (!parseDoubleSetting("lat", -90.0, 90.0, latitude) ||
      !parseDoubleSetting("lon", -180.0, 180.0, longitude) ||
      !parseIntSetting("utc_offset", -840, 840, utcOffset) ||
      !parseIntSetting("ws_port", 1, 65535, webSocketPort) ||
      !parseIntSetting("voice_volume", 0, 100, voiceVolume) ||
      !parseDoubleSetting("north", -180.0, 180.0, north) ||
      !parseDoubleSetting("limit", 1.0, 6.5, magnitude) ||
      !allowedMagnitude(static_cast<float>(magnitude))) {
    sendConfigurationError("One or more numeric values are outside the allowed range.");
    return;
  }

  const String place = configurationWebServer.arg("place");
  const String ssid = configurationWebServer.arg("ssid");
  const String submittedPassword = configurationWebServer.arg("wifi_password");
  const String webSocketHost = configurationWebServer.arg("ws_host");
  const String webSocketPath = configurationWebServer.arg("ws_path");
  if (place.isEmpty() || place.length() > 47 || ssid.length() > 32 ||
      submittedPassword.length() > 64 || webSocketHost.isEmpty() ||
      webSocketHost.length() > 127 || webSocketPath.isEmpty() ||
      webSocketPath.length() > 127 || webSocketPath[0] != '/') {
    sendConfigurationError(
        "Location, Wi-Fi, or WebSocket settings are invalid.");
    return;
  }

  CivilTime submittedUtc;
  if (!parseIntSetting("utc_year", 2000, 2099, submittedUtc.year) ||
      !parseIntSetting("utc_month", 1, 12, submittedUtc.month) ||
      !parseIntSetting("utc_day", 1, 31, submittedUtc.day) ||
      !parseIntSetting("utc_hour", 0, 23, submittedUtc.hour) ||
      !parseIntSetting("utc_minute", 0, 59, submittedUtc.minute) ||
      !parseIntSetting("utc_second", 0, 59, submittedUtc.second) ||
      submittedUtc.day > daysInMonth(submittedUtc.year, submittedUtc.month)) {
    sendConfigurationError("The phone supplied an invalid UTC time.");
    return;
  }

  const bool sameSsid = ssid.equals(settings.wifiSsid());
  if (!sameSsid || !submittedPassword.isEmpty()) {
    settings.setWifiCredentials(ssid.c_str(), submittedPassword.c_str());
  }
  settings.setVoiceWebSocket(webSocketHost.c_str(),
                             static_cast<std::uint16_t>(webSocketPort),
                             webSocketPath.c_str());
  settings.setVoiceVolumePercent(static_cast<std::uint8_t>(voiceVolume));
  settings.setLocation({latitude, longitude, utcOffset, place.c_str()});
  settings.setTrueNorthCorrection(static_cast<float>(north));
  settings.setLimitingMagnitude(static_cast<float>(magnitude));

  writeRtcUtc(submittedUtc);

  drawConfigurationMessage("SETTINGS SAVED", "Hold Atom to exit");
  configurationWebServer.send(
      200, "text/html; charset=utf-8",
      F("<!doctype html><meta name=viewport content='width=device-width'>"
        "<title>Saved</title><h1>Settings saved</h1>"
        "<p>Hold the Atom button for 2 seconds to return to the scope.</p>"
        "<p><a href='/'>Edit again</a></p>"));
}

void handleConfigurationExit() {
  configurationWebServer.send(
      200, "text/html; charset=utf-8",
      F("<!doctype html><meta name=viewport content='width=device-width'>"
        "<title>Exiting</title><h1>Exiting setup</h1>"
        "<p>The telescope is restarting.</p>"));
  scheduleConfigurationRestart(false, millis());
}

bool beginConfigurationPortal() {
  const unsigned long long suffix =
      static_cast<unsigned long long>(ESP.getEfuseMac() & 0xFFFFFFULL);
  std::snprintf(configurationApSsid, sizeof(configurationApSsid),
                "M5StarScope-%06llX", suffix);
  std::snprintf(configurationApPassword, sizeof(configurationApPassword),
                "scope-%06llX", suffix);

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  const IPAddress address(192, 168, 4, 1);
  const IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(address, address, subnet);
  if (!WiFi.softAP(configurationApSsid, configurationApPassword)) return false;

  configurationDnsServer.start(53, "*", address);
  configurationWebServer.on("/", HTTP_GET, sendConfigurationPage);
  configurationWebServer.on("/cities", HTTP_GET, sendCitySearchResults);
  configurationWebServer.on("/declination", HTTP_GET,
                            sendDeclinationEstimate);
  configurationWebServer.on("/save", HTTP_POST, handleConfigurationSave);
  configurationWebServer.on("/exit", HTTP_POST, handleConfigurationExit);
  configurationWebServer.onNotFound(sendConfigurationPage);
  configurationWebServer.begin();

  auto& display = M5.Display;
  display.fillScreen(TFT_BLACK);
  display.setTextDatum(middle_center);
  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.drawString("SMARTPHONE SETUP", kCenter, 40, 4);

  display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  display.drawString("WI-FI SSID", kCenter, 96, 2);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.drawString(configurationApSsid, kCenter, 135, 4);

  display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  display.drawString("PASSWORD", kCenter, 190, 2);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.drawString(configurationApPassword, kCenter, 229, 4);

  display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  display.drawString("OPEN IN BROWSER", kCenter, 284, 2);
  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.drawString("http://192.168.4.1", kCenter, 323, 4);

  display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  display.drawString("Hold Atom 2 sec to exit", kCenter, 414, 2);
  return true;
}

double currentJulianDate() {
  return starscope::astronomy::julianDateUtc(readRtcUtc(), 0);
}

void synchronizeJulianClock(std::uint32_t nowMs) {
  julianDateAnchor = currentJulianDate();
  julianDateAnchorMs = nowMs;
}

double estimatedJulianDate(std::uint32_t nowMs) {
  constexpr double kMillisecondsPerDay = 86400000.0;
  return julianDateAnchor +
         static_cast<std::uint32_t>(nowMs - julianDateAnchorMs) /
             kMillisecondsPerDay;
}

void refreshVoiceSkySnapshot(std::uint32_t nowMs) {
  VoiceSkySnapshot next;
  const auto& observation = settings.observation();
  next.julianDate = estimatedJulianDate(nowMs);
  next.latitudeDeg = observation.location.latitudeDeg;
  next.longitudeDeg = observation.location.longitudeDeg;
  next.utcOffsetMinutes = observation.location.utcOffsetMinutes;
  next.capturedAtMs = nowMs;
  std::snprintf(next.locationName, sizeof(next.locationName), "%s",
                observation.locationName);

  portENTER_CRITICAL(&voiceSkySnapshotMux);
  voiceSkySnapshot = next;
  portEXIT_CRITICAL(&voiceSkySnapshotMux);
}

class BoundedJsonWriter {
 public:
  BoundedJsonWriter(char* destination, std::size_t capacity)
      : destination_(destination), capacity_(capacity) {
    if (destination_ && capacity_ > 0) {
      destination_[0] = '\0';
    } else {
      valid_ = false;
    }
  }

  bool append(const char* text) {
    if (!valid_ || !text) return false;
    const std::size_t length = std::strlen(text);
    if (length >= capacity_ - size_) return fail();
    std::memcpy(destination_ + size_, text, length);
    size_ += length;
    destination_[size_] = '\0';
    return true;
  }

  bool appendChar(char value) {
    if (!valid_ || capacity_ - size_ <= 1) return fail();
    destination_[size_++] = value;
    destination_[size_] = '\0';
    return true;
  }

  bool appendFormat(const char* format, ...) {
    if (!valid_ || !format || capacity_ <= size_) return fail();
    va_list arguments;
    va_start(arguments, format);
    const int written = std::vsnprintf(destination_ + size_,
                                       capacity_ - size_, format, arguments);
    va_end(arguments);
    if (written < 0 ||
        static_cast<std::size_t>(written) >= capacity_ - size_) {
      return fail();
    }
    size_ += static_cast<std::size_t>(written);
    return true;
  }

  bool appendJsonString(const char* value) {
    if (!appendChar('"')) return false;
    for (const unsigned char* cursor =
             reinterpret_cast<const unsigned char*>(value ? value : "");
         *cursor; ++cursor) {
      if (*cursor == '"' || *cursor == '\\') {
        if (!appendChar('\\') || !appendChar(static_cast<char>(*cursor))) {
          return false;
        }
      } else if (*cursor < 0x20U) {
        if (!appendFormat("\\u%04x", static_cast<unsigned>(*cursor))) {
          return false;
        }
      } else if (!appendChar(static_cast<char>(*cursor))) {
        return false;
      }
    }
    return appendChar('"');
  }

  std::size_t size() const { return valid_ ? size_ : 0; }

 private:
  bool fail() {
    valid_ = false;
    if (destination_ && capacity_ > 0) destination_[capacity_ - 1] = '\0';
    return false;
  }

  char* destination_ = nullptr;
  std::size_t capacity_ = 0;
  std::size_t size_ = 0;
  bool valid_ = true;
};

std::size_t writeVoiceSkyContext(char* destination, std::size_t capacity,
                                 std::uint32_t nowMs, void*) {
  VoiceSkySnapshot snapshot;
  portENTER_CRITICAL(&voiceSkySnapshotMux);
  snapshot = voiceSkySnapshot;
  portEXIT_CRITICAL(&voiceSkySnapshotMux);

  constexpr double kMillisecondsPerDay = 86400000.0;
  const double julianDate =
      snapshot.julianDate +
      static_cast<std::uint32_t>(nowMs - snapshot.capturedAtMs) /
          kMillisecondsPerDay;
  const CivilTime utc = starscope::astronomy::civilTimeUtc(julianDate);
  const Location location{snapshot.latitudeDeg, snapshot.longitudeDeg,
                          snapshot.utcOffsetMinutes, snapshot.locationName};

  BoundedJsonWriter json(destination, capacity);
  char timestamp[32];
  std::snprintf(timestamp, sizeof(timestamp),
                "%04d-%02d-%02dT%02d:%02d:%02dZ", utc.year, utc.month,
                utc.day, utc.hour, utc.minute, utc.second);
  json.append("{\"observed_at_utc\":");
  json.appendJsonString(timestamp);
  json.append(",\"location\":{\"name\":");
  json.appendJsonString(snapshot.locationName);
  json.appendFormat(
      ",\"latitude_deg\":%.6f,\"longitude_deg\":%.6f,"
      "\"utc_offset_minutes\":%d},"
      "\"catalog\":\"HYG-4.1\",\"maximum_magnitude\":%.1f,"
      "\"minimum_altitude_deg\":0.0,"
      "\"star_fields\":[\"name\",\"object_type\",\"hip_id\","
      "\"azimuth_deg\",\"altitude_deg\",\"magnitude\"],\"stars\":[",
      snapshot.latitudeDeg, snapshot.longitudeDeg, snapshot.utcOffsetMinutes,
      static_cast<double>(kVoiceSkyMagnitudeLimit));

  bool first = true;
  const auto appendMovingObject =
      [&](const char* name, const char* objectType,
          const starscope::EquatorialCoordinate& equatorial) {
        const auto horizontal = starscope::astronomy::equatorialToHorizontal(
            equatorial.rightAscensionDeg, equatorial.declinationDeg,
            julianDate, location);
        if (horizontal.altitudeDeg <= 0.0F) return;
        if (!first) json.appendChar(',');
        first = false;
        json.appendChar('[');
        json.appendJsonString(name);
        json.appendChar(',');
        json.appendJsonString(objectType);
        json.appendFormat(",null,%.1f,%.1f,null]",
                          static_cast<double>(horizontal.azimuthDeg),
                          static_cast<double>(horizontal.altitudeDeg));
      };

  const auto ephemeris =
      starscope::astronomy::solarSystemEphemeris(julianDate);
  appendMovingObject("Sun", "sun", ephemeris.sun);
  appendMovingObject("Moon", "moon", ephemeris.moon);
  static constexpr const char* kPlanetNames[] = {
      "Mercury", "Venus", "Mars", "Jupiter", "Saturn",
  };
  for (std::size_t index = 0; index < starscope::kNakedEyePlanetCount;
       ++index) {
    appendMovingObject(kPlanetNames[index], "planet",
                       ephemeris.planets[index].equatorial);
  }

  for (std::size_t index = 0; index < starCatalog.size(); ++index) {
    const auto& packed = starCatalog.packed(index);
    if (starCatalog.isSolarPlaceholder(packed)) continue;
    const float magnitude = starCatalog.magnitude(packed);
    if (magnitude > kVoiceSkyMagnitudeLimit) break;
    const auto horizontal = starscope::astronomy::equatorialToHorizontal(
        starCatalog.rightAscensionDeg(packed),
        starCatalog.declinationDeg(packed), julianDate, location);
    if (horizontal.altitudeDeg <= 0.0F) continue;

    if (!first) json.appendChar(',');
    first = false;
    json.appendChar('[');
    const char* name = starCatalog.name(packed);
    if (name) {
      json.appendJsonString(name);
    } else {
      json.append("null");
    }
    json.appendFormat(",\"star\",%lu,%.1f,%.1f,%.2f]",
                      static_cast<unsigned long>(packed.hipId),
                      static_cast<double>(horizontal.azimuthDeg),
                      static_cast<double>(horizontal.altitudeDeg),
                      static_cast<double>(magnitude));
  }
  json.append("]}");
  return json.size();
}

std::uint16_t starColor(float bv, float magnitude) {
  auto rgb = starscope::astronomy::colorFromBv(bv);
  const float brightness = magnitude <= 1.0F ? 1.0F
                           : magnitude <= 2.0F ? 0.94F
                           : magnitude <= 3.0F ? 0.78F
                           : magnitude <= 4.0F ? 0.62F
                           : magnitude <= 5.0F ? 0.42F
                                               : 0.26F;
  rgb.red = static_cast<std::uint8_t>(rgb.red * brightness);
  rgb.green = static_cast<std::uint8_t>(rgb.green * brightness);
  rgb.blue = static_cast<std::uint8_t>(rgb.blue * brightness);
  return M5.Display.color565(rgb.red, rgb.green, rgb.blue);
}

std::uint16_t dimColor(std::uint16_t color) {
  const std::uint16_t red = ((color >> 11U) & 0x1FU) / 4U;
  const std::uint16_t green = ((color >> 5U) & 0x3FU) / 4U;
  const std::uint16_t blue = (color & 0x1FU) / 4U;
  return static_cast<std::uint16_t>((red << 11U) | (green << 5U) | blue);
}

const char* cardinal(float heading) {
  static constexpr const char* names[] = {"N", "NE", "E", "SE",
                                           "S", "SW", "W", "NW"};
  return names[static_cast<int>((heading + 22.5F) / 45.0F) & 7];
}

std::uint8_t starRadius(float magnitude) {
  return magnitude <= 0.0F ? 8
         : magnitude <= 1.0F ? 7
         : magnitude <= 2.0F ? 6
         : magnitude <= 3.0F ? 4
         : magnitude <= 4.0F ? 3
         : magnitude <= 5.0F ? 2
                             : 1;
}

void updateSkyVector(std::size_t index, double julianDate,
                     const Location& location) {
  const auto& packed = starCatalog.packed(index);
  if (starCatalog.isSolarPlaceholder(packed)) return;
  const Vec3 world = starscope::astronomy::equatorialToWorld(
      starCatalog.rightAscensionDeg(packed),
      starCatalog.declinationDeg(packed), julianDate, location);
  skyVectors[index].east =
      static_cast<std::int16_t>(std::lround(world.x * 32767));
  skyVectors[index].north =
      static_cast<std::int16_t>(std::lround(world.y * 32767));
  skyVectors[index].up =
      static_cast<std::int16_t>(std::lround(world.z * 32767));
}

void updateMovingBodyCache(std::uint32_t nowMs, bool force = false) {
  if (!force && nowMs - lastMovingBodyUpdateMs < kMovingBodyUpdateMs) return;
  const double jd = estimatedJulianDate(nowMs);
  const auto& location = settings.observation().location;
  const auto ephemeris = starscope::astronomy::solarSystemEphemeris(jd);
  sunWorld = starscope::astronomy::equatorialToWorld(
      ephemeris.sun.rightAscensionDeg, ephemeris.sun.declinationDeg, jd,
      location);
  moonWorld = starscope::astronomy::equatorialToWorld(
      ephemeris.moon.rightAscensionDeg, ephemeris.moon.declinationDeg, jd,
      location);
  for (std::size_t i = 0; i < starscope::kNakedEyePlanetCount; ++i) {
    planetWorld[i] = starscope::astronomy::equatorialToWorld(
        ephemeris.planets[i].equatorial.rightAscensionDeg,
        ephemeris.planets[i].equatorial.declinationDeg, jd, location);
  }
  moonIlluminatedFraction = ephemeris.moonIlluminatedFraction;
  lastMovingBodyUpdateMs = nowMs;
}

void rebuildSkyCache(std::uint32_t nowMs) {
  if (!skyVectors) return;
  const double jd = estimatedJulianDate(nowMs);
  const auto& location = settings.observation().location;
  for (std::size_t i = 0; i < starCatalog.size(); ++i) {
    const auto& packed = starCatalog.packed(i);
    if (starCatalog.isSolarPlaceholder(packed)) {
      skyVectors[i].radius = 0;
      continue;
    }
    updateSkyVector(i, jd, location);
    const float magnitude = starCatalog.magnitude(packed);
    skyVectors[i].color =
        starColor(starCatalog.colorIndex(packed), magnitude);
    skyVectors[i].radius = starRadius(magnitude);
  }
  skyUpdateCursor = 0;
  skyCacheValid = true;
  updateMovingBodyCache(nowMs, true);
}

void updateSkyCacheIncremental(std::uint32_t nowMs) {
  if (!skyCacheValid || !skyVectors || starCatalog.size() == 0) return;
  const double jd = estimatedJulianDate(nowMs);
  const auto& location = settings.observation().location;
  for (std::size_t count = 0; count < kSkyStarsPerFrame; ++count) {
    updateSkyVector(skyUpdateCursor, jd, location);
    skyUpdateCursor = (skyUpdateCursor + 1) % starCatalog.size();
  }
}

ScopeBasis currentScope(OrientationSample& correctedPose) {
  correctedPose = orientation.latest();
  correctedPose.sensorToWorld = starscope::applyWorldHeadingCorrection(
      correctedPose.sensorToWorld,
      settings.physical().trueNorthCorrectionDeg);
  starscope::quaternion::toEulerDeg(correctedPose.sensorToWorld,
                                   correctedPose.yawDeg,
                                   correctedPose.pitchDeg,
                                   correctedPose.rollDeg);
  const Quaternion tubeToSensor = starscope::quaternion::fromEulerDeg(
      kMountYawDeg, kMountPitchDeg, kMountRollDeg);
  return starscope::scopeBasis(correctedPose.sensorToWorld, tubeToSensor);
}

bool projectWorldToScope(const Vec3& world, const ScopeBasis& scope,
                         float tangent, int& x, int& y,
                         float maximumRadius = 1.0F) {
  const float forward = starscope::quaternion::dot(world, scope.forward);
  if (forward <= 0.0F) return false;
  const float normalizedX =
      starscope::quaternion::dot(world, scope.right) / (forward * tangent);
  const float normalizedY =
      starscope::quaternion::dot(world, scope.up) / (forward * tangent);
  if (normalizedX * normalizedX + normalizedY * normalizedY >
      maximumRadius * maximumRadius) {
    return false;
  }
  x = kCenter + static_cast<int>(normalizedX * kScopeRadius);
  y = kCenter - static_cast<int>(normalizedY * kScopeRadius);
  return true;
}

void drawHorizonAndCardinals(const ScopeBasis& scope, float tangent) {
  // Sample the astronomical horizon as a great circle so it rolls naturally
  // with the physical tube instead of remaining horizontal on the LCD.
  bool previousVisible = false;
  int previousX = 0;
  int previousY = 0;
  for (int azimuth = 0; azimuth <= 360; azimuth += 2) {
    const Vec3 world =
        starscope::horizontalToWorld({static_cast<float>(azimuth), 0.0F});
    int x = 0;
    int y = 0;
    const bool visible =
        projectWorldToScope(world, scope, tangent, x, y, 0.995F);
    if (visible && previousVisible) {
      scopeCanvas.drawLine(previousX, previousY, x, y,
                           displayColor(0x2945));
    }
    previousVisible = visible;
    previousX = x;
    previousY = y;
  }

  static constexpr const char* kCompassLabels[] = {
      "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
      "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  scopeCanvas.setTextDatum(middle_center);
  scopeCanvas.setTextColor(displayColor(0x632C), TFT_BLACK);
  for (int index = 0; index < 16; ++index) {
    // A small positive altitude keeps text just above the ground line.
    const Vec3 world = starscope::horizontalToWorld(
        {index * 22.5F, 1.3F});
    int x = 0;
    int y = 0;
    if (projectWorldToScope(world, scope, tangent, x, y, 0.90F)) {
      scopeCanvas.drawString(kCompassLabels[index], x, y, 2);
    }
  }
}

void drawScope() {
  if (!skyCacheValid || !skyVectors) return;
  OrientationSample pose;
  const ScopeBasis scope = currentScope(pose);
  const float tangent = std::tan(kFieldOfViewDeg * 3.14159265358979323846F /
                                 360.0F);
  const float magnitudeLimit = settings.observation().limitingMagnitude;
  char closestTarget[72]{};
  std::uint16_t closestTargetColor = TFT_WHITE;
  float closestTargetForward =
      std::cos(12.0F * 3.14159265358979323846F / 180.0F);

  scopeCanvas.fillScreen(TFT_BLACK);
  scopeCanvas.drawCircle(kCenter, kCenter, kScopeRadius,
                         displayColor(0x3186));
  drawHorizonAndCardinals(scope, tangent);

  for (std::size_t i = 0; i < starCatalog.size(); ++i) {
    const auto& packed = starCatalog.packed(i);
    if (starCatalog.isSolarPlaceholder(packed)) continue;
    const float magnitude = starCatalog.magnitude(packed);
    if (magnitude > magnitudeLimit) break;
    const PackedSkyVector& cached = skyVectors[i];
    if (cached.up < -200) continue;  // The Earth still blocks the lower sky.
    const Vec3 world{cached.east / 32767.0F, cached.north / 32767.0F,
                     cached.up / 32767.0F};
    const float forward = starscope::quaternion::dot(world, scope.forward);
    if (forward <= 0.0F) continue;
    const float normalizedX =
        starscope::quaternion::dot(world, scope.right) / (forward * tangent);
    const float normalizedY =
        starscope::quaternion::dot(world, scope.up) / (forward * tangent);
    if (normalizedX * normalizedX + normalizedY * normalizedY > 1.0F) continue;

    const int x = kCenter + static_cast<int>(normalizedX * kScopeRadius);
    const int y = kCenter - static_cast<int>(normalizedY * kScopeRadius);
    const int radius = cached.radius;
    const std::uint16_t color = displayColor(cached.color);
    if (magnitude <= 2.0F) {
      const int haloRadius = radius + (magnitude <= 0.0F ? 4 : 3);
      scopeCanvas.fillCircle(x, y, haloRadius, dimColor(color));
    }
    scopeCanvas.fillCircle(x, y, radius, color);
    if (magnitude <= 2.0F) {
      scopeCanvas.fillCircle(x, y, magnitude <= 1.0F ? 2 : 1,
                             displayColor(TFT_WHITE));
    }

    const char* name = starCatalog.name(packed);
    if (name && magnitude <= 1.7F &&
        normalizedX * normalizedX + normalizedY * normalizedY < 0.55F) {
      scopeCanvas.setTextDatum(middle_left);
      scopeCanvas.setTextColor(displayColor(0xBDF7), TFT_BLACK);
      scopeCanvas.drawString(name, x + radius + 5, y, 2);
    }
    if (name && forward > closestTargetForward) {
      closestTargetForward = forward;
      closestTargetColor = cached.color;
      std::snprintf(closestTarget, sizeof(closestTarget), "%s  mag %.1f", name,
                    magnitude);
    }
  }

  const auto drawMovingBody = [&](const Vec3& world, const char* name,
                                  std::uint16_t rawColor, int radius,
                                  const char* targetText) {
    // A center below the geometric horizon is blocked by the Earth. Keeping a
    // tiny tolerance avoids flicker from the packed/float boundary at 0 deg.
    if (world.z < -0.005F) return;
    int x = 0;
    int y = 0;
    if (!projectWorldToScope(world, scope, tangent, x, y)) return;
    const float forward = starscope::quaternion::dot(world, scope.forward);
    const std::uint16_t color = displayColor(rawColor);
    scopeCanvas.fillCircle(x, y, radius + 5, dimColor(color));
    scopeCanvas.fillCircle(x, y, radius, color);
    scopeCanvas.drawCircle(x, y, radius + 1, displayColor(TFT_WHITE));
    scopeCanvas.setTextDatum(middle_left);
    scopeCanvas.setTextColor(color, TFT_BLACK);
    scopeCanvas.drawString(name, x + radius + 6, y, 2);
    if (forward > closestTargetForward) {
      closestTargetForward = forward;
      closestTargetColor = rawColor;
      std::snprintf(closestTarget, sizeof(closestTarget), "%s", targetText);
    }
  };

  static constexpr const char* kPlanetNames[] = {
      "MERCURY", "VENUS", "MARS", "JUPITER", "SATURN"};
  static constexpr std::uint8_t kPlanetRgb[][3] = {
      {170, 170, 165}, {245, 245, 225}, {245, 75, 35},
      {240, 205, 145}, {225, 190, 95}};
  static constexpr int kPlanetRadii[] = {4, 7, 6, 9, 7};
  for (std::size_t i = 0; i < starscope::kNakedEyePlanetCount; ++i) {
    const std::uint16_t color = M5.Display.color565(
        kPlanetRgb[i][0], kPlanetRgb[i][1], kPlanetRgb[i][2]);
    drawMovingBody(planetWorld[i], kPlanetNames[i], color, kPlanetRadii[i],
                   kPlanetNames[i]);
  }

  const std::uint16_t sunColor = M5.Display.color565(255, 190, 20);
  drawMovingBody(sunWorld, "SUN", sunColor, 10, "SUN");
  const int moonBrightness = static_cast<int>(
      55.0F + 200.0F * std::sqrt(moonIlluminatedFraction));
  const std::uint16_t moonColor = M5.Display.color565(
      moonBrightness, moonBrightness, std::max(45, moonBrightness - 12));
  char moonTarget[32];
  std::snprintf(moonTarget, sizeof(moonTarget), "MOON  %d%%",
                static_cast<int>(moonIlluminatedFraction * 100.0F + 0.5F));
  drawMovingBody(moonWorld, "MOON", moonColor, 8, moonTarget);

  scopeCanvas.drawCircle(kCenter, kCenter, 12, displayColor(0x632C));
  scopeCanvas.drawFastHLine(kCenter - 18, kCenter, 36,
                            displayColor(0x632C));
  scopeCanvas.drawFastVLine(kCenter, kCenter - 18, 36,
                            displayColor(0x632C));

  if (showHud) {
    const std::uint16_t hudBackground = displayColor(0x1082);
    scopeCanvas.fillRoundRect(56, 7, 354, 66, 14, hudBackground);
    scopeCanvas.setTextDatum(middle_center);
    scopeCanvas.setTextColor(displayColor(TFT_WHITE), hudBackground);
    char heading[64];
    std::snprintf(heading, sizeof(heading),
                  "%s %03d  ALT %+03d  LIM %.1f",
                  cardinal(pose.yawDeg), static_cast<int>(pose.yawDeg),
                  static_cast<int>(pose.pitchDeg), magnitudeLimit);
    scopeCanvas.drawString(heading, kCenter, 25, 2);
    char clock[40];
    std::snprintf(clock, sizeof(clock), "UTC %04d/%02d/%02d %02d:%02d",
                  hudUtc.year, hudUtc.month, hudUtc.day, hudUtc.hour,
                  hudUtc.minute);
    scopeCanvas.setTextColor(displayColor(TFT_LIGHTGREY), hudBackground);
    scopeCanvas.drawString(clock, kCenter, 52, 2);

    scopeCanvas.fillRoundRect(38, 394, 390, 58, 16, hudBackground);
    scopeCanvas.setTextDatum(middle_center);
    if (closestTarget[0] != '\0') {
      scopeCanvas.setTextColor(displayColor(closestTargetColor), hudBackground);
      scopeCanvas.drawString(closestTarget, kCenter, 423, 4);
    } else {
      scopeCanvas.setTextColor(displayColor(TFT_DARKGREY), hudBackground);
      scopeCanvas.drawString("CENTER A STAR / BODY", kCenter, 423, 4);
    }
    scopeCanvas.fillCircle(
        424, 36, 6,
        displayColor(pose.accuracy > 0.65F ? TFT_GREEN
                     : pose.accuracy > 0.25F ? TFT_ORANGE
                                            : TFT_RED));
    scopeCanvas.setTextDatum(middle_left);
    scopeCanvas.setTextColor(displayColor(TFT_DARKGREY), TFT_BLACK);
    scopeCanvas.drawString(orientation.name(), 16, 36, 1);
    const std::uint16_t voiceColor =
        voiceAssistant.isRecording() ? TFT_RED
        : voiceAssistant.isBusy()    ? TFT_ORANGE
        : voiceAssistant.isOnline()  ? TFT_GREEN
                                     : TFT_DARKGREY;
    scopeCanvas.setTextColor(displayColor(voiceColor), TFT_BLACK);
    scopeCanvas.drawString(voiceAssistant.statusLabel(), 16, 54, 1);
  }
  scopeCanvas.pushSprite(0, 0);
}

void handleGesture(const Gesture& gesture) {
  if (!gesture.released) return;
  if (orientation.emulating() && !gesture.tap) {
    orientation.adjustEmulator(-gesture.deltaX * 0.20F,
                               -gesture.deltaY * 0.18F);
  }
}

void handleButtons(std::uint32_t nowMs) {
  if (M5.BtnB.wasHold()) setRedNightMode(!redNightMode);
  if (M5.BtnB.wasClicked()) showHud = !showHud;

  // Capture immediately, then decide at the one-second boundary whether the
  // yellow-button gesture was a magnitude click or push-to-talk. Recording
  // before the boundary prevents the first spoken word from being clipped.
  if (M5.BtnA.wasPressed()) voiceAssistant.pressPushToTalk(nowMs);
  if (M5.BtnA.pressedFor(kButtonHoldMs)) {
    voiceAssistant.commitPushToTalk();
  }
  if (M5.BtnA.wasReleased() && !voiceAssistant.releasePushToTalk(nowMs)) {
    settings.cycleMagnitude();
  }
}

}  // namespace

void setup() {
  // Arduino ESP32 3.x no longer makes USB CDC output usable here until the
  // application explicitly opens Serial. Do this before M5 initialization so
  // all voice/network diagnostics remain visible in `pio device monitor`.
  Serial.begin(115200);
  delay(50);
  Serial.println("[BOOT] M5StarScope starting");

  auto config = M5.config();
  config.clear_display = true;
  config.output_power = true;
  M5.begin(config);
  M5.Display.setRotation(0);
  M5.Display.setBrightness(kNormalDisplayBrightness);
  M5.Display.setTextWrap(false);
  M5.BtnA.setHoldThresh(kButtonHoldMs);
  M5.BtnB.setHoldThresh(kButtonHoldMs);
  M5.Power.setExtOutput(true);

  settings.begin();
  orientation.begin();
  configurationMode = settings.configurationMode();
  if (configurationMode) {
    Serial.println("[SETUP] starting smartphone configuration portal");
    if (!beginConfigurationPortal()) {
      drawConfigurationMessage("SETUP ERROR", "Could not start Wi-Fi AP");
    }
    return;
  }

  // Rendering and pose ingestion are the real-time path. The voice worker
  // shares core 1 at priority 1, so this loop always wins scheduling without
  // putting application-side TLS on Wi-Fi/lwIP's core 0.
  vTaskPrioritySet(nullptr, kScopeTaskPriority);
  Serial.printf("[BOOT] scope loop core=%d, priority=%u\n", xPortGetCoreID(),
                static_cast<unsigned>(uxTaskPriorityGet(nullptr)));

  scopeCanvas.setColorDepth(16);
  scopeCanvas.setPsram(true);
  scopeCanvas.createSprite(kScreenWidth, kScreenHeight);
  skyVectors = static_cast<PackedSkyVector*>(
      ps_malloc(sizeof(PackedSkyVector) * starCatalog.size()));
  if (!skyVectors) {
    skyVectors = static_cast<PackedSkyVector*>(
        std::malloc(sizeof(PackedSkyVector) * starCatalog.size()));
  }

  const std::uint32_t nowMs = millis();
  VoiceAssistantConfig voiceConfig;
  voiceConfig.wifiSsid = settings.wifiSsid();
  voiceConfig.wifiPassword = settings.wifiPassword();
  voiceConfig.webSocketHost = settings.voiceWebSocketHost();
  voiceConfig.webSocketPort = settings.voiceWebSocketPort();
  voiceConfig.webSocketPath = settings.voiceWebSocketPath();
  voiceConfig.playbackVolume = static_cast<std::uint8_t>(
      (static_cast<std::uint16_t>(settings.voiceVolumePercent()) * 255U + 50U) /
      100U);
  voiceConfig.skyContextWriter = writeVoiceSkyContext;
  voiceConfig.skyContextIntervalMs = kVoiceSkyContextIntervalMs;
  synchronizeJulianClock(nowMs);
  refreshVoiceSkySnapshot(nowMs);
  hudUtc = readRtcUtc();
  lastHudRtcReadMs = nowMs;
  voiceAssistant.begin(voiceConfig, nowMs);
  rebuildSkyCache(nowMs);
  drawScope();
}

void loop() {
  M5.update();
  const std::uint32_t nowMs = millis();
  orientation.update(nowMs);

  if (configurationTransitionPending) {
    if (static_cast<std::int32_t>(nowMs - configurationRestartAtMs) >= 0) {
      ESP.restart();
    }
    delay(2);
    return;
  }

  if (orientation.consumeSetupToggle()) {
    scheduleConfigurationRestart(!configurationMode, nowMs);
    delay(2);
    return;
  }

  if (configurationMode) {
    configurationWebServer.handleClient();
    delay(2);
    return;
  }

  handleButtons(nowMs);
  voiceAssistant.update(nowMs);
  handleGesture(gestures.update());

  if (showHud && nowMs - lastHudRtcReadMs >= 1000) {
    hudUtc = readRtcUtc();
    lastHudRtcReadMs = nowMs;
  }
  // A full rebuild remains only for explicit location/time changes. Normal
  // sidereal motion is maintained cooperatively below, six stars per frame.
  if (!skyCacheValid) rebuildSkyCache(nowMs);
  if (nowMs - lastFrameMs >= kFrameIntervalMs) {
    lastFrameMs = nowMs;
    updateSkyCacheIncremental(nowMs);
    updateMovingBodyCache(nowMs);
    drawScope();
  }
  delay(2);
}
