#include "starscope/wifi_connection.hpp"

#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <new>

namespace starscope {
namespace {

constexpr std::uint32_t kWifiRetryMs = 15000;
constexpr std::uint32_t kStatusIntervalMs = 250;
constexpr std::uint32_t kWorkerStackBytes = 4096;
constexpr UBaseType_t kWorkerPriority = 1;
constexpr BaseType_t kWorkerCore = 0;
// WiFi is global on the ESP32. Reject accidental duplicate managers instead of
// letting two independently configured tasks change the same station state.
std::atomic<bool> workerOwned{false};

}  // namespace

class WifiConnection::Impl {
 public:
  ~Impl() {
    if (taskHandle_) {
      shutdownRequested_.store(true, std::memory_order_release);
      xTaskNotifyGive(taskHandle_);
      // Let an in-progress Wi-Fi API call finish. Deleting the task underneath
      // the driver could abandon one of its locks. The worker acknowledges only
      // after it has stopped using this object and its copied credentials.
      xSemaphoreTake(stopped_, portMAX_DELAY);
      taskHandle_ = nullptr;
      workerOwned.store(false, std::memory_order_release);
    }
    if (stopped_) vSemaphoreDelete(stopped_);
    // Other services may still be using the station. Destroying this manager
    // stops its maintenance task without disconnecting or changing Wi-Fi mode.
  }

  bool begin(const WifiConnectionConfig& config) {
    if (taskHandle_) return true;
    if (!config.ssid || !config.ssid[0]) return true;

    const std::size_t ssidLength = strnlen(config.ssid, sizeof(ssid_));
    const std::size_t passwordLength = config.password
        ? strnlen(config.password, sizeof(password_)) : 0;
    if (ssidLength >= sizeof(ssid_) || passwordLength >= sizeof(password_)) {
      Serial.println("[WIFI] invalid credential length");
      return false;
    }
    std::memcpy(ssid_, config.ssid, ssidLength + 1);
    if (passwordLength) std::memcpy(password_, config.password, passwordLength);
    password_[passwordLength] = '\0';

    stopped_ = xSemaphoreCreateBinary();
    if (!stopped_) {
      Serial.println("[WIFI] worker completion allocation failed");
      return false;
    }
    bool expected = false;
    if (!workerOwned.compare_exchange_strong(expected, true,
                                             std::memory_order_acq_rel)) {
      vSemaphoreDelete(stopped_);
      stopped_ = nullptr;
      Serial.println("[WIFI] connection manager already running");
      return false;
    }

    const BaseType_t created = xTaskCreatePinnedToCore(
        taskEntry, "starscope_wifi", kWorkerStackBytes, this,
        kWorkerPriority, &taskHandle_, kWorkerCore);
    if (created != pdPASS) {
      taskHandle_ = nullptr;
      workerOwned.store(false, std::memory_order_release);
      vSemaphoreDelete(stopped_);
      stopped_ = nullptr;
      Serial.println("[WIFI] worker creation failed");
      return false;
    }
    return true;
  }

 private:
  static void taskEntry(void* argument) {
    auto* self = static_cast<Impl*>(argument);
    self->runWorker();
    // Do not access self after giving the semaphore: its owner may immediately
    // destroy it. The task can then retire using only its own stack.
    const SemaphoreHandle_t stopped = self->stopped_;
    xSemaphoreGive(stopped);
    vTaskDelete(nullptr);
  }

  void runWorker() {
    Serial.printf("[WIFI] worker started on core %d, priority=%u\n",
                  xPortGetCoreID(),
                  static_cast<unsigned>(uxTaskPriorityGet(nullptr)));
    if (shutdownRequested_.load(std::memory_order_acquire)) return;
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);
    WiFi.begin(ssid_, password_);
    std::uint32_t lastAttemptMs = millis();
    bool wasConnected = false;

    while (!shutdownRequested_.load(std::memory_order_acquire)) {
      const bool connected = WiFi.status() == WL_CONNECTED;
      if (connected != wasConnected) {
        Serial.println(connected ? "[WIFI] connected" : "[WIFI] disconnected");
        wasConnected = connected;
      }
      const std::uint32_t nowMs = millis();
      if (!connected && nowMs - lastAttemptMs >= kWifiRetryMs) {
        lastAttemptMs = nowMs;
        // Automatic reconnect normally handles a lost AP. This also retries an
        // initial failed association, including when the driver has no config.
        if (!WiFi.reconnect()) WiFi.begin(ssid_, password_);
      }
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kStatusIntervalMs));
    }
  }

  char ssid_[33]{};
  char password_[65]{};
  std::atomic<bool> shutdownRequested_{false};
  TaskHandle_t taskHandle_ = nullptr;
  SemaphoreHandle_t stopped_ = nullptr;
};

WifiConnection::WifiConnection() : impl_(new (std::nothrow) Impl()) {}
WifiConnection::~WifiConnection() { delete impl_; }

bool WifiConnection::begin(const WifiConnectionConfig& config) {
  if (!impl_) {
    Serial.println("[WIFI] connection manager allocation failed");
    return false;
  }
  return impl_->begin(config);
}

}  // namespace starscope
