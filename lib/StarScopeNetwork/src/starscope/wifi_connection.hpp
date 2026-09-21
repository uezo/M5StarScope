#pragma once

namespace starscope {

struct WifiConnectionConfig {
  const char* ssid = nullptr;
  const char* password = nullptr;
};

// Share one connection manager between all network services. Its worker owns
// station setup/retries; clients only inspect WiFi.status() and use sockets.
class WifiConnection {
 public:
  WifiConnection();
  ~WifiConnection();

  WifiConnection(const WifiConnection&) = delete;
  WifiConnection& operator=(const WifiConnection&) = delete;

  // Copies both strings before starting a background task. An empty SSID is a
  // successful no-op; invalid lengths or allocation/task failures return false.
  // Once running, additional calls are no-ops; configuration changes require
  // a new instance. Call begin/destruction from one owner task.
  bool begin(const WifiConnectionConfig& config);

 private:
  class Impl;
  Impl* impl_ = nullptr;
};

}  // namespace starscope
