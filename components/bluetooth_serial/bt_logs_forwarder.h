#ifndef BT_LOGS_FORWARDER_H_
#define BT_LOGS_FORWARDER_H_

#include <stdarg.h>

// This class enables all logs, printed by ESP32, to be copied to Bluetooth (now BLE)
class BtLogsForwarder {
 public:
  BtLogsForwarder();
  void startForwarding();
  void stopForwarding();

  // Forwards log data to the BLE characteristic instead of serial (when BLE is active)
  void printToSerial(const char* format, va_list& args);

 private:
  using ESPLogFn = int (*)(const char*, va_list);

  // This function must be static so that we can use it in the ESP Log API
  static int btVprintf(const char* format, va_list args);

  static ESPLogFn mSerialVprintf;
  bool mIsForwarding{false};
};

#endif  // BT_LOGS_FORWARDER_H_
