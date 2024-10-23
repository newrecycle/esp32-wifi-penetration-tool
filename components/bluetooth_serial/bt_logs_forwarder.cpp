#include "bt_logs_forwarder.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <vector>

#include "bluetooth_serial.h"
#include "esp_log.h"

namespace {
const char* LOG_TAG{"BT_FWD"};

// This helper formats the log messages with the variable argument list
std::vector<char> vformat(const char* format, va_list args) {
  va_list copy;
  va_copy(copy, args);
  int len = std::vsnprintf(nullptr, 0, format, copy);  // Get the length of the formatted string
  va_end(copy);

  if (len >= 0) {
    std::vector<char> data(std::size_t(len) + 1);  // Allocate buffer for the string
    std::vsnprintf(&data[0], data.size(), format, args);  // Format the string
    data[len] = '\0';  // Null-terminate the string
    return data;
  }

  return {};
}
}  // namespace

BtLogsForwarder::ESPLogFn BtLogsForwarder::mSerialVprintf{nullptr};

BtLogsForwarder::BtLogsForwarder() {
  // Save the original log function and set it back to the original
  mSerialVprintf = esp_log_set_vprintf(nullptr);
  esp_log_set_vprintf(mSerialVprintf);
}

void BtLogsForwarder::startForwarding() {
  if (mIsForwarding) {
    ESP_LOGE(LOG_TAG, "BLE logs forwarding is already enabled");
    return;
  }
  mIsForwarding = true;
  // Set our custom logging function to handle forwarding logs over BLE
  esp_log_set_vprintf(btVprintf);
}

void BtLogsForwarder::stopForwarding() {
  if (!mIsForwarding) {
    ESP_LOGE(LOG_TAG, "BLE logs forwarding is already disabled");
    return;
  }
  mIsForwarding = false;
  // Restore the original log function
  esp_log_set_vprintf(mSerialVprintf);
}

void BtLogsForwarder::printToSerial(const char* format, va_list& args) {
  // Call the original log function to print to the serial console
  if (mSerialVprintf != nullptr) {
    mSerialVprintf(format, args);
  }
}

// This is the custom log function that forwards logs to BLE
int BtLogsForwarder::btVprintf(const char* format, va_list args) {
  // Format the log message
  auto message = vformat(format, args);
  if (!message.empty()) {
    // Send the formatted message via BLE using BluetoothSerial
    BluetoothSerial::instance().send(std::move(message));
  }
  // Also print the log message to the original serial console
  return mSerialVprintf(format, args);
}
