/**
 * @file main.c
 * @author risinek (risinek@gmail.com)
 * @date 2021-04-03
 * @copyright Copyright (c) 2021
 *
 * @brief Main file used to setup ESP32 into initial state
 *
 * Starts management AP and webserver
 */

#include <stdio.h>

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "attack.h"
#include "bluetooth_serial.h"
#include "bt_logs_forwarder.h"
#include "device_id.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "serial_command_dispatcher.h"
#include "webserver.h"
#include "wifi_controller.h"

#ifdef CONFIG_ENABLE_UNIT_TESTS
#include "ring_buffer_test.h"
#endif

namespace {
const char* LOG_TAG = "main";
SerialCommandDispatcher gSerialCommandDispatcher;
BtLogsForwarder gBtLogsForwarder;
}  // namespace

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_ENABLE_UNIT_TESTS
void testBT() {
  ESP_LOGD(LOG_TAG, "Testing BT transmition");

  for (int i = 0; i < 10; ++i) {
    std::vector<char> line(10, '0' + i);
    BluetoothSerial::instance().send(std::move(line));
  }

  for (int i = 0; i < 50; ++i) {
    std::vector<char> line(i + 1, '0' + i);
    BluetoothSerial::instance().send(std::move(line));
  }
}
#endif

void setLogLevel(const std::string& levelStr) {
  if (levelStr == "n")
    esp_log_level_set("*", ESP_LOG_NONE);
  else if (levelStr == "e")
    esp_log_level_set("*", ESP_LOG_ERROR);
  else if (levelStr == "w")
    esp_log_level_set("*", ESP_LOG_WARN);
  else if (levelStr == "i")
    esp_log_level_set("*", ESP_LOG_INFO);
  else if (levelStr == "d")
    esp_log_level_set("*", ESP_LOG_DEBUG);
  else if (levelStr == "v")
    esp_log_level_set("*", ESP_LOG_VERBOSE);
  else
    ESP_LOGE(LOG_TAG, "Unsupported log level '%s'", levelStr.c_str());
}

void setSerialCommandsHandlers() {
  gSerialCommandDispatcher.setCommandHandler(SerialCommandDispatcher::CommandType::kReset,
                                             [](const std::string& param) {
                                               ESP_LOGI(LOG_TAG, "RESETTING ESP32");
                                               esp_restart();
                                             });
  gSerialCommandDispatcher.setCommandHandler(SerialCommandDispatcher::CommandType::kStartLogs,
                                             [](const std::string& param) {
                                               ESP_LOGI(LOG_TAG, "Start redirecting logs to BT");
                                               gBtLogsForwarder.startForwarding();
                                             });
  gSerialCommandDispatcher.setCommandHandler(SerialCommandDispatcher::CommandType::kStopLogs,
                                             [](const std::string& param) {
                                               ESP_LOGI(LOG_TAG, "Stop redirecting logs to BT");
                                               gBtLogsForwarder.stopForwarding();
                                             });
  gSerialCommandDispatcher.setCommandHandler(SerialCommandDispatcher::CommandType::kLimitLogs,
                                             [](const std::string& param) {
                                               bool shouldLimit{false};
                                               if (param == "1") {
                                                 ESP_LOGI(LOG_TAG, "Limitting logs ");
                                                 shouldLimit = true;
                                               } else {
                                                 ESP_LOGI(LOG_TAG, "Unlimitting logs ");
                                                 shouldLimit = false;
                                               }
                                               attack_limit_logs(shouldLimit);
                                             });
  gSerialCommandDispatcher.setCommandHandler(SerialCommandDispatcher::CommandType::kSetLogLevel,
                                             [](const std::string& param) { setLogLevel(param); });
  gSerialCommandDispatcher.setCommandHandler(SerialCommandDispatcher::CommandType::kHelp, [](const std::string& param) {
    BluetoothSerial::instance().send(gSerialCommandDispatcher.getSupportedCommands());
  });
  gSerialCommandDispatcher.setCommandHandler(
      SerialCommandDispatcher::CommandType::kBtTerminalConnected, [](const std::string& param) {
        if (param == "1") {
          BluetoothSerial::instance().limitBTLogs(true);
          // Hide it if you don't wan't people to know what is this Bluetooth device about
          std::string greeting{
              "\n\r\n\r\n\r\n\r\n\rWelcome to ESP32 WiFi penetration tool\n\r"
              "Supported commands: "};
          BluetoothSerial::instance().send(greeting + gSerialCommandDispatcher.getSupportedCommands());
        } else {
          // Should limit logs from BT, otherwise there will be infinite stream of logs from BT about sent messages,
          // which will cause more messages to send, etc.
          BluetoothSerial::instance().limitBTLogs(false);
        }
      });
}

void app_main(void) {
  BluetoothSerial::instance().init(
      [](std::string receivedData) { gSerialCommandDispatcher.onNewSymbols(std::move(receivedData)); });
  gBtLogsForwarder.startForwarding();

  ESP_LOGD(LOG_TAG, "app_main() started. Device ID='%d'", CONFIG_DEVICE_ID);
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifictl_mgmt_ap_start();
  attack_init();
  webserver_run();
  setSerialCommandsHandlers();

  attack_limit_logs(true);

#ifdef CONFIG_ENABLE_UNIT_TESTS
  // testBT();
  test_ring_buffer();
#endif
}

#ifdef __cplusplus
}
#endif
