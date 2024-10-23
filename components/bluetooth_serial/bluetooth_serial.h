#ifndef BT_SERIAL_H_
#define BT_SERIAL_H_

#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "bt_logs_forwarder.h"
#include "esp_gatts_api.h"  // BLE-specific API
#include "ring_buffer.h"

// BluetoothSerial class for BLE GATT-based communication.
class BluetoothSerial {
 public:
  // Public methods for managing BLE connection and data transmission
  void setConnectionHandle(uint16_t conn_id) { mConnectionHandle = conn_id; }
  void resetConnectionHandle() { mConnectionHandle = 0; }
  void triggerLimitBTLogs(bool limit) { limitBTLogs(limit); }
  void triggerOnBtDataReceived(std::string data) { onBtDataRecevied(std::move(data)); }

  using OnBtDataReceviedCallbackType = std::function<void(std::string receivedData)>;
  
  // Singleton instance of BluetoothSerial
  static BluetoothSerial& instance();

  // Initializes the BLE system and sets up the GATT server
  bool init(BtLogsForwarder* btLogsForwarder, OnBtDataReceviedCallbackType dataReceviedCallback,
            uint32_t maxTxBufSize = 16 * 1024);

  // Send data through BLE characteristic
  bool send(std::string message);
  bool send(std::vector<char> message);

 private:
  BluetoothSerial() = default;

  // Callback for receiving data over BLE
  void onBtDataRecevied(std::string receivedData);

  // GATT event handler (handles connect, disconnect, read, write events)
  static void esp_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);

  // Handles transmitting chunks of data
  void transmitChunkOfData();

  // Extracts a data chunk from the ring buffer for transmission
  void extractDataChunkToTransmit();

  // Limit Bluetooth logs when the connection is established
  void limitBTLogs(bool isLimited);

  // Print limited logs
  void limitedPrint(uint8_t logLevel, const char* tag, const char* format, ...);

  // Forwarder for Bluetooth logs
  BtLogsForwarder* mBtLogsForwarder{nullptr};

  // Callback when data is received over BLE
  OnBtDataReceviedCallbackType mDataReceviedCallback{nullptr};

  // Mutex to protect shared resources
  std::mutex mMutex;

  // BLE connection and GATT-related variables
  uint16_t mConnectionHandle{0};
  esp_gatt_if_t mGattIf = ESP_GATT_IF_NONE;  // GATT interface
  uint16_t mServiceHandle = 0;               // GATT service handle
  uint16_t mCharHandle = 0;                  // GATT characteristic handle

  // Transmission data buffer
  RingBuffer mTxData;
  uint32_t mMaxTxBufSize{16 * 1024};
  std::vector<char> mCurrentTransmittedChunk;
  bool mIsTransmissionRequestInProgress{false};
  bool mIsLimitedLogs{false};
};

#endif  // BT_SERIAL_H_
