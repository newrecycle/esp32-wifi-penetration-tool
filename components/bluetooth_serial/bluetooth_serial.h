#ifndef BT_SERIAL_H_
#define BT_SERIAL_H_

#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "bt_logs_forwarder.h"
#include "esp_gatts_api.h"  // BLE-specific API
#include "ring_buffer.h"

// Currently no PIN is required to pair ESP32. BLE pairing process will rely on passkey if required.
// You can set a passkey or use just-confirm-on-device pairing if desired.
class BluetoothSerial {
 public:
  using OnBtDataReceviedCallbackType = std::function<void(std::string receivedData)>;
  static BluetoothSerial& instance();
  bool init(BtLogsForwarder* btLogsForwarder, OnBtDataReceviedCallbackType dataReceviedCallback,
            uint32_t maxTxBufSize = 16 * 1024);

  bool send(std::string message);
  bool send(std::vector<char> message);

 private:
  BluetoothSerial() = default;
  void onBtDataRecevied(std::string receivedData);
  
  // Replacing SPP callback with GATT event handling
  friend void esp_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);

  void transmitChunkOfData();
  // NOTE!
  // 1. Requires external lock
  // 2. Moves extracted data to mCurrentTransmittedChunk
  void extractDataChunkToTransmit();
  
  void limitBTLogs(bool isLimited);
  
  void limitedPrint(uint8_t logLevel, const char* tag, const char* format, ...);

  BtLogsForwarder* mBtLogsForwarder{nullptr};
  OnBtDataReceviedCallbackType mDataReceviedCallback{nullptr};

  // This data can be accessed from multiple threads, so it is protected by mutex
  std::mutex mMutex;
  
  // Connection handle for BLE
  uint16_t mConnectionHandle{0};
  
  RingBuffer mTxData;
  uint32_t mMaxTxBufSize{16 * 1024};
  std::vector<char> mCurrentTransmittedChunk;
  bool mIsTransmissionRequestInProgress{false};
  bool mIsLimitedLogs{false};
};

#endif  // BT_SERIAL_H_
