#include "bluetooth_serial.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <map>

#include "device_id.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gatts_api.h"  // BLE GATT API
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

namespace {
const char* LOG_TAG{"BLE"};
const char* kServiceUUID = "00001800-0000-1000-8000-00805f9b34fb";  // Example BLE service UUID
const char* kCharacteristicUUID = "00002a00-0000-1000-8000-00805f9b34fb";  // Example BLE characteristic UUID

std::string bleDataToString(const uint8_t* data, uint16_t len) {
    return std::string(reinterpret_cast<const char*>(data), len);
}

void esp_ble_gap_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    switch (event) {
        case ESP_GAP_BLE_AUTH_CMPL_EVT: {
            if (param->ble_security.auth_cmpl.success) {
                ESP_LOGI(LOG_TAG, "BLE Authentication Success");
            } else {
                ESP_LOGE(LOG_TAG, "BLE Authentication Failed");
            }
            break;
        }
        // Other gap events can be handled here as needed
        default:
            break;
    }
}

void esp_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
    auto& bleSerial = BluetoothSerial::instance();
    
    switch (event) {
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(LOG_TAG, "BLE device connected");
            bleSerial.mConnectionHandle = param->connect.conn_id;
            bleSerial.limitBTLogs(true);
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(LOG_TAG, "BLE device disconnected");
            bleSerial.mConnectionHandle = 0;
            bleSerial.limitBTLogs(false);
            // Simulate disconnection event to other parts of the system
            bleSerial.onBtDataRecevied("bleterminalconnected 0");
            break;

        case ESP_GATTS_WRITE_EVT:
            if (param->write.is_prep) {
                // Handle prepared write
            } else {
                std::string receivedData = bleDataToString(param->write.value, param->write.len);
                bleSerial.onBtDataRecevied(receivedData);
            }
            break;

        case ESP_GATTS_READ_EVT:
            ESP_LOGI(LOG_TAG, "BLE Read event");
            // Handle reading from characteristic
            break;

        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(LOG_TAG, "MTU size updated to %d", param->mtu.mtu);
            break;

        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(LOG_TAG, "Service created");
            break;

        case ESP_GATTS_START_EVT:
            ESP_LOGI(LOG_TAG, "Service started");
            break;

        default:
            break;
    }
}

}  // namespace

BluetoothSerial& BluetoothSerial::instance() {
    static BluetoothSerial instance;
    return instance;
}

bool BluetoothSerial::init(BtLogsForwarder* btLogsForwarder, OnBtDataReceviedCallbackType dataReceviedCallback, uint32_t maxTxBufSize) {
    mBtLogsForwarder = btLogsForwarder;
    mDataReceviedCallback = std::move(dataReceviedCallback);
    mMaxTxBufSize = maxTxBufSize;
    mTxData.resize(mMaxTxBufSize);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(LOG_TAG, "Bluetooth controller init failed: %s", esp_err_to_name(ret));
        return false;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
        ESP_LOGE(LOG_TAG, "Enable Bluetooth controller failed: %s", esp_err_to_name(ret));
        return false;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(LOG_TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return false;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(LOG_TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return false;
    }

    if ((ret = esp_ble_gatts_register_callback(esp_gatts_event_handler)) != ESP_OK) {
        ESP_LOGE(LOG_TAG, "GATT register callback failed: %s", esp_err_to_name(ret));
        return false;
    }

    if ((ret = esp_ble_gap_register_callback(esp_ble_gap_callback)) != ESP_OK) {
        ESP_LOGE(LOG_TAG, "GAP register callback failed: %s", esp_err_to_name(ret));
        return false;
    }

    // Set up BLE service and characteristics
    // You'll need to define the GATT profile with characteristics that support data transmission

    return true;
}

bool BluetoothSerial::send(std::string message) {
    std::vector<char> data{message.begin(), message.end()};
    message.clear();
    return send(std::move(data));
}

bool BluetoothSerial::send(std::vector<char> message) {
    std::unique_lock<std::mutex> lock(mMutex);
    auto doesMessageFitBuffer = mTxData.insert(message.begin(), message.end());

    if (mConnectionHandle == 0) {
        // No terminal connected
        return doesMessageFitBuffer;
    }

    if (mIsTransmissionRequestInProgress == true) {
        return doesMessageFitBuffer;
    }

    extractDataChunkToTransmit();
    mIsTransmissionRequestInProgress = true;
    lock.unlock();

    // Write data to BLE characteristic (replace with proper BLE GATT write API)
    esp_ble_gatts_send_indicate(/*gatts_if*/, mConnectionHandle, /*attr_handle*/, 
        mCurrentTransmittedChunk.size(), reinterpret_cast<uint8_t*>(mCurrentTransmittedChunk.data()), false);

    return doesMessageFitBuffer;
}

void BluetoothSerial::onBtDataRecevied(std::string receivedData) {
    if (nullptr != mDataReceviedCallback) {
        mDataReceviedCallback(std::move(receivedData));
    }
}

void BluetoothSerial::transmitChunkOfData() {
    std::unique_lock<std::mutex> lock(mMutex);
    if (mConnectionHandle == 0 || mIsTransmissionRequestInProgress) {
        return;
    }

    extractDataChunkToTransmit();
    if (mCurrentTransmittedChunk.empty()) {
        return;
    }

    mIsTransmissionRequestInProgress = true;
    lock.unlock();

    esp_ble_gatts_send_indicate(/*gatts_if*/, mConnectionHandle, /*attr_handle*/,
        mCurrentTransmittedChunk.size(), reinterpret_cast<uint8_t*>(mCurrentTransmittedChunk.data()), false);
}

void BluetoothSerial::extractDataChunkToTransmit() {
    if (!mCurrentTransmittedChunk.empty()) {
        return;
    }
    mCurrentTransmittedChunk = mTxData.extract(/*BLE_MAX_CHUNK_SIZE*/);
}

void BluetoothSerial::limitBTLogs(bool isLimited) {
    mIsLimitedLogs = isLimited;
    // Same logic as in the original version
}

void BluetoothSerial::limitedPrint(uint8_t logLevel, const char* tag, const char* format, ...) {
    va_list args;
    va_start(args, format);

    if (!mIsLimitedLogs) {
        esp_log_writev((esp_log_level_t)logLevel, tag, format, args);
    } else if (mBtLogsForwarder) {
        mBtLogsForwarder->printToSerial(format, args);
    }

    va_end(args);
}
