#include "bluetooth_serial.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <map>
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_gap_ble_api.h"

#define GATTS_SERVICE_UUID   0x00FF  // Custom service UUID
#define GATTS_CHAR_UUID      0xFF01  // Custom characteristic UUID
#define GATTS_NUM_HANDLE     4       // Number of handles for your GATT service

namespace {
const char* LOG_TAG{"BLE"};

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
        default:
            break;
    }
}

}  // namespace

BluetoothSerial& BluetoothSerial::instance() {
    static BluetoothSerial instance;
    return instance;
}

void BluetoothSerial::esp_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
    auto& bleSerial = BluetoothSerial::instance();
    switch (event) {
        case ESP_GATTS_REG_EVT: {
            ESP_LOGI(LOG_TAG, "GATT server registered");
            bleSerial.mGattIf = gatts_if;

            // Break down the service ID initialization into multiple steps
            esp_gatt_srvc_id_t service_id;
            service_id.is_primary = true;  // Initialize 'is_primary'
            service_id.id.inst_id = 0x00;  // Initialize 'inst_id'
            service_id.id.uuid.len = ESP_UUID_LEN_16;  // Set UUID length
            service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID;  // Set 16-bit UUID

            // Create GATT service
            esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
            break;
        }

        case ESP_GATTS_CREATE_EVT: {
            ESP_LOGI(LOG_TAG, "GATT service created");
            bleSerial.mServiceHandle = param->create.service_handle;
            esp_ble_gatts_start_service(bleSerial.mServiceHandle);

            // Add GATT characteristic
            esp_bt_uuid_t char_uuid;
            char_uuid.len = ESP_UUID_LEN_16;
            char_uuid.uuid.uuid16 = GATTS_CHAR_UUID;

            esp_ble_gatts_add_char(bleSerial.mServiceHandle, &char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY, NULL, NULL);
            break;
        }

        case ESP_GATTS_ADD_CHAR_EVT:
            ESP_LOGI(LOG_TAG, "GATT characteristic added");
            bleSerial.mCharHandle = param->add_char.attr_handle;
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(LOG_TAG, "Client connected");
            bleSerial.setConnectionHandle(param->connect.conn_id);
            bleSerial.triggerLimitBTLogs(true);
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(LOG_TAG, "Client disconnected");
            bleSerial.resetConnectionHandle();
            bleSerial.triggerLimitBTLogs(false);
            bleSerial.triggerOnBtDataReceived("bleterminalconnected 0");
            break;

        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep) {
                std::string receivedData = bleDataToString(param->write.value, param->write.len);
                bleSerial.onBtDataRecevied(receivedData);
            }
            break;

        default:
            ESP_LOGI(LOG_TAG, "Unhandled GATT event: %d", event);
            break;
    }
}
void BluetoothSerial::onBtDataRecevied(std::string receivedData) {
    if (nullptr != mDataReceviedCallback) {
        mDataReceviedCallback(std::move(receivedData));
    }
}

bool BluetoothSerial::init(BtLogsForwarder* btLogsForwarder, OnBtDataReceviedCallbackType dataReceviedCallback, uint32_t maxTxBufSize) {
    mBtLogsForwarder = btLogsForwarder;
    mDataReceviedCallback = std::move(dataReceviedCallback);
    mMaxTxBufSize = maxTxBufSize;
    mTxData.resize(mMaxTxBufSize);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
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

    // Register GATT and GAP callbacks
    if ((ret = esp_ble_gatts_register_callback(esp_gatts_event_handler)) != ESP_OK) {
        ESP_LOGE(LOG_TAG, "GATT callback registration failed: %s", esp_err_to_name(ret));
        return false;
    }

    esp_ble_gatts_app_register(0);  // Register app profile

    if ((ret = esp_ble_gap_register_callback(esp_ble_gap_callback)) != ESP_OK) {
        ESP_LOGE(LOG_TAG, "GAP register callback failed: %s", esp_err_to_name(ret));
        return false;
    }

    return true;
}

bool BluetoothSerial::send(std::string message) {
    std::vector<char> data{message.begin(), message.end()};
    return send(std::move(data));
}

bool BluetoothSerial::send(std::vector<char> message) {
    std::unique_lock<std::mutex> lock(mMutex);
    auto doesMessageFitBuffer = mTxData.insert(message.begin(), message.end());

    if (mConnectionHandle == 0) {
        return doesMessageFitBuffer;
    }

    if (mIsTransmissionRequestInProgress == true) {
        return doesMessageFitBuffer;
    }

    extractDataChunkToTransmit();
    mIsTransmissionRequestInProgress = true;
    lock.unlock();

    esp_ble_gatts_send_indicate(mGattIf, mConnectionHandle, mCharHandle,
        mCurrentTransmittedChunk.size(), (uint8_t*)mCurrentTransmittedChunk.data(), false);

    return doesMessageFitBuffer;
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

    esp_ble_gatts_send_indicate(mGattIf, mConnectionHandle, mCharHandle,
        mCurrentTransmittedChunk.size(), reinterpret_cast<uint8_t*>(mCurrentTransmittedChunk.data()), false);
}

void BluetoothSerial::extractDataChunkToTransmit() {
    if (!mCurrentTransmittedChunk.empty()) {
        return;
    }
    mCurrentTransmittedChunk = mTxData.extract(512);  // Adjust size as needed
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
