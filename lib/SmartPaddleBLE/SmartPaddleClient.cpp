#include <SmartPaddleClient.h>
#include <SmartPaddle.h>
#include <Preferences.h>
#include <SP_BLESerialClient.h>
#include <SP_BLESerial.h>
#include <SP_CommandParams.h>
//#include <SP_CoordinateTransform.h>
#include "BLEHealthMonitor.h"

#define BLE_STACK_SIZE 4096
#define BLE_RECEIVE_STACK_SIZE 8192
#define EVENT_STACK_SIZE 4096

void SetPaddleCalibration(SmartPaddleBLEClient* paddle){
    float offset[3];
    float softIron[6];
    offset[0] = -2367.511161;
    offset[1] = 16424.030349;
    offset[2] = 6467.202519;

    softIron[0] = 0.0163386;
    softIron[1] = 0.0161951;
    softIron[2] = 0.0175254;

    softIron[3] = -0.0000684;
    softIron[4] = -0.0004882;
    softIron[5] = -0.0004394;

    paddle->getSerial()->sendString(SP_MessageProcessor::createSetMagnetometerCalibrationCommand(offset, softIron, &softIron[3]));

}

void SetPaddleSpecs(SmartPaddleBLEClient* paddle){

    PaddleSpecs specs;
    specs=paddle->getSpecs();

    specs.paddleType = PaddleType::TWO_BLADES;
    specs.length = 2.2f;
    specs.imuDistance = 0.00f;
    specs.bladeWeight = 0.3f;
    specs.bladeCenter = 0.20f;
    specs.bladeMomentInertia = 0.01;

    specs.firmwareVersion = 1.1;
    specs.paddleModel = "RST-220-KM";
    specs.hasLeftBlade = true;
    specs.hasRightBlade = true;
    specs.imuFrequency = 100;

    specs.axisDirection = Y_AXIS_LEFT;

//------------------------
    specs.paddleType = PaddleType::TWO_BLADES;
    specs.length = 2.07f;
    specs.imuDistance = -0.20f;
    specs.bladeWeight = 0.25f;
    specs.bladeCenter = 0.18f;
    specs.bladeMomentInertia = 0.01;

    specs.firmwareVersion = 1.1;
    specs.paddleModel = "AKSU-200-AB";
    specs.hasLeftBlade = true;
    specs.hasRightBlade = true;
    specs.imuFrequency = 100;
    specs.axisDirection = Y_AXIS_LEFT;
    specs.axisDirectionSign = -1;

    paddle->getSerial()->sendString(SP_MessageProcessor::createSpecsMessage(specs));
}

void SmartPaddleBLEClient::setSpecs(const PaddleSpecs& sp, bool save) {
    specs=sp;
    if (save) {
        getSerial()->sendString(SP_MessageProcessor::createSpecsMessage(specs));
    }
}


/**
 * @brief Обработчик сообщений для SmartPaddleClient (мигрирован на callback подход)
 */
class SPClient_MessageHandler: public SP_MessageHandler{
private:
    SmartPaddleBLEClient* paddle;

    void setupHandlers() {
        // ============================================
        // ДАННЫЕ
        // ============================================
        
        // Обработка спецификаций весла
        registerData(SP_Protocol::DataTypes::SPECS,
            [this](SP_Data* data) {
                SP_DataValue value(data);
                
                paddle->specs.paddleID = value.get<String>(SP_Protocol::DataTypes::Specs::PADDLE_ID, "");
                paddle->specs.paddleType = (PaddleType)value.get<int>(SP_Protocol::DataTypes::Specs::PADDLE_TYPE, 0);
                paddle->specs.paddleModel = value.get<String>(SP_Protocol::DataTypes::Specs::PADDLE_MODEL, "");
                paddle->specs.length = value.get<float>(SP_Protocol::DataTypes::Specs::LENGTH, 2.0f);
                paddle->specs.bladeWeight = value.get<float>(SP_Protocol::DataTypes::Specs::BLADE_WEIGHT, 0.3f);
                paddle->specs.bladeCenter = value.get<float>(SP_Protocol::DataTypes::Specs::BLADE_CENTER, 0.20f);
                paddle->specs.bladeMomentInertia = value.get<float>(SP_Protocol::DataTypes::Specs::BLADE_MOMENT_INERTIA, 0.01f);
                paddle->specs.imuFrequency = value.get<int>(SP_Protocol::DataTypes::Specs::IMU_FREQUENCY, 0);
                paddle->specs.hasLeftBlade = value.get<bool>(SP_Protocol::DataTypes::Specs::HAS_LEFT_BLADE, false);
                paddle->specs.hasRightBlade = value.get<bool>(SP_Protocol::DataTypes::Specs::HAS_RIGHT_BLADE, false);
                paddle->specs.firmwareVersion = value.get<int>(SP_Protocol::DataTypes::Specs::FIRMWARE_VERSION, 0);
                paddle->specs.imuDistance = value.get<float>(SP_Protocol::DataTypes::Specs::IMU_DISTANCE, 0.0f);
                paddle->specs.axisDirection = (AxisDirection)value.get<int>(SP_Protocol::DataTypes::Specs::AXIS_DIRECTION, (int)Y_AXIS_LEFT);
                paddle->specs.axisDirectionSign = paddle->specs.axisDirection==Y_AXIS_LEFT || paddle->specs.axisDirection==Z_AXIS_LEFT || paddle->specs.axisDirection==X_AXIS_LEFT ? -1 : 1;
                
                Serial.println("Paddle specs:");
                Serial.printf("  Paddle ID: %s\n", paddle->specs.paddleID.c_str());
                Serial.printf("  Firmware version: %d\n", paddle->specs.firmwareVersion);
                Serial.printf("  Paddle model: %s\n", paddle->specs.paddleModel.c_str());
                Serial.printf("  Length: %.2f\n", paddle->specs.length);
                Serial.printf("  Paddle type: %d\n", paddle->specs.paddleType);
                Serial.printf("  Has left blade: %d\n", paddle->specs.hasLeftBlade);
                Serial.printf("  Has right blade: %d\n", paddle->specs.hasRightBlade);
                Serial.printf("  IMU distance: %.2f\n", paddle->specs.imuDistance);
                Serial.printf("  Axis direction: %d\n", paddle->specs.axisDirection);
                Serial.printf("  Blade weight: %.2f\n", paddle->specs.bladeWeight);
                Serial.printf("  Blade center: %.2f\n", paddle->specs.bladeCenter);
                Serial.printf("  Blade moment inertia: %.2f\n", paddle->specs.bladeMomentInertia);
                
                for (int i = 0; i < paddle->eventHandlerCount; i++) 
                    paddle->eventHandler[i]->onUpdateSpecs(paddle->specs, paddle);
            });

        // Обработка ориентации лопасти
        registerData(SP_Protocol::DataTypes::BLADE_ORIENTATION,
            [this](SP_Data* data) {
                SP_DataValue value(data);
                
                paddle->bladeOrientation.rightBladeAngle = value.get<float>(
                    SP_Protocol::DataTypes::BladeOrientation::RIGHT_BLADE_ANGLE, 0.0f);
                paddle->bladeOrientation.leftBladeAngle = value.get<float>(
                    SP_Protocol::DataTypes::BladeOrientation::LEFT_BLADE_ANGLE, 0.0f);
                paddle->bladeOrientation.rightBladeVector[0] = value.get<float>(
                    SP_Protocol::DataTypes::BladeOrientation::RIGHT_BLADE_VECTOR_X, 0.0f);
                paddle->bladeOrientation.rightBladeVector[1] = value.get<float>(
                    SP_Protocol::DataTypes::BladeOrientation::RIGHT_BLADE_VECTOR_Y, 0.0f);
                paddle->bladeOrientation.rightBladeVector[2] = value.get<float>(
                    SP_Protocol::DataTypes::BladeOrientation::RIGHT_BLADE_VECTOR_Z, 0.0f);
                paddle->bladeOrientation.leftBladeVector[0] = value.get<float>(
                    SP_Protocol::DataTypes::BladeOrientation::LEFT_BLADE_VECTOR_X, 0.0f);
                paddle->bladeOrientation.leftBladeVector[1] = value.get<float>(
                    SP_Protocol::DataTypes::BladeOrientation::LEFT_BLADE_VECTOR_Y, 0.0f);
                paddle->bladeOrientation.leftBladeVector[2] = value.get<float>(
                    SP_Protocol::DataTypes::BladeOrientation::LEFT_BLADE_VECTOR_Z, 0.0f);
                
                
                Serial.println("Blade orientation:");
                Serial.printf("  Right blade angle: %.1f°\n", paddle->bladeOrientation.rightBladeAngle);
                Serial.printf("  Left blade angle: %.1f°\n", paddle->bladeOrientation.leftBladeAngle);
                
//                SetPaddleSpecs(paddle);

                for (int i = 0; i < paddle->eventHandlerCount; i++) 
                    paddle->eventHandler[i]->onUpdateBladeAngle(paddle->bladeOrientation, paddle);
            });

        // Обработка калибровочных данных магнитометра
        registerData(SP_Protocol::DataTypes::MAGNETOMETER_CALIBRATION,
            [this](SP_Data* data) {
                SP_DataValue value(data);
                
                float offset[3];
                float softIron[6];
                
                offset[0] = value.get<float>(SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_X, 0.0f);
                offset[1] = value.get<float>(SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_Y, 0.0f);
                offset[2] = value.get<float>(SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_Z, 0.0f);
                
                softIron[0] = value.get<float>(SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_0_0, 1.0f);
                softIron[1] = value.get<float>(SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_1_1, 1.0f);
                softIron[2] = value.get<float>(SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_2_2, 1.0f);
                softIron[3] = value.get<float>(SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_0_1, 0.0f);
                softIron[4] = value.get<float>(SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_0_2, 0.0f);
                softIron[5] = value.get<float>(SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_1_2, 0.0f);
                
                Serial.println("Got Magnetometer calibration data");
                Serial.printf("  Offset: %.2f, %.2f, %.2f\n", offset[0], offset[1], offset[2]);
                Serial.printf("  Soft iron (diag): %.6f, %.6f, %.6f\n", softIron[0], softIron[1], softIron[2]);
                Serial.printf("  Soft iron (off): %.6f, %.6f, %.6f\n", softIron[3], softIron[4], softIron[5]);
                
//                SetPaddleSpecs(paddle);
                // SetPaddleCalibration(paddle);
            });
    }

public:
    SPClient_MessageHandler(SmartPaddleBLEClient* p) : paddle(p) {
        setupHandlers();
    }

    // ============================================
    // ПЕРЕОПРЕДЕЛЕНИЕ БАЗОВЫХ ОБРАБОТЧИКОВ
    // ============================================
    
    void onLog(SP_LogMessage* log) override {
        Serial.printf("Paddle log: %s\n", log->message.c_str());
    }

    void onUnknownCommand(SP_Command* command) override {
        Serial.printf("Paddle command: %s\n", command->command.c_str());
    }

    void onResponse(SP_Response* response) override {
        Serial.printf("Paddle response: %s\n", response->command.c_str());
    }

    void onStatus(SP_StatusMessage* status) override {
        Serial.println("Got Paddle status");
    }
};






class SPBLEClientCallbacks : public BLEClientCallbacks {
private:
    SmartPaddleBLEClient* client;
    
public:
    SPBLEClientCallbacks(SmartPaddleBLEClient* c) : client(c) {}
    
    void onConnect(BLEClient* pClient) override {
        Serial.println("Client connected");
        client->isConnected = true;
    }
    
    void onDisconnect(BLEClient* pClient) override {

        Serial.printf("On disconnect\n");

        client->do_disconnect=true;
    }
    
    void onAuthenticationComplete(esp_ble_auth_cmpl_t auth_cmpl) {
        if(auth_cmpl.success) {
            Serial.println("Authentication successful");
        } else {
            Serial.println("Authentication failed");
            client->isConnected = false;
            client->disconnect();
            client->do_scan=true;
        }
    }
};


class SPClientRTOS{
    private:

    // Задача обработки BLE

    static void bleSendTask(void *pvParameters) {
        SmartPaddleBLEClient* paddle = (SmartPaddleBLEClient*)pvParameters;
        TickType_t xLastWakeTime = xTaskGetTickCount();
        uint32_t frequency = paddle->bleSendFrequency;
        if (frequency == 0){
            vTaskDelete(NULL);
            return;
        }
        const TickType_t xFrequency = pdMS_TO_TICKS(1000/frequency); 
        while(1) {
            paddle->updateBLE();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    static void bleReceiveTask(void *pvParameters) {
        SmartPaddleBLEClient* paddle = (SmartPaddleBLEClient*)pvParameters;
        TickType_t xLastWakeTime = xTaskGetTickCount();
        uint32_t frequency = paddle->bleReceiveFrequency;
        if (frequency == 0){
            vTaskDelete(NULL);
            return;
        }
        const TickType_t xFrequency = pdMS_TO_TICKS(1000/frequency); 
        
        while(1) {
            if(paddle->serial) 
                paddle->serial->updateJSON(true);
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    static void eventTask(void *pvParameters) {
        SmartPaddleBLEClient* paddle = (SmartPaddleBLEClient*)pvParameters;
        IMUData imuData;
        imuData.timestamp=UINT32_MAX;
        OrientationData orientationData;
        orientationData.timestamp=UINT32_MAX;
        loadData loadData;
        loadData.timestamp=UINT32_MAX;
        bool event_updated=false;
        while(1) {
            if (imuData.timestamp==UINT32_MAX) {
                paddle->receiveIMUData(imuData);
            }
            if (orientationData.timestamp==UINT32_MAX) {
                paddle->receiveOrientationData(orientationData);
            }
            if (loadData.timestamp==UINT32_MAX) {
                paddle->receiveLoadData(loadData);
            }

            if (imuData.timestamp<orientationData.timestamp) {
                if(loadData.timestamp<imuData.timestamp) {
                    for (int i = 0; i < paddle->eventHandlerCount; i++) 
                        paddle->eventHandler[i]->onUpdateLoad(loadData, paddle);
                    loadData.timestamp=UINT32_MAX;
                    event_updated=true;
                }
                else {
                    if (imuData.timestamp<UINT32_MAX){ 
                        for (int i = 0; i < paddle->eventHandlerCount; i++) 
                            paddle->eventHandler[i]->onUpdateIMU(imuData, paddle);
                        imuData.timestamp=UINT32_MAX;
                        event_updated=true;
                    }
                }
            }
            else {
                if(loadData.timestamp<orientationData.timestamp) {
                    for (int i = 0; i < paddle->eventHandlerCount; i++) 
                        paddle->eventHandler[i]->onUpdateLoad(loadData, paddle);
                    loadData.timestamp=UINT32_MAX;
                    event_updated=true;
                }
                else {
                    if (orientationData.timestamp<UINT32_MAX){ 
                        for (int i = 0; i < paddle->eventHandlerCount; i++) 
                            paddle->eventHandler[i]->onUpdateOrientation(orientationData, paddle);
                        orientationData.timestamp=UINT32_MAX;
                        event_updated=true;
                    }
                }
            }
            if (!event_updated) {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            }
            event_updated=false;
        }
    }

    public:

    static void startTasks(SmartPaddleBLEClient* paddle) {
    
        // Задачи на ядре 1
        xTaskCreatePinnedToCore(
            SPClientRTOS::bleSendTask,
            "BLESend",
            BLE_STACK_SIZE,
            paddle,
            2,  // Более высокий приоритет для send
            &paddle->bleSendTaskHandle,
            0
        );
    
        xTaskCreatePinnedToCore(
            SPClientRTOS::bleReceiveTask,
            "BLEReceive",
            BLE_RECEIVE_STACK_SIZE,
            paddle,
            1,
            &paddle->bleReceiveTaskHandle,
            0
        );

        xTaskCreatePinnedToCore(
            SPClientRTOS::eventTask,
            "Event",
            EVENT_STACK_SIZE,
            paddle,
            1,
            &paddle->eventTaskHandle,
            1
        );
    }
};


SmartPaddleBLEClient::SmartPaddleBLEClient(const char* prefs_Name)
    : SmartPaddleBLE(),
      trustedDevice(nullptr),
      pClient(nullptr),
      clientCallbacks(nullptr),
      do_scan(false),
      do_connect(false),
      do_disconnect(false),
      prefsName(prefs_Name),
      loadsensorQueue(SENSOR_QUEUE_SIZE),
      orientationQueue(SENSOR_QUEUE_SIZE),
      imuQueue(SENSOR_QUEUE_SIZE),
      bleSendFrequency(SPClient_Default_Frequencies::BLE_SEND_FREQUENCY),
      bleReceiveFrequency(SPClient_Default_Frequencies::BLE_RECEIVE_FREQUENCY),
      timeDifference(UINT32_MAX),
      lastScanStartTime(0),
      scanInProgress(false),
      imuNotifyCallback(nullptr),
      orientationNotifyCallback(nullptr),
      forceNotifyCallback(nullptr),
      bladeNotifyCallback(nullptr)

 {
    serial=new SP_BLESerialClient(this);
    messageHandler=new SPClient_MessageHandler(this);
    serial->setMessageHandler(messageHandler);
    clientCallbacks = new SPBLEClientCallbacks(this);
    specs.axisDirection = Y_AXIS_LEFT;
    specs.length = 2.0f;
    specs.imuDistance = 0.00f;
    specs.bladeWeight = 0.3f;
    specs.bladeCenter = 0.20f;
    specs.bladeMomentInertia = 0.01;
    specs.firmwareVersion = 0;
    specs.paddleModel = "No MODEL";
    specs.hasLeftBlade = true;
    specs.hasRightBlade = true;
    specs.imuFrequency = 0;
    specs.axisDirectionSign = -1;
}


void SmartPaddleBLEClient::disconnect() {
    Serial.printf("Disconnecting\n");
    if(pClient) {
        // Отменяем подписки
        if (serial) serial->end();
        
        pClient->disconnect();
        delete pClient;
        pClient = nullptr;
    }
    isConnected = false;
    for (int i = 0; i < eventHandlerCount; i++) 
        eventHandler[i]->onDisconnect(this);

    // Сбрасываем флаг сканирования для немедленного начала нового цикла
    scanInProgress = false;
    lastScanStartTime = 0;
    do_scan=true;
    specs.paddleID = "";
    bladeOrientation.rightBladeAngle = 0;
    bladeOrientation.leftBladeAngle = 0;

}


// Колбэки для получения данных
void SmartPaddleBLEClient::forceCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    loadData data;
    if(length == sizeof(loadData)) {
        memcpy(&data, pData, sizeof(loadData));
        if (millis()-data.timestamp < timeDifference) {
            timeDifference=millis()-data.timestamp;
        }
//        Serial.printf("Load data:  timestamp: %d\n", data.timestamp);
        loadsensorQueue.send(data);
        current_loads_data=data;
        if (eventHandlerCount>0) xTaskNotifyGive(eventTaskHandle);
    }
}


void SmartPaddleBLEClient::imuCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {

    IMUData data;
    if(length == sizeof(IMUData)) {
        if (millis()-data.timestamp < timeDifference) {
            timeDifference=millis()-data.timestamp;
        }
        memcpy(&data, pData, sizeof(IMUData));    




        imuQueue.send(data);
        current_imu_data=data;
        if (eventHandlerCount>0) xTaskNotifyGive(eventTaskHandle);
    }


}   

void SmartPaddleBLEClient::orientationCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    OrientationData data;
    if(length == sizeof(OrientationData)) {
        memcpy(&data, pData, sizeof(OrientationData));
        if (millis()-data.timestamp < timeDifference) {
            timeDifference=millis()-data.timestamp;
        }


        
        orientationQueue.send(data);
        current_orientation_data=data;
        if (eventHandlerCount>0) xTaskNotifyGive(eventTaskHandle);
    }
}

// Методы для получения данных из очередей
bool SmartPaddleBLEClient::receiveLoadData(loadData& data, TickType_t timeout) {
    return loadsensorQueue.receive(data, timeout);
}

bool SmartPaddleBLEClient::receiveIMUData(IMUData& data, TickType_t timeout) {
    return imuQueue.receive(data, timeout);
}

bool SmartPaddleBLEClient::receiveOrientationData(OrientationData& data, TickType_t timeout) {
    return orientationQueue.receive(data, timeout);
}      

void SmartPaddleBLEClient::loadTrustedDevice() {
    if (trustedDevice) {
        delete trustedDevice;
    }
    trustedDevice = nullptr;
    
    Preferences prefs;
    prefs.begin(prefsName.c_str(), true);
    if (prefs.isKey("trustedDevice")) {
        esp_bd_addr_t address;
        prefs.getBytes("trustedDevice", &address, sizeof(esp_bd_addr_t));
        trustedDevice = new BLEAddress(address);
        Serial.printf("Loaded trusted device: %s\n", trustedDevice->toString().c_str());
    }
    prefs.end();
}

void SmartPaddleBLEClient::saveTrustedDevice(BLEAddress* address) {
    Preferences prefs;
    prefs.begin(prefsName.c_str(), false);
    prefs.putBytes("trustedDevice", address->getNative(), sizeof(esp_bd_addr_t));
    prefs.putString("trustedDevStr", address->toString().c_str());
    prefs.end();
    
    if (trustedDevice) {
        delete trustedDevice;
    }
    trustedDevice = new BLEAddress(*address);
    Serial.printf("Saved trusted device: %s\n", trustedDevice->toString().c_str());
}

void SmartPaddleBLEClient::clearTrustedDevice() {
    Preferences prefs;
    prefs.begin(prefsName.c_str(), false);
    prefs.remove("trustedDevice");
    prefs.remove("trustedDevStr");
    prefs.end();
    
    if (trustedDevice) {
        delete trustedDevice;
        trustedDevice = nullptr;
    }
    Serial.println("Cleared trusted device");
}

// Обновленный класс для сканирования
class SPBLEClientScanCallbacks: public BLEAdvertisedDeviceCallbacks {
private:
    SmartPaddleBLEClient* client;

    
public:
    SPBLEClientScanCallbacks(SmartPaddleBLEClient* c) 
        : client(c) {}
    
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        if(advertisedDevice.haveServiceUUID() && 
           advertisedDevice.getServiceUUID().equals(BLEUUID(SmartPaddleUUID::SERVICE_UUID))) {

            Serial.printf("Found device: %s\n", advertisedDevice.toString().c_str());
            
            if(client->isPairing()) {
                Serial.println("In pairing mode");
                // В режиме сопряжения сохраняем новое устройство
                BLEAddress* address = new BLEAddress(advertisedDevice.getAddress());
                client->saveTrustedDevice(address);
                delete address;
                client->is_pairing=false;
                BLEDevice::getScan()->stop();
                client->scanInProgress = false;

                client->do_connect=true;
            } 
            else if(client->trustedDevice && 
                   advertisedDevice.getAddress().equals(*client->trustedDevice)) {
                // Вне режима сопряжения подключаемся только к доверенному устройству
                Serial.println("Connecting to trusted device");
                BLEDevice::getScan()->stop();
                client->scanInProgress = false;

                client->do_connect=true;
            }
        }
    }
};

void SmartPaddleBLEClient::startPairing() {

    disconnect();
    is_pairing=true;
//    clearTrustedDevice();
    
    startScan(30);
}

void SmartPaddleBLEClient::startScan(uint32_t duration) {
    // Запускаем сканирование для поиска известного устройства
    
//    Serial.printf("Starting scan for %d seconds\n", duration);
    
    // Получаем объект сканирования
    BLEScan* pScan = BLEDevice::getScan();
    
    // Останавливаем предыдущее сканирование если оно было запущено
    if (scanInProgress) {
        Serial.println("Stopping previous scan...");
        pScan->stop();
        delay(100);
    }
    
    // Очищаем старые результаты для предотвращения утечки памяти
    pScan->clearResults();
    
    // Запускаем новое сканирование (неблокирующий режим)
    pScan->start(duration, false);
    scanInProgress = true;
    lastScanStartTime = millis();
}

void SmartPaddleBLEClient::begin(const char* deviceName) {
    Serial.println("Starting SmartPaddleBLEClient::begin()");

    do_connect=false;
    
    // Инициализация NVS если еще не инициализирован
/*    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        Serial.println("Erasing and reinitializing NVS...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    Serial.println("NVS ready for SmartPaddle client");
*/    
    // Инициализируем мониторинг здоровья BLE
    BLEHealthMonitor::begin();
    
    BLEDevice::init(deviceName);
    BLEDevice::setMTU(BLEMTU);
    
    // Настройка безопасности
    BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
    pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
    pSecurity->setCapability(ESP_IO_CAP_NONE);
    pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);    
    // Загрузка сохраненного устройства
    loadTrustedDevice();

    BLEScan* pScan = BLEDevice::getScan();
    Serial.println("Setting up scan callbacks");
    pScan->setAdvertisedDeviceCallbacks(new SPBLEClientScanCallbacks(this));
    Serial.println("Setting active scan");
    pScan->setActiveScan(true);
    pScan->setInterval(2000);
    pScan->setWindow(1000);

    if (trustedDevice) Serial.printf("Trusted device: %s\n", trustedDevice->toString().c_str());
    do_scan=true;

    if (!imuNotifyCallback)
        imuNotifyCallback = std::bind(&SmartPaddleBLEClient::imuCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
    if (!orientationNotifyCallback)
        orientationNotifyCallback = std::bind(&SmartPaddleBLEClient::orientationCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
    if (!forceNotifyCallback)
        forceNotifyCallback = std::bind(&SmartPaddleBLEClient::forceCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
//    SPClientRTOS::startTasks(this);
}


bool SmartPaddleBLEClient::connect() {
    if(!trustedDevice) {
        Serial.println("No trusted device to connect to");
        return false;
    }
    
    if(isConnected) {
        Serial.println("Already connected");
        return true;
    }
    
    Serial.printf("Connecting to trusted device: %s\n", trustedDevice->toString().c_str());
    
    // Проверяем, что предыдущий клиент был корректно удален
    if (pClient != nullptr) {
        Serial.println("⚠️ Previous client still exists, cleaning up...");
        pClient->disconnect();
        delete pClient;
        pClient = nullptr;
        delay(100);
    }
    
    pClient = BLEDevice::createClient();
    if (!pClient) {
        Serial.println("❌ Failed to create BLE client");
        return false;
    }
    
    // Используем callback из члена класса
    if (clientCallbacks) {
        pClient->setClientCallbacks(clientCallbacks);
        Serial.println("✓ Created client with callbacks");
    } else {
        Serial.println("⚠️ No client callbacks available");
    }

    // Подключение к серверу - эта операция блокирующая
    if(!pClient->connect(*trustedDevice)) {
        Serial.println("Connection failed");
        delete pClient;
        pClient = nullptr;
        do_scan=true;
        return false;
    }

    // ДОБАВЛЯЕМ задержки между BLE операциями
    //vTaskDelay(pdMS_TO_TICKS(100));
    delay(100);

    // Установка MTU с проверкой
    Serial.printf("Requesting MTU: %d\n", BLEMTU);
    if (pClient->setMTU(BLEMTU)) {
        delay(100); // Даем время на установку MTU
        Serial.printf("MTU successfully set to %d\n", BLEDevice::getMTU());
    } else {
        Serial.printf("Failed to set MTU, current MTU: %d\n", BLEDevice::getMTU());
    }
    
    delay(100);
    
    Serial.println("Connected to trusted device");
    
    // Получение сервиса
    pRemoteService = pClient->getService(SmartPaddleUUID::SERVICE_UUID);
    if(!pRemoteService) {
        Serial.println("Failed to find service");
        pClient->disconnect();
        delete pClient;
        pClient = nullptr;
        do_scan=true;
        return false;
    }
    
    //vTaskDelay(pdMS_TO_TICKS(50));
    delay(50);

    // Получение характеристик и подписка на уведомления
    serial->begin();
    
    //vTaskDelay(pdMS_TO_TICKS(50));
    delay(50);

    if(!setupCharacteristics()) {
        Serial.println("Failed to setup characteristics");
        pClient->disconnect();
        delete pClient;
        pClient = nullptr;
        do_scan=true;
        return false;
    }

//    Serial.printf("Max subscriptions: %d\n", CONFIG_BT_NIMBLE_MAX_CCCDS);
    Serial.printf("Current MTU: %d\n", BLEDevice::getMTU());
    isConnected = true;
    is_pairing=false;
    Serial.println("Connected successfully");
    for (int i = 0; i < eventHandlerCount; i++) 
        eventHandler[i]->onConnect(this);
    Serial.printf("Connected to %s\n", trustedDevice->toString().c_str());
    return true;
}

void SmartPaddleBLEClient::updateBLE() {
    // Проверяем здоровье BLE стека
    if (!BLEHealthMonitor::checkBLEHealth()) {
        Serial.println("⚠️ BLE unhealthy, skipping update");
        return;
    }
    
    if (do_connect) {
        Serial.printf("Connecting (free heap: %d bytes)\n", ESP.getFreeHeap());
        delay(50);
        do_connect=false;        
        connect();
        Serial.printf("Out from connect\n");
    }
    
    if(do_disconnect) {
        Serial.printf("Disconnecting (free heap: %d bytes)\n", ESP.getFreeHeap());
        disconnect();
        do_disconnect=false;
        Serial.printf("Out from disconnect\n");
    }
    
    if(do_scan && !connected()&&!do_disconnect&&!do_connect) {
        // Проверяем, прошло ли достаточно времени с последнего запуска сканирования
        uint32_t currentTime = millis();
        const uint32_t SCAN_DURATION = 5000;  // 5 секунд сканирование
        const uint32_t SCAN_PAUSE = 1000;     // 1 секунда пауза между циклами
        
        if (scanInProgress) {
            // Проверяем, не завершилось ли сканирование
            if (currentTime - lastScanStartTime > SCAN_DURATION) {
                scanInProgress = false;
//                Serial.println("Scan cycle completed");
            }
        } else {
            // Сканирование не идет - можем запустить новое после паузы
            if (currentTime - lastScanStartTime > (SCAN_DURATION + SCAN_PAUSE) || lastScanStartTime == 0) {
//                Serial.printf("Scanning for devices (free heap: %d bytes)\n", ESP.getFreeHeap());
                startScan(5);  // Сканирование 5 секунд
//                Serial.println("Started new scan cycle");
            }
        }
        // do_scan остается true, чтобы продолжать циклы сканирования
        // пока не подключимся (тогда connected() вернет true)
    }
  
    if (serial && connected()) {
        serial->update();

        if (!specsValid() && millis() > last_send_specs_time +1000) {
            last_send_specs_time = millis();
            serial->sendString(SP_MessageProcessor::createSendSpecsCommand());
        }

        if (!bladeOrientationValid() && millis() > last_send_paddle_orientation_time +1500) {
            last_send_paddle_orientation_time = millis();
            serial->sendString(SP_MessageProcessor::createSendPaddleOrientationCommand());
        }
    }

}

void SmartPaddleBLEClient::calibrateIMU() {
    if (serial) serial->sendString(SP_MessageProcessor::createCalibrateIMUCommand());
}

void SmartPaddleBLEClient::calibrateLoads(BladeSideType blade_side) {
    if (serial) {
        serial->sendString(SP_MessageProcessor::createCalibrateLoadsCommand(blade_side));    
    }
}

// Приватный метод для настройки характеристик
bool SmartPaddleBLEClient::setupCharacteristics() {
    if (!pRemoteService) {
        Serial.println("Remote service is NULL");
        return false;
    }
    
    // Получение характеристик с дополнительными проверками
    Serial.println("Getting characteristics...");
    
    forceChar = pRemoteService->getCharacteristic(SmartPaddleUUID::FORCE_UUID);
    if (!forceChar) {
        Serial.println("Failed to get force characteristic");
        return false;
    }
    //vTaskDelay(pdMS_TO_TICKS(10));
    delay(10);
    
    imuChar = pRemoteService->getCharacteristic(SmartPaddleUUID::IMU_UUID);
    if (!imuChar) {
        Serial.println("Failed to get IMU characteristic");
        return false;
    }
    //vTaskDelay(pdMS_TO_TICKS(10));
    delay(10);
    
    orientationChar = pRemoteService->getCharacteristic(SmartPaddleUUID::ORIENTATION_UUID);
    if (!orientationChar) {
        Serial.println("Failed to get orientation characteristic");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // ИЗБЕГАЕМ автоматического получения дескрипторов
    // Подписываемся только если характеристика поддерживает notify
    Serial.println("Setting up notifications...");
    
    if (forceChar->canNotify()) {
        try {
            forceChar->registerForNotify(forceNotifyCallback, false); // false = не получать дескрипторы автоматически
            //vTaskDelay(pdMS_TO_TICKS(50));
            delay(50);
        } catch (...) {
            Serial.println("Failed to register force notify");
        }
    }
    
    if (imuChar->canNotify()) {
        try {
            imuChar->registerForNotify(imuNotifyCallback, false);
            //vTaskDelay(pdMS_TO_TICKS(50));
            delay(50);
        } catch (...) {
            Serial.println("Failed to register IMU notify");
        }
    }
    
    if (orientationChar->canNotify()) {
        try {
            orientationChar->registerForNotify(orientationNotifyCallback, false);
            //vTaskDelay(pdMS_TO_TICKS(50));
            delay(50);
        } catch (...) {
            Serial.println("Failed to register orientation notify");
        }
    }
    
    return true;
}

void SmartPaddleBLEClient::shutdown() {
    if (serial) serial->sendString(SP_MessageProcessor::createShutdownCommand());
}

void SmartPaddleBLEClient::calibrateBladeAngle(BladeSideType blade_side) {
    if (serial) {
        serial->sendString(SP_MessageProcessor::createCalibrateBladeAngleCommand(blade_side));    
    }
}

IMUData SmartPaddleBLEClient::getIMUData(){
    return current_imu_data;
}

loadData SmartPaddleBLEClient::getLoadData(){
    return current_loads_data;
}

OrientationData SmartPaddleBLEClient::getOrientationData(){
    return current_orientation_data;
}

uint32_t SmartPaddleBLEClient::paddleMillis(){
    return ::millis()-timeDifference;
}

void SmartPaddleBLEClient::startTasks() {
    SPClientRTOS::startTasks(this);
}
