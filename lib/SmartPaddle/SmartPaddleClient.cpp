#include <SmartPaddleClient.h>
#include <SmartPaddle.h>
#include <Preferences.h>
#include <SP_BLESerialClient.h>
#include <SP_BLESerial.h>

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
    paddle->getSerial()->sendString(SP_MessageProcessor::createSpecsMessage(specs));
}


class SPClient_MessageHandler: public SP_MessageHandler{
    private:
    SmartPaddleBLEClient* paddle;

    public:
    SPClient_MessageHandler(SmartPaddleBLEClient* paddle):paddle(paddle){}

    virtual void onLog(SP_LogMessage* log) override{
        Serial.printf("Paddle log: %s\n", log->message.c_str());
    }
    virtual void onCommand(SP_Command* command) override{
        Serial.printf("Paddle command: %s\n", command->command.c_str());
    }
    virtual void onResponse(SP_Response* response) override{
        Serial.printf("Paddle response: %s\n", response->command.c_str());
    }

    virtual void onSpecsData(SP_Data* data, const PaddleSpecs& specs) override{
        paddle->specs = specs;
        Serial.printf("Paddle specs: \n");
        Serial.printf("Paddle ID: %s\n", paddle->specs.paddleID);
        Serial.printf("Firmware version: %d\n", paddle->specs.firmwareVersion);

        Serial.printf("Paddle model: %s\n", paddle->specs.paddleModel);
        Serial.printf("Length: %f\n", paddle->specs.length);
        Serial.printf("Paddle type: %d\n", paddle->specs.paddleType);
        Serial.printf("Has left blade: %d\n", paddle->specs.hasLeftBlade);
        Serial.printf("Has right blade: %d\n", paddle->specs.hasRightBlade);
        Serial.printf("IMU distance: %f\n", paddle->specs.imuDistance);


    }

    virtual void onBladeOrientationData(SP_Data* data, const BladeOrientation& bladeOrientation) override{
        paddle->bladeOrientation = bladeOrientation;
        Serial.printf("Blade orientation: \n");
        Serial.printf("Y axis direction: %d\n", paddle->bladeOrientation.YAxisDirection);
        Serial.printf("Right blade angle: %f\n", paddle->bladeOrientation.rightBladeAngle);
        Serial.printf("Left blade angle: %f\n", paddle->bladeOrientation.leftBladeAngle);
        SetPaddleSpecs(paddle);
    }

    virtual void onStatus(SP_StatusMessage* status) override{
        Serial.printf("Got Paddle status\n");

    }

    virtual void onMagnetometerCalibrationData(SP_Data* data, float* offset, float* softIron) override{
        Serial.printf("Got Magnetometer calibration data\n");
        Serial.printf("Magnetometer calibration data: %f, %f, %f\n", offset[0], offset[1], offset[2]); 
        Serial.printf("Magnetometer calibration data: %f, %f, %f\n", softIron[0], softIron[1], softIron[2]);
        Serial.printf("Magnetometer calibration data: %f, %f, %f\n", softIron[3], softIron[4], softIron[5]);
        SetPaddleSpecs(paddle);
//        SetPaddleCalibration(paddle);
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
                    paddle->eventHandler->onUpdateLoad(loadData, paddle);
                    loadData.timestamp=UINT32_MAX;
                    event_updated=true;
                }
                else {
                    paddle->eventHandler->onUpdateIMU(imuData, paddle);
                    imuData.timestamp=UINT32_MAX;
                    event_updated=true;
                }
            }
            else {
                if(loadData.timestamp<orientationData.timestamp) {
                    paddle->eventHandler->onUpdateLoad(loadData, paddle);
                    loadData.timestamp=UINT32_MAX;
                    event_updated=true;
                }
                else {
                    if (orientationData.timestamp<UINT32_MAX){ 
                        paddle->eventHandler->onUpdateOrientation(orientationData, paddle);
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
    : SmartPaddle(),
      trustedDevice(nullptr),
      pClient(nullptr),
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
      imuNotifyCallback(nullptr),
      orientationNotifyCallback(nullptr),
      forceNotifyCallback(nullptr),
      bladeNotifyCallback(nullptr)

 {
    serial=new SP_BLESerialClient(this);
    messageHandler=new SPClient_MessageHandler(this);
    serial->setMessageHandler(messageHandler);
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
    if (eventHandler)
        eventHandler->onDisconnect(this);

    do_scan=true;

}


// Колбэки для получения данных
void SmartPaddleBLEClient::forceCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    loadData data;
    if(length == sizeof(loadData)) {
        memcpy(&data, pData, sizeof(loadData));
        if (millis()-data.timestamp < timeDifference) {
            timeDifference=millis()-data.timestamp;
        }
        loadsensorQueue.send(data);
        current_loads_data=data;
        if (eventHandler) xTaskNotifyGive(eventTaskHandle);
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
        if (eventHandler) xTaskNotifyGive(eventTaskHandle);
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
        if (eventHandler) xTaskNotifyGive(eventTaskHandle);
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
    prefs.putString("trustedDevice_string", address->toString().c_str());
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
    prefs.remove("trustedDevice_string");
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

                client->do_connect=true;
            } 
            else if(client->trustedDevice && 
                   advertisedDevice.getAddress().equals(*client->trustedDevice)) {
                // Вне режима сопряжения подключаемся только к доверенному устройству
                Serial.println("Connecting to trusted device");
                BLEDevice::getScan()->stop();

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
    
    Serial.printf("Starting scan for %d seconds\n", duration);
    BLEDevice::getScan()->clearResults();
    BLEDevice::getScan()->start(duration, false);
}

void SmartPaddleBLEClient::begin(const char* deviceName) {

    do_connect=false;
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
    
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new SPBLEClientCallbacks(this));
    Serial.println("Created client");

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

    // Установка MTU
    if (pClient->setMTU(BLEMTU)) {
        Serial.printf("Set MTU to %d\n", BLEMTU);
    } else {
        Serial.printf("Failed to set MTU to %d\n", BLEMTU);
    }
    
    //vTaskDelay(pdMS_TO_TICKS(100));
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

    Serial.printf("Max subscriptions: %d\n", CONFIG_BT_NIMBLE_MAX_CCCDS);
    Serial.printf("Current MTU: %d\n", BLEDevice::getMTU());
    isConnected = true;
    is_pairing=false;
    Serial.println("Connected successfully");
    if (eventHandler)
        eventHandler->onConnect(this);
    Serial.printf("Connected to %s\n", trustedDevice->toString().c_str());
    return true;
}

void SmartPaddleBLEClient::updateBLE() {
    if (do_connect) {
        Serial.printf("Connecting\n");
        delay(50);
        do_connect=false;        
        connect();
        Serial.printf("Out from connect\n");
    }
  if(do_disconnect) {
        Serial.printf("Disconnecting\n");
        disconnect();
        do_disconnect=false;
        Serial.printf("Out from disconnect\n");
    }
    if(do_scan && !connected()&&!do_disconnect&&!do_connect) {
        Serial.println("Scanning for devices");
        do_scan=false;
        startScan(0);
        if (!do_connect&&!connected()) {
            do_scan=true;
        }
        Serial.printf("Out from scan\n");
    }
  
    if (serial) serial->update();
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
