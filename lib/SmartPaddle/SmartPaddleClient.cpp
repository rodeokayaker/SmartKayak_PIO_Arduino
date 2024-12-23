#include <SmartPaddleClient.h>
#include <SmartPaddle.h>
#include <Preferences.h>
#include <SP_BLESerialClient.h>
#include <SP_BLESerial.h>



extern bool log_imu;
extern bool log_load;

class SerialClient_MessageHandler: public BLESerialMessageHandler{
    private:
    SmartPaddleBLEClient* paddle;

    public:
    SerialClient_MessageHandler(SmartPaddleBLEClient* paddle):paddle(paddle){}

    void onLogMessage(const char* message){
        Serial.printf("Paddle log: %s\n", message);
    }
    void onCommand(const char* command, JsonObject& params){
        Serial.printf("Paddle command: %s\n", command);
    }
    void onResponse(const char* command, bool success, const char* message){
        Serial.printf("Paddle response: %s\n", command);
    }
    void onData(const char* dataType, JsonObject& data){
        Serial.printf("Paddle data: %s\n", dataType);
    }
    void onStatus(JsonObject& status){
        Serial.printf("Got Paddle status\n");
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
        Serial.println("Client disconnected");
//        client->isConnected = false;
        client->disconnect();
        delay(100);
        // Если не в режиме сопряжения, пытаемся переподключиться
        if(!client->isPairing()) {
            client->do_scan=true;
        }
    }
    
    void onAuthenticationComplete(esp_ble_auth_cmpl_t auth_cmpl) {
        if(auth_cmpl.success) {
            Serial.println("Authentication successful");
        } else {
            Serial.println("Authentication failed");
            client->isConnected = false;
            client->disconnect();
        }
    }
};

//NEED TO FIX THIS to use more than one paddle!!!!
OverwritingQueue<loadData> SmartPaddleBLEClient::loadsensorQueue(SENSOR_QUEUE_SIZE);
OverwritingQueue<OrientationData> SmartPaddleBLEClient::orientationQueue(SENSOR_QUEUE_SIZE);
OverwritingQueue<IMUData> SmartPaddleBLEClient::imuQueue(SENSOR_QUEUE_SIZE);
OverwritingQueue<BladeData> SmartPaddleBLEClient::bladeQueue(SENSOR_QUEUE_SIZE);
OverwritingQueue<PaddleStatus> SmartPaddleBLEClient::statusQueue(1);
PaddleSpecs clientSpecs;


//BAD Situation
PaddleSpecs SmartPaddleBLEClient::getSpecs(){
    specs=clientSpecs;
    return clientSpecs;
}

SmartPaddleBLEClient::SmartPaddleBLEClient(const char* prefs_Name)
    : SmartPaddle(),
      trustedDevice(nullptr),
      pClient(nullptr),
      do_scan(false),
      do_connect(false),
      prefsName(prefs_Name),
      last_orientation_ts(0),
      last_blade_ts(0),
      last_imu_ts(0),
      last_loads_ts(0) {
    serial=new SP_BLESerialClient(this);
    messageHandler=new SerialClient_MessageHandler(this);
    serial->setMessageHandler(messageHandler);
}


void SmartPaddleBLEClient::disconnect() {
    if(pClient) {
        PaddleMap.erase((void*)pClient);
        serial->end();
        pClient->disconnect();
        delete pClient;
        pClient = nullptr;
    }
    isConnected = false;
}

static uint32_t last_notify_time = millis();
static float frequency_notify = 50;


// Колбэки для получения данных
void SmartPaddleBLEClient::forceCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
//    Serial.printf("Force callback length = %d\n", length);
    loadData data;
    if(length == sizeof(loadData)) {
        memcpy(&data, pData, sizeof(loadData));
        if (log_load) {
            Serial.printf("Loads: %d, %d, ts: %d\n", data.forceR, data.forceL, data.timestamp);
        }
        loadsensorQueue.send(data);
    }
}


void SmartPaddleBLEClient::imuCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
//    Serial.printf("IMU callback length = %d\n", length);
    int32_t time_diff = millis()-last_notify_time;
    last_notify_time = millis();
    frequency_notify = 100.0/(99.0/frequency_notify+time_diff/1000.0);
//    Serial.printf("IMU callback time = %d, frequency = %f\n", time_diff, frequency_notify);
    IMUData data;

    if(length == sizeof(IMUData)) {

        memcpy(&data, pData, sizeof(IMUData));
        
//        Serial.printf("IMU imestamp: %u\n",data.timestamp);


/*            Serial.printf("IMU: ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f mx=%.2f my=%.2f mz=%.2f ts=%u\n",
                data.ax, data.ay, data.az,
                data.gx, data.gy, data.gz,
                data.mx, data.my, data.mz,
                data.timestamp);*/

        if (log_imu) {
            Serial.printf("IMU: ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f mx=%.2f my=%.2f mz=%.2f ts=%u\n",
                data.ax, data.ay, data.az,
                data.gx, data.gy, data.gz,
                data.mx, data.my, data.mz,
                data.timestamp);
            Serial.printf("DMP: q0=%.3f q1=%.3f q2=%.3f q3=%.3f ts=%u\n",
                data.q0, data.q1, data.q2, data.q3, data.timestamp);
        }
    
        imuQueue.send(data);
    }


}   

void SmartPaddleBLEClient::statusCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    Serial.println("Status callback");
    PaddleStatus data;
    if(length == sizeof(PaddleStatus)) {
        memcpy(&data, pData, sizeof(PaddleStatus));
        statusQueue.send(data);
    }
}

void SmartPaddleBLEClient::orientationCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
//    Serial.printf("Orientation callback length = %d\n", length);
    OrientationData data;
    if(length == sizeof(OrientationData)) {
        memcpy(&data, pData, sizeof(OrientationData));
        if (log_imu) {
            Serial.printf("Orientation: q0=%.3f q1=%.3f q2=%.3f q3=%.3f ts=%u\n",
                data.q0, data.q1, data.q2, data.q3, data.timestamp);
        }
        orientationQueue.send(data);
    }
}

void SmartPaddleBLEClient::bladeCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    Serial.println("Blade callback");
    BladeData data;
    if(length == sizeof(BladeData)) {
        memcpy(&data, pData, sizeof(BladeData));
        bladeQueue.send(data);
    }
}

void SmartPaddleBLEClient::specsCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    Serial.println("Specs callback");
    if(length == sizeof(PaddleSpecs)) {
        memcpy(&clientSpecs, pData, sizeof(PaddleSpecs));
    }
}

// Методы для получения данных из очередей
bool SmartPaddleBLEClient::getLoadData(loadData& data, TickType_t timeout) {
    return loadsensorQueue.receive(data, timeout);
}

bool SmartPaddleBLEClient::getIMUData(IMUData& data, TickType_t timeout) {

    //READ LAST DATA
    if (imuQueue.available() > 0) {
//        while (imuQueue.available() > 0) {
            imuQueue.receive(data, timeout);
//        }
        return true;
    }
    return false;
}

bool SmartPaddleBLEClient::getStatusData(PaddleStatus& data, TickType_t timeout) {
    return statusQueue.receive(data, timeout);
}   

bool SmartPaddleBLEClient::getOrientationData(OrientationData& data, TickType_t timeout) {
    //READ LAST DATA
    if (orientationQueue.available() > 0) {
//        while (orientationQueue.available() > 0) {
            orientationQueue.receive(data, timeout);
//        }
        return true;
    }
    return false;
}      

bool SmartPaddleBLEClient::getBladeData(BladeData& data, TickType_t timeout) {
    return bladeQueue.receive(data, timeout);
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
    pScan->setWindow(1500);

    if (trustedDevice) Serial.printf("Trusted device: %s\n", trustedDevice->toString().c_str());
    do_scan=true;

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

    // Подключение к серверу
    if(!pClient->connect(*trustedDevice)) {
        Serial.println("Connection failed");
        delete pClient;
        pClient = nullptr;
        return false;
    }

    if (pClient->setMTU(BLEMTU)) {
        Serial.printf("Set MTU to %d\n", BLEMTU);
    } else {
        Serial.printf("Failed to set MTU to %d\n", BLEMTU);
    }
    
    Serial.println("Connected to trusted device");
    
    // Получение сервиса
    pRemoteService = pClient->getService(SmartPaddleUUID::SERVICE_UUID);
    if(!pRemoteService) {
        Serial.println("Failed to find service");
        pClient->disconnect();
        delete pClient;
        pClient = nullptr;
        return false;
    }

    PaddleMap.insert(std::make_pair((void*)pClient, this));

    // Получение характеристик и подписка на уведомления
    serial->begin();

    if(!setupCharacteristics()) {
        Serial.println("Failed to setup characteristics");
        pClient->disconnect();
        delete pClient;
        pClient = nullptr;
        return false;
    }



    Serial.printf("Max subscriptions: %d\n", CONFIG_BT_NIMBLE_MAX_CCCDS);
    Serial.printf("Current MTU: %d\n", BLEDevice::getMTU());


    
    isConnected = true;
    is_pairing=false;

    Serial.println("Connected successfully");
    return true;
}

void SmartPaddleBLEClient::updateBLE() {
    if (do_connect) {
        delay(100);
        connect();
        do_connect=false;
    }
    if(do_scan) {
        do_scan=false;
        startScan(0);
    }

    if(!isConnected && trustedDevice) {
        // Если есть доверенное устройство, но нет подключения,
        // запускаем сканирование для его поиска
//        startScan(5);    
        return;
    }

    // Чтение данных из характеристик
    if(isConnected) {
        // Чтение данных с тензодатчиков
/*        if(forceChar) {
            std::string value = forceChar->readValue();
            if(value.length() >= sizeof(loadData)) {
                loadData data;
                memcpy(&data, value.c_str(), sizeof(loadData));
                if(data.timestamp > last_loads_ts) {
                    loadsensorQueue.send(data);
                    last_loads_ts = data.timestamp;
                }
                if(log_load) {
                    Serial.printf("Force: L=%d R=%d ts=%u\n", 
                        data.forceL, data.forceR, data.timestamp);
                }
            }
        }*/

        // Чтение данных IMU
/*        if(imuChar) {
            std::string value = imuChar->readValue();
            if(value.length() >= sizeof(IMUData)) {
                IMUData data;
                memcpy(&data, value.c_str(), sizeof(IMUData));
                if(data.timestamp > last_imu_ts) {
                    imuQueue.send(data);
                    last_imu_ts = data.timestamp;
                }
                
                if(log_imu) {
                    Serial.printf("IMU: ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f mx=%.2f my=%.2f mz=%.2f ts=%u\n",
                        data.ax, data.ay, data.az,
                        data.gx, data.gy, data.gz,
                        data.mx, data.my, data.mz,
                        data.timestamp);
                    Serial.printf("DMP: q0=%.3f q1=%.3f q2=%.3f q3=%.3f ts=%u\n",
                        data.q0, data.q1, data.q2, data.q3, data.timestamp);
                }
            }
        }*/

        // Чтение данных ориентации
/*        if(orientationChar) {
            std::string value = orientationChar->readValue();
            if(value.length() >= sizeof(OrientationData)) {
                OrientationData data;
                memcpy(&data, value.c_str(), sizeof(OrientationData));
                if(data.timestamp > last_orientation_ts) {
                    orientationQueue.send(data);
                    last_orientation_ts = data.timestamp;
                }
                
                if(log_imu) {
                    Serial.printf("Orientation: q0=%.3f q1=%.3f q2=%.3f q3=%.3f ts=%u\n",
                        data.q0, data.q1, data.q2, data.q3, data.timestamp);
                }
            }
        }*/
    }

    if (serial) serial->update();
}

void SmartPaddleBLEClient::calibrateIMU() {
    if (serial) serial->sendCommand("calibrate_imu");
}

void SmartPaddleBLEClient::calibrateLoads(BladeSideType blade_side) {
    if (serial) {
        JsonDocument doc;
        JsonObject params=doc.to<JsonObject>();
        params["blade_side"] = blade_side;
        switch(blade_side){
            case RIGHT_BLADE:
            serial->sendCommand("calibrate_loads_right", &params);    
            break;
            case LEFT_BLADE:
            serial->sendCommand("calibrate_loads_left", &params);            
            break;
            case ALL_BLADES:
            serial->sendCommand("calibrate_loads", &params);
            break;
        }
    }
}

// Приватный метод для настройки характеристик
bool SmartPaddleBLEClient::setupCharacteristics() {
    // Получение характеристик
    forceChar = pRemoteService->getCharacteristic(SmartPaddleUUID::FORCE_UUID);
    imuChar = pRemoteService->getCharacteristic(SmartPaddleUUID::IMU_UUID);
    statusChar = pRemoteService->getCharacteristic(SmartPaddleUUID::STATUS_UUID);
    specsChar = pRemoteService->getCharacteristic(SmartPaddleUUID::SPECS_UUID);
    orientationChar = pRemoteService->getCharacteristic(SmartPaddleUUID::ORIENTATION_UUID);
    bladeChar = pRemoteService->getCharacteristic(SmartPaddleUUID::BLADE_UUID);

    if(!forceChar || !imuChar || !statusChar || !specsChar || 
       !orientationChar || !bladeChar) {
        Serial.println("Failed to get one or more characteristics");
        return false;
    }
    Serial.println("Got all characteristics");
    
    imuChar->registerForNotify(imuCallback);
    forceChar->registerForNotify(forceCallback);
    orientationChar->registerForNotify(orientationCallback);
//    statusChar->registerForNotify(statusCallback);
    specsChar->registerForNotify(specsCallback);
//    bladeChar->registerForNotify(bladeCallback);
    
    return true;
}

void SmartPaddleBLEClient::shutdown() {
    if (serial) serial->sendCommand("shutdown");
}
