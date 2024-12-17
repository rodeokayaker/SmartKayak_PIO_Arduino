#include <SmartPaddleServer.h>
#include <Preferences.h>
#include <SP_BLESerialServer.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <MadgwickAHRS.h>
#include <esp_gatts_api.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


class SerialServer_MessageHandler: public BLESerialMessageHandler{
    private:
    SmartPaddleBLEServer* paddle;

    public:
    SerialServer_MessageHandler(SmartPaddleBLEServer* paddle):paddle(paddle){}

    void onLogMessage(const char* message){
        Serial.printf("Kayak log: %s\n", message);
    }
    void onCommand(const char* command, JsonObject& params){
        Serial.printf("Paddle command: %s\n", command);
        if (strcmp(command, "calibrate_imu") == 0) {
            paddle->setLogStream(paddle->getSerial());
            paddle->calibrateIMU();
            paddle->setLogStream(&Serial);
            return;
        }
        if (strcmp(command, "calibrate_loads") == 0) {
            paddle->setLogStream(paddle->getSerial());
            paddle->calibrateLoads(params["blade_side"]);
            paddle->setLogStream(&Serial);
            return;
        }
        if (strcmp(command, "send_specs") == 0) {
            paddle->sendSpecs();
            return;
        }
        if (strcmp(command, "start_pair") == 0) {
            paddle->startPairing();
            return;
        }
    }
    void onResponse(const char* command, bool success, const char* message){
        Serial.printf("Kayak response: %s\n", command);
    }
    void onData(const char* dataType, JsonObject& data){
        Serial.printf("Kayak data: %s\n", dataType);
    }
    void onStatus(JsonObject& status){
        Serial.printf("Got Kayak status\n");
    }
};

SmartPaddle::~SmartPaddle(){
    if(serial) {delete serial; serial=nullptr;}
    if(messageHandler) {delete messageHandler; messageHandler=nullptr;}
}

void SmartPaddleBLEServer::updateIMU() {
    if(imu) {
//        imu->update();
        IMUData imuData = imu->getData();
        imuQueue.send(imuData);
        
        OrientationData orientation = imu->getOrientation();
        orientationQueue.send(orientation);

        if (logInterface)
            logInterface->logQuaternion(&orientation.q0);

    }
}

void SmartPaddleBLEServer::calibrateIMU(){
    imu->calibrate();
}

void SmartPaddleBLEServer::calibrateLoads(BladeSideType blade_side){
    switch(blade_side){
        case LEFT_BLADE:
            if (loads[1])
                loads[1]->calibrate(); else
                logStream->println("No left blade");
            break;
        case RIGHT_BLADE:
            if (loads[0])
                loads[0]->calibrate(); else
                logStream->println("No right blade");
            break;
        case ALL_BLADES:
            if (loads[0])
                loads[0]->calibrate(); else
                logStream->println("No right blade");
            if (loads[1])
                loads[1]->calibrate(); else
                logStream->println("No left blade");
            break;
    }
}

void SmartPaddleBLEServer::updateLoads(){
    loadData data;
    data.forceR = (int32_t)loads[0]->getForce();
    if (loads[1])
        data.forceL = (int32_t)loads[1]->getForce();
    else
        data.forceL = 0;

//        Serial.printf("Loads: rigth: %d, left: %d\n",  data.forceR, data.forceL);
    
    data.timestamp=millis();
    loadsensorQueue.send(data);
}


// Конструктор класса SmartPaddle
SmartPaddleBLEServer::SmartPaddleBLEServer(const char* prefs_Name,uint16_t filterFrequency)
    : SmartPaddle(),
      orientationQueue(SENSOR_QUEUE_SIZE),
      imuQueue(SENSOR_QUEUE_SIZE),
      loadsensorQueue(SENSOR_QUEUE_SIZE),
      bladeQueue(SENSOR_QUEUE_SIZE),
      statusQueue(1),
      FilterFrequency(filterFrequency),
      trustedDevice(nullptr),
      send_specs(false),
      is_pairing(false),
      imu(nullptr),
      loads{nullptr,nullptr},
      prefsName(prefs_Name),
      logInterface(nullptr)
{
    run_madgwick=true;
    serial=new SP_BLESerialServer(this);
    messageHandler=new SerialServer_MessageHandler(this);
    serial->setMessageHandler(messageHandler);    

}

void SmartPaddleBLEServer::setLogStream(Stream* stream){
    logStream = stream;
    if (loads[0])
        loads[0]->setLogStream(stream);
    if (loads[1])
        loads[1]->setLogStream(stream);
    if (imu)
        imu->setLogStream(stream);
}

// класс для подключения и отключения устройства
class SPBLEServerCallbacks: public BLEServerCallbacks {
    private:
    SmartPaddleBLEServer* server;
    
    public:
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
        Serial.println("On Connect");
        BLEAddress clientAddress(param->connect.remote_bda);
        Serial.printf("Client address: %s\n", clientAddress.toString().c_str());
        server->conn_id=param->connect.conn_id;
        
        if (server->connected()){
            Serial.printf("Already connected to %s\n", server->trustedDevice->toString().c_str());
//            pServer->disconnect(param->connect.conn_id);
            return;
        }

        if (!server->trustedDevice || server->isPairing()) {
            // Если нет сохраненного доверенного устройства, сохраняем первое подключившееся
            Serial.println("No trusted device, allowing new connection");
            server->setTrustedDevice(&clientAddress);
            server->conn_id=param->connect.conn_id;
            server->connect();
        } 
        else if (server->trustedDevice->equals(clientAddress)) {
            // Если адрес совпадает с сохраненным доверенным устройством
            Serial.printf("Trusted device connected: %s\n", clientAddress.toString().c_str());
            server->conn_id=param->connect.conn_id;
            server->connect();
        } 
        else {
            // Если адрес не совпадает с доверенным устройством
            Serial.printf("Untrusted device rejected: %s\n", clientAddress.toString().c_str());
            pServer->disconnect(param->connect.conn_id);
        }
    }

    void onDisconnect(BLEServer* pServer) {
        server->disconnect();
    }

    SPBLEServerCallbacks(SmartPaddleBLEServer* serverPTR) : server(serverPTR) {}
};

class MySecurity : public BLESecurityCallbacks {
  
  bool onConfirmPIN(uint32_t pin){
    return false;
  }
  
	uint32_t onPassKeyRequest(){
        Serial.println("PassKeyRequest");
		return 123456;
	}

	void onPassKeyNotify(uint32_t pass_key){
        Serial.printf("On passkey Notify number:%d", pass_key);
	}

	bool onSecurityRequest(){
	    Serial.println("On Security Request");
		return true;
	}

	void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl){
		Serial.println("On Authentication Complete");
		if(cmpl.success){
			uint16_t length;
			esp_ble_gap_get_whitelist_size(&length);
			Serial.printf("size: %d", length);
		}
	}
};

void SmartPaddleBLEServer::begin(const char* deviceName) {

    loadTrustedDevice();
    Serial.println("Device name: " + String(deviceName));
    BLEDevice::init(deviceName);
    BLEDevice::setMTU(BLEMTU);
    BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
    BLEDevice::setSecurityCallbacks(new MySecurity());
    
    // Настройка безопасности с сохранением указателя
    pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
    pSecurity->setCapability(ESP_IO_CAP_NONE);
    pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    
    Serial.println("Creating server");
    pServer = BLEDevice::createServer();

    pServer->setCallbacks(new SPBLEServerCallbacks(this));

    Serial.println("Creating service");
    BLEService *pService = pServer->createService(BLEUUID(SmartPaddleUUID::SERVICE_UUID),30,0);
    forceCharacteristic = pService->createCharacteristic(
        SmartPaddleUUID::FORCE_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    Serial.println("Creating imu characteristic");
    imuCharacteristic = pService->createCharacteristic(
        SmartPaddleUUID::IMU_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    Serial.println("Creating status characteristic");
    statusCharacteristic = pService->createCharacteristic(
        SmartPaddleUUID::STATUS_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    Serial.println("Creating specs characteristic");
    specsCharacteristic = pService->createCharacteristic(
        SmartPaddleUUID::SPECS_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    Serial.println("Creating orientation characteristic");
    orientationCharacteristic = pService->createCharacteristic(
        SmartPaddleUUID::ORIENTATION_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    Serial.println("Creating blade characteristic");
    bladeCharacteristic = pService->createCharacteristic(
        SmartPaddleUUID::BLADE_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    Serial.println("Starting service");
    pService->start();
    serial->begin();
    Serial.println("Starting advertising");
    startAdvertising(BLEDevice::getAdvertising());
    Serial.println("Service started");
    Serial.printf("Bonded devices count: %d\n", esp_ble_get_bond_device_num());
}

void SmartPaddleBLEServer::startAdvertising(BLEAdvertising* advertising){
    advertising->addServiceUUID(SmartPaddleUUID::SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("Advertising started (SmartPaddleBLEServer::startAdvertising)");
}


// Метод для подключения к веслу
bool SmartPaddleBLEServer::connect() {
    // Логика подключения к веслу
    isConnected = true; // Установить в true, если подключение успешно
    is_pairing = false;
    BLEDevice::stopAdvertising();
    Serial.println("Connected");
    Serial.printf("Bonded devices count: %d\n", esp_ble_get_bond_device_num());
    send_specs=true;
    return isConnected;
}

// Мето для отключения от весла
void SmartPaddleBLEServer::disconnect() {
    // Логика отключения от весла
    if (isConnected){
        isConnected = false;
        pServer->disconnect(conn_id);
        delay(500);
        startAdvertising(BLEDevice::getAdvertising());
    }
}

void SmartPaddleBLEServer::setTrustedDevice(BLEAddress* address) {

    Preferences prefs;
    prefs.begin(prefsName.c_str(), false);
    prefs.putString("trustedDevice_string", address->toString().c_str());
    prefs.putBytes("trustedDevice", address->getNative(), sizeof(esp_bd_addr_t));
    prefs.end();
    
    if (trustedDevice) {
        delete trustedDevice;
    }
    trustedDevice = new BLEAddress(*address);
}

void SmartPaddleBLEServer::loadTrustedDevice() {
    if (trustedDevice) {
        delete trustedDevice;
        trustedDevice = nullptr;
    }

    Preferences prefs;
    prefs.begin(prefsName.c_str(), true);
    if (prefs.isKey("trustedDevice")) {
        Serial.println("Loading trusted device");
        esp_bd_addr_t address;
        prefs.getBytes("trustedDevice", &address, sizeof(esp_bd_addr_t));
        trustedDevice = new BLEAddress(address);
        Serial.printf("Trusted device: %s\n", trustedDevice->toString().c_str());
    } else {
        Serial.println("No trusted device found in EEPROM");
    }
    prefs.end();
}

void SmartPaddleBLEServer::clearTrustedDevice() {
    Preferences prefs;
    prefs.begin(prefsName.c_str(), false);
    prefs.remove("trustedDevice");
    prefs.remove("trustedDevice_string");
    prefs.end();
    
    if (trustedDevice) {
        delete trustedDevice;
        trustedDevice = nullptr;
    }
}

void SmartPaddleBLEServer::printBondedDevices() {
    int deviceCount = esp_ble_get_bond_device_num();
    Serial.printf("Bonded devices count: %d\n", deviceCount);
    
    if (deviceCount > 0) {
        esp_ble_bond_dev_t* deviceList = (esp_ble_bond_dev_t*)malloc(sizeof(esp_ble_bond_dev_t) * deviceCount);
        esp_ble_get_bond_device_list(&deviceCount, deviceList);
        
        for (int i = 0; i < deviceCount; i++) {
            BLEAddress address(deviceList[i].bd_addr);
            Serial.printf("Device %d: %s\n", i, address.toString().c_str());
        }
        
        free(deviceList);
    }
}

void SmartPaddleBLEServer::removeAllBondedDevices() {
    int deviceCount = esp_ble_get_bond_device_num();
    
    if (deviceCount > 0) {
        esp_ble_bond_dev_t* deviceList = (esp_ble_bond_dev_t*)malloc(sizeof(esp_ble_bond_dev_t) * deviceCount);
        esp_ble_get_bond_device_list(&deviceCount, deviceList);
        
        for (int i = 0; i < deviceCount; i++) {
            esp_ble_remove_bond_device(deviceList[i].bd_addr);
        }
        
        free(deviceList);
        Serial.println("All bonded devices removed");
    }
}

void SmartPaddleBLEServer::removeBondedDevice(BLEAddress address) {
    if (esp_ble_remove_bond_device(*address.getNative()) == ESP_OK) {
        Serial.printf("Device %s removed from bonding list\n", address.toString().c_str());
    } else {
        Serial.printf("Failed to remove device %s\n", address.toString().c_str());
    }
}

bool SmartPaddleBLEServer::isBonded(BLEAddress address) {
    int deviceCount = esp_ble_get_bond_device_num();
    bool found = false;
    
    if (deviceCount > 0) {
        esp_ble_bond_dev_t* deviceList = (esp_ble_bond_dev_t*)malloc(sizeof(esp_ble_bond_dev_t) * deviceCount);
        esp_ble_get_bond_device_list(&deviceCount, deviceList);
        
        for (int i = 0; i < deviceCount; i++) {
            BLEAddress bondedAddress(deviceList[i].bd_addr);
            if (address.equals(bondedAddress)) {
                found = true;
                break;
            }
        }
        
        free(deviceList);
    }
    
    return found;
}

void SmartPaddleBLEServer::startPairing(){
    if (connected()){
        disconnect();
    } else {
        BLEDevice::stopAdvertising();
    }
    delay(500);
    removeAllBondedDevices();
    is_pairing = true;
    startAdvertising(BLEDevice::getAdvertising());
}

void SmartPaddleBLEServer::updateBLE(){

    
    if(connected()) {
        loadData loadSensorData;
        IMUData imuDataStruct;
        PaddleStatus statusData;
        OrientationData orientationData;
        BladeData bladeData;

        // Получение данных из очередей
        if(loadsensorQueue.receive(loadSensorData,0)) {
            forceCharacteristic->setValue((uint8_t*)&loadSensorData, sizeof(loadData));
            forceCharacteristic->notify();
        }

        if(imuQueue.receive(imuDataStruct,0)) {
            imuCharacteristic->setValue((uint8_t*)&imuDataStruct, sizeof(IMUData));
            imuCharacteristic->notify();
        }

        if(statusQueue.receive(statusData,0)) {
            statusCharacteristic->setValue((uint8_t*)&statusData, sizeof(PaddleStatus));
            statusCharacteristic->notify();
        }

        if(orientationQueue.receive(orientationData,0)) {
            orientationCharacteristic->setValue((uint8_t*)&orientationData, sizeof(OrientationData));
            orientationCharacteristic->notify();
        }

        if(bladeQueue.receive(bladeData,0)) {
            bladeCharacteristic->setValue((uint8_t*)&bladeData, sizeof(BladeData));
            bladeCharacteristic->notify();
        }

        if(send_specs) {
            specsCharacteristic->setValue((uint8_t*)&specs, sizeof(PaddleSpecs));
            specsCharacteristic->notify();
            send_specs=false;
        }

        if (serial) serial->update();

    }
}