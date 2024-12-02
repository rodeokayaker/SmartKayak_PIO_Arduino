#include "SmartPaddle.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <MadgwickAHRS.h>
#include <EEPROM.h>
#include <esp_gatts_api.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

namespace SmartPaddleUUID {
    const char* SERVICE_UUID = "4b2de81d-c131-4636-8d56-83b83758f7ca";
    const char* FORCE_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
    const char* IMU_UUID = "d2e5bfeb-d9f8-4b75-a295-d3f4032086ea";
    const char* STATUS_UUID = "667ACEB9-4E06-4325-B7FD-AE1FE82017D5";
    const char* SPECS_UUID = "8346E073-0CBB-4F34-B3B7-83203AC052DA";
    const char* ORIENTATION_UUID = "6EB39A41-1B23-4C63-92ED-B6236DE7E7A6";
    const char* BLADE_UUID = "C7D2019D-22C9-40C7-ABFB-28F570217153";
}

#define VALID_PAIRING_FLAG 0x42

// Определения для FreeRTOS
#define SENSOR_STACK_SIZE 4096
#define BLE_STACK_SIZE 4096
#define IMU_STACK_SIZE 4096

extern bool log_imu;
extern bool log_load;

void SmartPaddleBLEServer::updateIMU(IIMU* imu){
    IMUData imuData; 
    imu->getData(imuData);
 
    imuQueue.send(imuData); 

}

void SmartPaddleBLEServer::updateLoads(ILoadCell* right, ILoadCell* left){
    loadData data;
    data.forceR = (int32_t)right->getForce();
    if (left)
        data.forceL = (int32_t)left->getForce();
    else
        data.forceL = 0;
    data.timestamp=millis();
    loadsensorQueue.send(data);
}


// Конструктор класса SmartPaddle
SmartPaddleBLEServer::SmartPaddleBLEServer(int tdevAddr,uint16_t filterFrequency)
    : SmartPaddle(tdevAddr),
      calibrationFactorL(SmartPaddleConstants::CALIBRATION_FACTOR_L),
      calibrationFactorR(SmartPaddleConstants::CALIBRATION_FACTOR_R),
      orientationQueue(SENSOR_QUEUE_SIZE),
      imuQueue(SENSOR_QUEUE_SIZE),
      loadsensorQueue(SENSOR_QUEUE_SIZE),
      bladeQueue(SENSOR_QUEUE_SIZE),
      statusQueue(1),
      FilterFrequency(filterFrequency),
      trustedDevice(nullptr),
      send_specs(false),
      is_pairing(false),
      log_imu_level(0)
{
    // Дополнительная инициализация, если необходимо
}

// класс для подключения и отключения устройства
class SPBLEServerCallbacks: public BLEServerCallbacks {
    private:
    SmartPaddleBLEServer* server;
    
    public:
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
        BLEAddress clientAddress(param->connect.remote_bda);
        param->connect.conn_id;
        
        if (server->connected()){
            Serial.printf("Already connected to %s\n", server->trustedDevice->toString().c_str());
            pServer->disconnect(param->connect.conn_id);
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
        log_v("PassKeyRequest");
		return 123456;
	}

	void onPassKeyNotify(uint32_t pass_key){
        log_v("On passkey Notify number:%d", pass_key);
	}

	bool onSecurityRequest(){
	    log_v("On Security Request");
		return true;
	}

	void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl){
		log_v("Starting BLE work!");
		if(cmpl.success){
			uint16_t length;
			esp_ble_gap_get_whitelist_size(&length);
			log_v("size: %d", length);
		}
	}
};

void SmartPaddleBLEServer::begin(const char* deviceName) {

    filter.begin(FilterFrequency);
    loadTrustedDevice();
    BLEDevice::init(deviceName);
    BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
    BLEDevice::setSecurityCallbacks(new MySecurity());
    
    // Настройка безопасности с сохранением указателя
    pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
    pSecurity->setCapability(ESP_IO_CAP_NONE);
    pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    
    pServer = BLEDevice::createServer();

    pServer->setCallbacks(new SPBLEServerCallbacks(this));

    BLEService *pService = pServer->createService(SmartPaddleUUID::SERVICE_UUID);
    forceCharacteristic = pService->createCharacteristic(
        SmartPaddleUUID::FORCE_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    imuCharacteristic = pService->createCharacteristic(
        SmartPaddleUUID::IMU_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    statusCharacteristic = pService->createCharacteristic(
        SmartPaddleUUID::STATUS_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    specsCharacteristic = pService->createCharacteristic(
        SmartPaddleUUID::SPECS_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    orientationCharacteristic = pService->createCharacteristic(
        SmartPaddleUUID::ORIENTATION_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    bladeCharacteristic = pService->createCharacteristic(
        SmartPaddleUUID::BLADE_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pService->start();
    startAdvertising(BLEDevice::getAdvertising());
    log_v("Service started\n");
    log_v("Bonded devices count: %d\n", esp_ble_get_bond_device_num());
}

void SmartPaddleBLEServer::startAdvertising(BLEAdvertising* advertising){
    advertising->addServiceUUID(SmartPaddleUUID::SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();
}

// Метод для подключения к веслу
bool SmartPaddleBLEServer::connect() {
    // Логика подключения к веслу
    isConnected = true; // Установить в true, если подключение успешно
    is_pairing = false;
    BLEDevice::stopAdvertising();
    log_v("Connected\n");
    log_v("Bonded devices count: %d\n", esp_ble_get_bond_device_num());
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
    EEPROM.begin(512);
    EEPROM.put(trustedDeviceAddr+2, *address->getNative());
    EEPROM.put(trustedDeviceAddr, VALID_PAIRING_FLAG);
    EEPROM.commit();
    EEPROM.end();
    
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

    EEPROM.begin(512);
    if (EEPROM.readByte(trustedDeviceAddr) == VALID_PAIRING_FLAG) {
        esp_bd_addr_t address;
        EEPROM.get(trustedDeviceAddr+2, address);
        trustedDevice = new BLEAddress(address);
    }
    EEPROM.end();
}

void SmartPaddleBLEServer::clearTrustedDevice() {
    EEPROM.begin(512);
    EEPROM.put(trustedDeviceAddr, 0);  // Очищаем флаг
    EEPROM.commit();
    EEPROM.end();
    
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
    loadData loadSensorData;
    IMUData imuDataStruct;
    PaddleStatus statusData;
    OrientationData orientationData;
    BladeData bladeData;
    
    if(connected()) {
        // Получение данных из очередей
        if(loadsensorQueue.receive(loadSensorData)) {
            forceCharacteristic->setValue((uint8_t*)&loadSensorData, sizeof(loadData));
            forceCharacteristic->notify();
        }

        if(imuQueue.receive(imuDataStruct)) {
            imuCharacteristic->setValue((uint8_t*)&imuDataStruct, sizeof(IMUData));
            imuCharacteristic->notify();
        }

        if(statusQueue.receive(statusData)) {
            statusCharacteristic->setValue((uint8_t*)&statusData, sizeof(PaddleStatus));
            statusCharacteristic->notify();
        }

        if(orientationQueue.receive(orientationData)) {
            orientationCharacteristic->setValue((uint8_t*)&orientationData, sizeof(OrientationData));
            orientationCharacteristic->notify();
        }

        if(bladeQueue.receive(bladeData)) {
            bladeCharacteristic->setValue((uint8_t*)&bladeData, sizeof(BladeData));
            bladeCharacteristic->notify();
        }

        if(send_specs) {
            specsCharacteristic->setValue((uint8_t*)&specs, sizeof(PaddleSpecs));
            specsCharacteristic->notify();
            send_specs=false;
        }

    }

}

void SmartPaddleBLEServer::updateMadgwick(IMUData& imuData){
   OrientationData orientationData;
  filter.update(imuData.gx,imuData.gy,imuData.gz,imuData.ax,imuData.ay,imuData.az,imuData.mx,imuData.my,imuData.mz);
    orientationData.timestamp=imuData.timestamp;
    filter.getQuaternion(&orientationData.q0);
    if (log_imu_level>0)
    {
        float yaw, pitch, roll;
        yaw=filter.getYaw();
        pitch=filter.getPitch();
        roll=filter.getRoll();
        Serial.printf("Yaw: %f, Pitch: %f, Roll: %f\n", yaw, pitch, roll);
    }
    orientationQueue.send(orientationData);
}


/**BLE Client code
*
*
*    
*
*
**/

// Колбэк класс для клиентских подключений
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
        client->isConnected = false;
        
        // Если не в режиме сопряжения, пытаемся переподключиться
        if(!client->isPairing()) {
            client->startScan(5);
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

SmartPaddleBLEClient::SmartPaddleBLEClient(int tdevAddr)
    : SmartPaddle(tdevAddr),
      trustedDevice(nullptr),
      pClient(nullptr) {
}

void SmartPaddleBLEClient::begin(const char* deviceName) {
    BLEDevice::init(deviceName);
    
    // Настройка безопасности
    BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
    pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
    pSecurity->setCapability(ESP_IO_CAP_NONE);
    pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    
    // Загрузка сохраненного устройства
    loadTrustedDevice();

}

void SmartPaddleBLEClient::disconnect() {
    if(pClient) {
        pClient->disconnect();
        delete pClient;
        pClient = nullptr;
    }
    isConnected = false;
}


// Колбэки для получения данных
void SmartPaddleBLEClient::forceCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    loadData data;
    if(length == sizeof(loadData)) {
        memcpy(&data, pData, sizeof(loadData));
        // Отправка в очередь
        loadsensorQueue.send(data);
    }
}

void SmartPaddleBLEClient::imuCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    IMUData data;
    if(length == sizeof(IMUData)) {
        memcpy(&data, pData, sizeof(IMUData));
        imuQueue.send(data);
    }
}   

void SmartPaddleBLEClient::statusCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    PaddleStatus data;
    if(length == sizeof(PaddleStatus)) {
        memcpy(&data, pData, sizeof(PaddleStatus));
        statusQueue.send(data);
    }
}

void SmartPaddleBLEClient::orientationCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    OrientationData data;
    if(length == sizeof(OrientationData)) {
        memcpy(&data, pData, sizeof(OrientationData));
        orientationQueue.send(data);
    }
}

void SmartPaddleBLEClient::bladeCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    BladeData data;
    if(length == sizeof(BladeData)) {
        memcpy(&data, pData, sizeof(BladeData));
        bladeQueue.send(data);
    }
}

void SmartPaddleBLEClient::specsCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    if(length == sizeof(PaddleSpecs)) {
        memcpy(&clientSpecs, pData, sizeof(PaddleSpecs));
    }
}

// Методы для получения данных из очередей
bool SmartPaddleBLEClient::getLoadData(loadData& data) {
    return loadsensorQueue.receive(data, 0);
}

bool SmartPaddleBLEClient::getIMUData(IMUData& data) {
    return imuQueue.receive(data, 0);
}

bool SmartPaddleBLEClient::getStatusData(PaddleStatus& data) {
    return statusQueue.receive(data, 0);
}   

bool SmartPaddleBLEClient::getOrientationData(OrientationData& data) {
    return orientationQueue.receive(data, 0);
}      

bool SmartPaddleBLEClient::getBladeData(BladeData& data) {
    return bladeQueue.receive(data, 0);
}   

// ... (остальные методы получения данных) ...

void SmartPaddleBLEClient::loadTrustedDevice() {
    if (trustedDevice) {
        delete trustedDevice;
    }
    trustedDevice = nullptr;
    
    EEPROM.begin(512);
    if (EEPROM.read(trustedDeviceAddr) == VALID_PAIRING_FLAG) {
        esp_bd_addr_t address;
        EEPROM.get(trustedDeviceAddr + 2, address);
        trustedDevice = new BLEAddress(address);
        Serial.printf("Loaded trusted device: %s\n", trustedDevice->toString().c_str());
    }
    EEPROM.end();
}

void SmartPaddleBLEClient::saveTrustedDevice(BLEAddress* address) {
    EEPROM.begin(512);
    EEPROM.write(trustedDeviceAddr, VALID_PAIRING_FLAG);
    EEPROM.put(trustedDeviceAddr + 2, *address->getNative());
    EEPROM.commit();
    EEPROM.end();
    
    if (trustedDevice) {
        delete trustedDevice;
    }
    trustedDevice = new BLEAddress(*address);
    Serial.printf("Saved trusted device: %s\n", trustedDevice->toString().c_str());
}

void SmartPaddleBLEClient::clearTrustedDevice() {
    EEPROM.begin(512);
    EEPROM.write(trustedDeviceAddr, 0);
    EEPROM.commit();
    EEPROM.end();
    
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
            
            if(client->isPairing()) {
                // В режиме сопряжения сохраняем новое устройство
                BLEAddress* address = new BLEAddress(advertisedDevice.getAddress());
                client->saveTrustedDevice(address);
                delete address;
                client->is_pairing=false;
                // Останавливаем сканирование и подключаемся
                BLEDevice::getScan()->stop();
                client->connect();
            } 
            else if(client->trustedDevice && 
                   advertisedDevice.getAddress().equals(*client->trustedDevice)) {
                // Вне режима сопряжения подключаемся только к доверенному устройству
                BLEDevice::getScan()->stop();
                client->connect();
            }
        }
    }
};

void SmartPaddleBLEClient::startPairing() {
    disconnect();
    is_pairing=true;
//    clearTrustedDevice();
    
    // Запускаем сканирование в режиме сопряжения
    BLEScan* pScan = BLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new SPBLEClientScanCallbacks(this));
    pScan->setActiveScan(true);
    pScan->start(30); // Сканируем 30 секунд
    
    Serial.println("Started pairing mode");
}

void SmartPaddleBLEClient::startScan(uint32_t duration) {
    // Запускаем сканирование для поиска известного устройства
    is_pairing=false;
    BLEScan* pScan = BLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new SPBLEClientScanCallbacks(this));
    pScan->setActiveScan(true);
    pScan->start(duration);
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
    
    // Подключение к серверу
    if(!pClient->connect(*trustedDevice)) {
        Serial.println("Connection failed");
        delete pClient;
        pClient = nullptr;
        return false;
    }
    
    // Получение сервиса
    pRemoteService = pClient->getService(SmartPaddleUUID::SERVICE_UUID);
    if(!pRemoteService) {
        Serial.println("Failed to find service");
        pClient->disconnect();
        delete pClient;
        pClient = nullptr;
        return false;
    }
    
    // Получение характеристик и подписка на уведомления
    if(!setupCharacteristics()) {
        Serial.println("Failed to setup characteristics");
        pClient->disconnect();
        delete pClient;
        pClient = nullptr;
        return false;
    }
    
    isConnected = true;
    Serial.println("Connected successfully");
    return true;
}

void SmartPaddleBLEClient::updateBLE() {
    if(!isConnected && trustedDevice) {
        // Если есть доверенное устройство, но нет подключения,
        // запускаем сканирование для его поиска
//        startScan(5);
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
        return false;
    }
    
    // Подписка на уведомления
    forceChar->registerForNotify(forceCallback);
    imuChar->registerForNotify(imuCallback);
    statusChar->registerForNotify(statusCallback);
    specsChar->registerForNotify(specsCallback);
    orientationChar->registerForNotify(orientationCallback);
    bladeChar->registerForNotify(bladeCallback);
    
 /*   // Получение начальных спецификаций весла
    std::string value = specsChar->readValue();
    if(value.length() == sizeof(PaddleSpecs)) {
        memcpy(&specs, value.c_str(), sizeof(PaddleSpecs));
    }*/
    
    return true;
}

