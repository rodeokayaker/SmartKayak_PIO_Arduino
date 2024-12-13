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
#include <SP_BLESerialClient.h>
#include <SP_BLESerialServer.h>

namespace SmartPaddleUUID {
    const char* SERVICE_UUID = "4b2de81d-c131-4636-8d56-83b83758f7ca";
    const char* FORCE_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
    const char* IMU_UUID = "d2e5bfeb-d9f8-4b75-a295-d3f4032086ea";
    const char* STATUS_UUID = "667ACEB9-4E06-4325-B7FD-AE1FE82017D5";
    const char* SPECS_UUID = "8346E073-0CBB-4F34-B3B7-83203AC052DA";
    const char* ORIENTATION_UUID = "6EB39A41-1B23-4C63-92ED-B6236DE7E7A6";
    const char* BLADE_UUID = "C7D2019D-22C9-40C7-ABFB-28F570217153";

/*    
    const char* NORDIC_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
    const char* NORDIC_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
    const char* NORDIC_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";
*/
}

std::map<void*, SmartPaddle*> PaddleMap = {} ;

#define VALID_PAIRING_FLAG 0x42

// Определения для FreeRTOS
#define SENSOR_STACK_SIZE 4096
#define BLE_STACK_SIZE 4096
#define IMU_STACK_SIZE 4096
#define MADGWICK_STACK_SIZE 4096

#define BLE_BUFFER_SIZE ESP_GATT_MAX_ATTR_LEN // must be greater than MTU, less than ESP_GATT_MAX_ATTR_LEN
#define RX_BUFFER_SIZE 4096


static int BLEMTU = max({sizeof(IMUData), sizeof(loadData), sizeof(OrientationData), sizeof(PaddleStatus), sizeof(PaddleSpecs), sizeof(BladeData)})+4;




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
            paddle->imu->setLogStream(paddle->getSerial());
            paddle->calibrateIMU();
            paddle->imu->setLogStream(nullptr);
        }
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

SmartPaddle::~SmartPaddle(){
    if(serial) {delete serial; serial=nullptr;}
    if(messageHandler) {delete messageHandler; messageHandler=nullptr;}
}

void SmartPaddleBLEServer::updateIMU(){
    IMUData imuData; 
    imu->getData(imuData);
    imuQueue.send(imuData); 
    if (run_madgwick) {
        updateMadgwick(&imuData);
    }
}

void SmartPaddleBLEServer::calibrateIMU(){
    imu->calibrate();
}

void SmartPaddleBLEServer::updateLoads(){
    loadData data;
    data.forceR = (int32_t)loads[0]->getForce();
    if (loads[1])
        data.forceL = (int32_t)loads[1]->getForce();
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
      imu(nullptr),
      loads{nullptr,nullptr}
{
    run_madgwick=true;
    serial=new SP_BLESerialServer(this);
    messageHandler=new SerialServer_MessageHandler(this);
    serial->setMessageHandler(messageHandler);    

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

    filter.begin(FilterFrequency);
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
    EEPROM.begin(512);
  //  esp_bd_addr_t address_native;
  //  (*(address->getNative()))[1];
    EEPROM.writeByte(trustedDeviceAddr, VALID_PAIRING_FLAG);
    EEPROM.put<esp_bd_addr_t>(trustedDeviceAddr+2, *(address->getNative()));
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
        Serial.println("Loading trusted device");
        esp_bd_addr_t address;
        EEPROM.get<esp_bd_addr_t>(trustedDeviceAddr+2, address);
        trustedDevice = new BLEAddress(address);
        Serial.printf("Trusted device: %s\n", trustedDevice->toString().c_str());
    } else {
        Serial.println("No trusted device found in EEPROM");
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

void SmartPaddleBLEServer::updateMadgwick(IMUData* imuData){
    OrientationData orientationData;
    filter.update(imuData->gx,imuData->gy,imuData->gz,imuData->ax,imuData->ay,imuData->az,imuData->mx,imuData->my,imuData->mz);
    orientationData.timestamp=imuData->timestamp;
    filter.getQuaternion(&orientationData.q0);
 /*   if (log_imu_level>0)
    {
        float yaw, pitch, roll;
        yaw=filter.getYaw();
        pitch=filter.getPitch();
        roll=filter.getRoll();
        Serial.printf("Yaw: %f, Pitch: %f, Roll: %f\n", yaw, pitch, roll);
    }*/
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

SmartPaddleBLEClient::SmartPaddleBLEClient(int tdevAddr)
    : SmartPaddle(tdevAddr),
      trustedDevice(nullptr),
      pClient(nullptr),
      do_scan(false),
      do_connect(false) {
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
        // Отправка в очередь
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
//        Serial.printf("IMU data match\n");
        memcpy(&data, pData, sizeof(IMUData));
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
    return imuQueue.receive(data, timeout);
}

bool SmartPaddleBLEClient::getStatusData(PaddleStatus& data, TickType_t timeout) {
    return statusQueue.receive(data, timeout);
}   

bool SmartPaddleBLEClient::getOrientationData(OrientationData& data, TickType_t timeout) {
    return orientationQueue.receive(data, timeout);
}      

bool SmartPaddleBLEClient::getBladeData(BladeData& data, TickType_t timeout) {
    return bladeQueue.receive(data, timeout);
}   


void SmartPaddleBLEClient::loadTrustedDevice() {
    if (trustedDevice) {
        delete trustedDevice;
    }
    trustedDevice = nullptr;
    
    EEPROM.begin(512);
    if (EEPROM.readByte(trustedDeviceAddr) == VALID_PAIRING_FLAG) {
        esp_bd_addr_t address;
        EEPROM.get<esp_bd_addr_t>(trustedDeviceAddr + 2, address);
        trustedDevice = new BLEAddress(address);
        Serial.printf("Loaded trusted device: %s\n", trustedDevice->toString().c_str());
    }
    EEPROM.end();
}

void SmartPaddleBLEClient::saveTrustedDevice(BLEAddress* address) {
    EEPROM.begin(512);
    EEPROM.writeByte(trustedDeviceAddr, VALID_PAIRING_FLAG);
    EEPROM.put<esp_bd_addr_t>(trustedDeviceAddr + 2, *address->getNative());
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
    EEPROM.writeByte(trustedDeviceAddr, 0);
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
    }

    if (serial) serial->update();
}

void SmartPaddleBLEClient::calibrateIMU() {
    if (serial) serial->sendCommand("calibrate_imu");
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
    
    // Подписка на уведомления
    forceChar->registerForNotify(forceCallback);
    imuChar->registerForNotify(imuCallback);
    statusChar->registerForNotify(statusCallback);
    specsChar->registerForNotify(specsCallback);
    orientationChar->registerForNotify(orientationCallback);
    bladeChar->registerForNotify(bladeCallback);
    Serial.println("Registered for notifications");
    
 /*   // Получение начальных спецификаций весла
    std::string value = specsChar->readValue();
    if(value.length() == sizeof(PaddleSpecs)) {
        memcpy(&specs, value.c_str(), sizeof(PaddleSpecs));
    }*/
    
    return true;
}


