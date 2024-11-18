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

namespace SmartPaddleUUID {
    const char* SERVICE_UUID = "4b2de81d-c131-4636-8d56-83b83758f7ca";
    const char* FORCE_L_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
    const char* FORCE_R_UUID = "cba1d466-344c-4be3-ab3f-189f80dd7518";
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
    OrientationData orientationData;
    #ifdef CALCULATE_ON_SERVER
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
    #else
        imuQueue.send(imuData); 
    #endif
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
        
        if (!server->devicePaired) {
            // Если нет сохраненного устройства, сохраняем первое подключившееся
            server->setTrustedDevice(new BLEAddress(clientAddress));
            Serial.printf("New device paired: %s\n", clientAddress.toString().c_str());
            server->connect();
        } 
        else if (server->trustedDevice->equals(clientAddress)) {
            // Если адрес совпадает с сохраненным - разрешаем подключение
            Serial.printf("Trusted device connected: %s\n", clientAddress.toString().c_str());
            server->connect();
        } 
        else {
            // Если адрес не совпадает - отключаем
            Serial.printf("Untrusted device rejected: %s\n", clientAddress.toString().c_str());
            pServer->disconnect(param->connect.conn_handle);
        }
    }

    void onDisconnect(BLEServer* pServer) {
        server->disconnect();
    }

    SPBLEServerCallbacks(SmartPaddleBLEServer* serverPTR) {
        server = serverPTR;
    }

};

void SmartPaddleBLEServer::begin(const char* deviceName){
    filter.begin(FilterFrequency);

    BLEDevice::init(deviceName);
    
    // Настройка безопасности
    BLESecurity *pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
    pSecurity->setCapability(ESP_IO_CAP_NONE);
    pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    
    *pServer = *BLEDevice::createServer();
    pServer->setCallbacks(new SPBLEServerCallbacks(this));
    BLEService *pService = pServer->createService(SmartPaddleUUID::SERVICE_UUID);

    forceLCharacteristic = pService->createCharacteristic(
        SmartPaddleUUID::FORCE_L_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );

    forceRCharacteristic = pService->createCharacteristic(
        SmartPaddleUUID::FORCE_R_UUID,
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
}

void SmartPaddleBLEServer::startAdvertising(BLEAdvertising* advertising){
    advertising->addServiceUUID(SmartPaddleUUID::SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
}

// Метод для подключения к веслу
bool SmartPaddleBLEServer::connect() {
    // Логика подключения к веслу
    isConnect = true; // Установить в true, если подключение успешно
    
    return isConnect;
}

// Метод для отключения от весла
void SmartPaddleBLEServer::disconnect() {
    // Логика отключения от весла
    isConnect = false;
    delay(500);
    startAdvertising(BLEDevice::getAdvertising());
}

void SmartPaddleBLEServer::setTrustedDevice(BLEAddress* address) {

    EEPROM.put(trustedDeviceAddr+2, address->getNative());
    EEPROM.put(trustedDeviceAddr, VALID_PAIRING_FLAG);
    EEPROM.commit();
    trustedDevice = address;
    devicePaired = true;
}

// В конструкторе или begin загружаем сохраненный адрес
void SmartPaddleBLEServer::loadTrustedDevice() {
    
    if (EEPROM.readByte(trustedDeviceAddr) == VALID_PAIRING_FLAG) {
        esp_bd_addr_t address;
        EEPROM.get<esp_bd_addr_t>(trustedDeviceAddr+2, address);
        trustedDevice = new BLEAddress(address);
        devicePaired = true;
    } else {
        trustedDevice = nullptr;
        devicePaired = false;
    }
}

void SmartPaddleBLEServer::clearTrustedDevice() {

    trustedDevice = nullptr;
    devicePaired = false;
}





