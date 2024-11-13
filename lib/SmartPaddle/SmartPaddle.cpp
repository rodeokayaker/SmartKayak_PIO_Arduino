#include "SmartPaddle.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <MadgwickAHRS.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace SmartPaddleUUID {
    const char* SERVICE_UUID = "4b2de81d-c131-4636-8d56-83b83758f7ca";
    const char* FORCE_L_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
    const char* FORCE_R_UUID = "cba1d466-344c-4be3-ab3f-189f80dd7518";
    const char* IMU_UUID = "d2e5bfeb-d9f8-4b75-a295-d3f4032086ea";

}



// Определения для FreeRTOS
#define SENSOR_STACK_SIZE 4096
#define BLE_STACK_SIZE 4096
#define IMU_STACK_SIZE 4096

extern bool log_imu;
extern bool log_load;


void SmartPaddle::updateIMU(IIMU* imu){
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

void SmartPaddle::updateLoads(ILoadCell* right, ILoadCell* left){
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
SmartPaddle::SmartPaddle(bool isServer, uint16_t filterFrequency)
    : batteryLevel(100),         // Инициализация уровня заряда батареи
      bladeType(TWO_BLADES),     // Инициализация типа лопасти
      calibrationFactorL(SmartPaddleConstants::CALIBRATION_FACTOR_L),
      calibrationFactorR(SmartPaddleConstants::CALIBRATION_FACTOR_R),
      isConnect(false),        // Изначально не подключен
      operateServer(isServer),   // Установка режима работы (сервер/клиент)
      #ifdef CALCULATE_ON_SERVER
      orientationQueue(SENSOR_QUEUE_SIZE),
      #else
      imuQueue(SENSOR_QUEUE_SIZE),
      #endif
      loadsensorQueue(SENSOR_QUEUE_SIZE),
      FilterFrequency(filterFrequency),
      log_imu_level(0)
{
    // Дополнительная инициализация, если необходимо
}

// класс для подключения и отключения устройства
class MyServerCallbacks: public BLEServerCallbacks {
    private:
    bool* deviceConnected;
    public:
    void onConnect(BLEServer* pServer) {
      Serial.println("Connected");
      *deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      Serial.println("Disconnected");
      *deviceConnected = false;
    }

    MyServerCallbacks(bool* deviceConnectedPTR) {
        deviceConnected = deviceConnectedPTR;
    }

};

void SmartPaddle::begin(){
    filter.begin(FilterFrequency);
    if (operateServer)
        beginServer("Smart Paddle v. 1.0");
    else
        beginClient();
}

// Метод для инициализации сервера BLE
void SmartPaddle::beginServer(const char* deviceName) {
/*    BLEDevice::init(deviceName);
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks(&isConnect));

    BLEService *pService = pServer->createService(SmartPaddleUUID::SERVICE_UUID);

    BLECharacteristic *forceL = pService->createCharacteristic(
        SmartPaddleUUID::FORCE_L_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    forceL->setValue("0");

    BLECharacteristic *forceR = pService->createCharacteristic(
        SmartPaddleUUID::FORCE_R_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    forceR->setValue("0");

    BLECharacteristic *imuData = pService->createCharacteristic(
        SmartPaddleUUID::IMU_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    imuData->setValue("0");

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SmartPaddleUUID::SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();*/
}

// Метод для инициализации клиента BLE
void SmartPaddle::beginClient() {
//    BLEDevice::init("SmartPaddleClient");
    // Настройка клиента BLE
}

// Метод для подключения к веслу
bool SmartPaddle::connect() {
    // Логика подключения к веслу
    isConnect = true; // Установить в true, если подключение успешно
    return isConnect;
}

// Метод для отключения от весла
void SmartPaddle::disconnect() {
    // Логика отключения от весла
    isConnect = false;
}

// Метод для проверки подключения
bool SmartPaddle::isConnected() {
    return isConnect;
}





