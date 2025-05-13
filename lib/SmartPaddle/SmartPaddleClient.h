#ifndef SMARTPADDLE_BLE_CLIENT_H
#define SMARTPADDLE_BLE_CLIENT_H
#include <SmartPaddle.h>
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLESecurity.h>
#include <BLE2902.h>

namespace SPClient_Default_Frequencies {
    const uint16_t BLE_SEND_FREQUENCY = 10;
    const uint16_t BLE_RECEIVE_FREQUENCY = 10;
}


// Класс для работы со SmartPaddle на стороне клиента (каяка)
class SmartPaddleBLEClient : public SmartPaddle {
    friend class SPBLEClientCallbacks;
    friend class SPBLEClientScanCallbacks;
    friend class SPClientRTOS;
    friend class SPClient_MessageHandler;

private:
    BLEClient* pClient;
    BLERemoteService* pRemoteService;
    BLERemoteCharacteristic* forceChar;
    BLERemoteCharacteristic* imuChar;
    BLERemoteCharacteristic* orientationChar;
    BLERemoteCharacteristic* bladeChar;
    std::function<void(BLERemoteCharacteristic*, uint8_t*, size_t, bool)> imuNotifyCallback;
    std::function<void(BLERemoteCharacteristic*, uint8_t*, size_t, bool)> orientationNotifyCallback;
    std::function<void(BLERemoteCharacteristic*, uint8_t*, size_t, bool)> forceNotifyCallback;
    std::function<void(BLERemoteCharacteristic*, uint8_t*, size_t, bool)> bladeNotifyCallback;
    
    BLEAddress* trustedDevice;
    BLESecurity* pSecurity;

    bool is_pairing;
    bool do_connect;
    bool do_scan;
    bool do_disconnect;

    std::string prefsName;

    TaskHandle_t bleReceiveTaskHandle;
    TaskHandle_t bleSendTaskHandle;
    TaskHandle_t eventTaskHandle;
    uint16_t bleSendFrequency;
    uint16_t bleReceiveFrequency;

    OverwritingQueue<loadData> loadsensorQueue;
    OverwritingQueue<OrientationData> orientationQueue;
    OverwritingQueue<IMUData> imuQueue;

    uint32_t timeDifference;

    IMUData current_imu_data;
    loadData current_loads_data;
    OrientationData current_orientation_data;
    
    void forceCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify);
    void imuCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify);
    void orientationCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify);
    
    void loadTrustedDevice();
    void saveTrustedDevice(BLEAddress* address);
    void clearTrustedDevice();

    bool setupCharacteristics();
    
public:
    SmartPaddleBLEClient(const char* prefs_Name);
    
    // Реализация интерфейса SmartPaddle
    void begin(const char* deviceName) override;
    void setFilterFrequency(uint32_t frequency) override {} // Не используется на клиенте
    
    // Эти методы не используются на клиенте
    void updateBLE();
    
    bool connect() override;
    void disconnect() override;
    void startPairing() override;
    
    // Дополнительные методы для получения данных
    bool receiveLoadData(loadData& data, TickType_t timeout = 0);
    bool receiveIMUData(IMUData& data, TickType_t timeout = 0);
    bool receiveOrientationData(OrientationData& data, TickType_t timeout = 0);
    bool isPairing(){return is_pairing;}
    
    // Методы для работы со сканированием
    void startScan(uint32_t duration = 5);
    void stopScan();
    std::vector<BLEAdvertisedDevice> getScannedDevices();
    BLEClient* getBLEClient() { return pClient; } // Для доступа к BLE клиенту
    void calibrateIMU() override;
    void calibrateLoads(BladeSideType blade_side) override;
    void shutdown();

    IMUData getIMUData() override;
    loadData getLoadData() override;
    OrientationData getOrientationData() override;
    void calibrateBladeAngle(BladeSideType blade_side) override;
    uint32_t paddleMillis() override;
    uint32_t toLocalMillis(uint32_t timestamp) {return timestamp+timeDifference;}
};

#endif // SMARTPADDLE_BLE_CLIENT_H
