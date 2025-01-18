#ifndef SMARTPADDLE_BLE_CLIENT_H
#define SMARTPADDLE_BLE_CLIENT_H
#include <SmartPaddle.h>
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLESecurity.h>
#include <BLE2902.h>



// Класс для работы со SmartPaddle на стороне клиента (каяка)
class SmartPaddleBLEClient : public SmartPaddle {
    friend class SPBLEClientCallbacks;
    friend class SPBLEClientScanCallbacks;

private:
    BLEClient* pClient;
    BLERemoteService* pRemoteService;
    BLERemoteCharacteristic* forceChar;
    BLERemoteCharacteristic* imuChar;
    BLERemoteCharacteristic* statusChar;
    BLERemoteCharacteristic* specsChar;
    BLERemoteCharacteristic* orientationChar;
    BLERemoteCharacteristic* bladeChar;
    
    BLEAddress* trustedDevice;
    BLESecurity* pSecurity;

    bool is_pairing;
    bool do_connect;
    bool do_scan;

    std::string prefsName;
    
    // Очереди для хранения полученных данных

    //TODO: для подключения нескольких весел, с этим нужно будет поработать
    //Надо будет сделать очередь для каждого весла
    static OverwritingQueue<loadData> loadsensorQueue;
    static OverwritingQueue<OrientationData> orientationQueue;
    static OverwritingQueue<IMUData> imuQueue;
    static OverwritingQueue<BladeData> bladeQueue;
    static OverwritingQueue<PaddleStatus> statusQueue;

    uint32_t last_orientation_ts;
    uint32_t last_blade_ts;
    uint32_t last_imu_ts;
    uint32_t last_loads_ts;

    IMUData current_imu_data;
    loadData current_loads_data;
    OrientationData current_orientation_data;
    BladeData current_blade_data;
    
    // Колбэки для получения нотификаций
    //TODO: для подключения нескольких весел, с этим нужно будет поработать
    //Надо будет сделать отдельные колбэки для каждого весла
    static void forceCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify);
    static void imuCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify);
    static void orientationCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify);
    static void bladeCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify);
    static void statusCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify);
    static void specsCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify);
    
    void loadTrustedDevice();
    void saveTrustedDevice(BLEAddress* address);
    void clearTrustedDevice();

    bool setupCharacteristics();
    
public:
    SmartPaddleBLEClient(const char* prefs_Name);
    
    // Реализация интерфейса SmartPaddle
    void begin(const char* deviceName) override;
    void setPaddleID(uint32_t id) override { specs.PaddleID = id; }
    void setFilterFrequency(uint32_t frequency) override {} // Не используется на клиенте
    
    // Эти методы не используются на клиенте
    void updateIMU() override;
    void updateLoads() override;
    void updateBLE() override;
    
    bool connect() override;
    void disconnect() override;
    void startPairing() override;
    
    // Дополнительные методы для получения данных
    bool receiveLoadData(loadData& data, TickType_t timeout = 0);
    bool receiveIMUData(IMUData& data, TickType_t timeout = 0);
    bool receiveOrientationData(OrientationData& data, TickType_t timeout = 0);
    bool receiveBladeData(BladeData& data, TickType_t timeout = 0);
    bool receiveStatusData(PaddleStatus& data, TickType_t timeout = 0);
    PaddleSpecs getSpecs();
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

    uint32_t paddleMillis() override;
};

#endif // SMARTPADDLE_BLE_CLIENT_H
