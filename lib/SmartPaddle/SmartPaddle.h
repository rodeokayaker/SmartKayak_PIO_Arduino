#ifndef SmartPaddle_h
#define SmartPaddle_h
#include <Arduino.h>
#include <MadgwickAHRS.h>
#include <BLEServer.h>
#include <BLEClient.h>
#include "SP_BLESerial.h"
#define SENSOR_QUEUE_SIZE 10


// UUID для BLE сервисов и характеристик
namespace SmartPaddleUUID {
    extern const char* SERVICE_UUID;
    extern const char* FORCE_L_UUID;
    extern const char* FORCE_R_UUID;
    extern const char* IMU_UUID;

}

namespace SmartPaddleConstants{
    const float CALIBRATION_FACTOR_L = 106.165;
    const float CALIBRATION_FACTOR_R = 98.777;
}

struct loadData {
    int32_t forceL;
    int32_t forceR;
    uint32_t timestamp;
};
// IMU data structure
struct IMUData {
    float ax, ay, az;    // Acceleration raw data
    float gx, gy, gz;      // Angular velocity raw data
    float mx, my, mz;         // Magnetic field raw data
    float q0, q1, q2, q3;    // Quaternion if available
    uint32_t timestamp;               // Timestamp in milliseconds
};

struct OrientationData {
    float q0, q1, q2, q3;           // Madgwick quaternion
    uint32_t timestamp;               // Timestamp in milliseconds
};

struct BladeData {
    uint8_t bladeSide;
    float quaternion[4];
    float force;
    uint32_t timestamp;
};

#define TWO_BLADES 1
#define ONE_BLADE 0

// Интерфейсы для датчиков
class ILoadCell {
public:
    virtual float getForce() = 0;
    virtual void calibrate() = 0;
    virtual bool isCalibrationValid() = 0;
    virtual bool begin() = 0;
    virtual ~ILoadCell() = default;
};

class IIMU {
public:
    virtual IMUData getData() = 0;
    virtual void getData(IMUData& data) = 0;
    virtual void calibrate() = 0;
    virtual bool isCalibrationValid() = 0;
    virtual bool begin() = 0;
    virtual void setLogStream(Stream* stream = nullptr) = 0;
    virtual ~IIMU() = default;
};


struct PaddleSpecs{
    uint32_t PaddleID;                // Paddle ID
    uint8_t bladeType;                // Blade type (TWO_BLADES or ONE_BLADE)
    float leftFeatheredQuaternion[4];     // Feathered orientation
    float rightFeatheredQuaternion[4];     // Feathered orientation
};

struct PaddleStatus{
    int8_t batteryLevel;
    int8_t temperature;
};

// Класс для очереди с перезаписью
template<typename T>
class OverwritingQueue {
private:
    QueueHandle_t queue;
    size_t maxSize;

public:
    OverwritingQueue(size_t size) : maxSize(size) {
        queue = xQueueCreate(size, sizeof(T));
    }

    void send(const T& data) {
        if (uxQueueSpacesAvailable(queue) == 0) {
            T dummy;
            xQueueReceive(queue, &dummy, 0);
        }
        xQueueSendToBack(queue, &data, 0);
    }

    bool receive(T& data, TickType_t timeout = portMAX_DELAY) {
        return xQueueReceive(queue, &data, timeout) == pdTRUE;
    }

    ~OverwritingQueue() {
        vQueueDelete(queue);
    }
};

class SP_BLESerial;

// Class for smart paddle unified interface
class SmartPaddle {
    protected:
    PaddleSpecs specs;
    uint32_t KayakID;                // Kayak ID
    PaddleStatus status;
    bool isConnected;                  // Paddle connection status
    const int trustedDeviceAddr;
    bool run_madgwick;
    SP_BLESerial* serial;
    BLESerialMessageHandler* messageHandler;
    public:
    SmartPaddle(int tdevAddr):
        specs(),
        KayakID(0),
        status(),
        isConnected(false),
        trustedDeviceAddr(tdevAddr),
        run_madgwick(false),
        serial(nullptr),
        messageHandler(nullptr){}
    virtual void begin(const char* deviceName)=0;
    virtual void setPaddleID(uint32_t id)=0;
    virtual void setFilterFrequency(uint32_t frequency)=0;
    virtual void setRunMadgwick(bool run){run_madgwick=run;}
    
    virtual void updateIMU()=0;
    virtual void updateLoads()=0;
    virtual void updateBLE()=0;
    virtual void updateMadgwick(IMUData* imuData)=0;


    virtual bool connect()=0;                   // Connect to paddle
    virtual void disconnect()=0;               // Disconnect paddle
    bool connected() {return isConnected;}
    virtual void startPairing()=0;
    virtual SP_BLESerial* getSerial() {return serial;}
    virtual void calibrateIMU()=0;
    virtual ~SmartPaddle();
 
};


class SmartPaddleBLEServer : public SmartPaddle {
    friend class SPBLEServerCallbacks;
    friend class SerialServer_MessageHandler;
private:
    float calibrationFactorL;        // Calibration factor for left blade
    float calibrationFactorR;        // Calibration factor for right blade

    IIMU* imu;
    ILoadCell* loads[2];

    uint16_t FilterFrequency;
    Madgwick filter;

    OverwritingQueue<loadData> loadsensorQueue;
    OverwritingQueue<OrientationData> orientationQueue;       
    OverwritingQueue<IMUData> imuQueue;
    OverwritingQueue<BladeData> bladeQueue;
    OverwritingQueue<PaddleStatus> statusQueue;

    bool send_specs;

    BLEServer* pServer;
    BLECharacteristic *forceCharacteristic;
    BLECharacteristic *imuCharacteristic;
    BLECharacteristic *statusCharacteristic;
    BLECharacteristic *specsCharacteristic;
    BLECharacteristic *orientationCharacteristic;
    BLECharacteristic *bladeCharacteristic;
    BLEAdvertising *pAdvertising;

    BLEAddress* trustedDevice;
    BLESecurity* pSecurity;
    bool is_pairing;
    uint16_t conn_id;

    void setTrustedDevice(BLEAddress* address);
    void loadTrustedDevice();
public:
    void clearTrustedDevice();
private:
    void printBondedDevices();
    void removeAllBondedDevices();
    void removeBondedDevice(BLEAddress address);
    bool isBonded(BLEAddress address);    
    bool connect();                   // Connect to paddle

    
public:

    SmartPaddleBLEServer(int tdevAddr,uint16_t filterFrequency=98);

    void begin(const char* deviceName);
    void setPaddleID(uint32_t id){specs.PaddleID=id;}
    void setPaddleType(uint8_t type){specs.bladeType=type;}    
    void setFilterFrequency(uint32_t frequency){FilterFrequency=frequency;}

    void setIMU(IIMU* imuSensor){imu=imuSensor;}
    void setLoads(ILoadCell* right, ILoadCell* left=0){loads[0]=right; loads[1]=left;}

    void updateIMU() override;
    void updateLoads() override;
    void updateBLE() override;

    void startAdvertising(BLEAdvertising* advertising);
    void disconnect();               // Disconnect paddle

    void startPairing();
    void updateMadgwick(IMUData* imuData) override;
    void sendSpecs(){send_specs=true;}
    bool isPairing(){return is_pairing;}
    BLEServer* getBLEServer() { return pServer; } // Для доступа к BLE серверу
    void calibrateIMU() override;
}; 

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

    
    
    // Очереди для хранения полученных данных

    //TODO: для подключения нескольких весел, с этим нужно будет поработать
    //Надо будет сделать очередь для каждого весла
    static OverwritingQueue<loadData> loadsensorQueue;
    static OverwritingQueue<OrientationData> orientationQueue;
    static OverwritingQueue<IMUData> imuQueue;
    static OverwritingQueue<BladeData> bladeQueue;
    static OverwritingQueue<PaddleStatus> statusQueue;
    
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
    SmartPaddleBLEClient(int tdevAddr);
    
    // Реализация интерфейса SmartPaddle
    void begin(const char* deviceName) override;
    void setPaddleID(uint32_t id) override { specs.PaddleID = id; }
    void setFilterFrequency(uint32_t frequency) override {} // Не используется на клиенте
    
    // Эти методы не используются на клиенте
    void updateIMU() override {}
    void updateLoads() override {}
    void updateBLE() override;
    void updateMadgwick(IMUData* imuData) override {}
    
    bool connect() override;
    void disconnect() override;
    void startPairing() override;
    
    // Дополнительные методы для получения данных
    bool getLoadData(loadData& data, TickType_t timeout = 0);
    bool getIMUData(IMUData& data, TickType_t timeout = 0);
    bool getOrientationData(OrientationData& data, TickType_t timeout = 0);
    bool getBladeData(BladeData& data, TickType_t timeout = 0);
    bool getStatusData(PaddleStatus& data, TickType_t timeout = 0);
    PaddleSpecs getSpecs();
    bool isPairing(){return is_pairing;}
    
    // Методы для работы со сканированием
    void startScan(uint32_t duration = 5);
    void stopScan();
    std::vector<BLEAdvertisedDevice> getScannedDevices();
    BLEClient* getBLEClient() { return pClient; } // Для доступа к BLE клиенту
    void calibrateIMU() override;
};

extern std::map<void*, SmartPaddle*> PaddleMap;
#endif

