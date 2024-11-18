#ifndef SmartPaddle_h
#define SmartPaddle_h
#include <Arduino.h>
#include <MadgwickAHRS.h>
#include <BLEServer.h>

#define SENSOR_QUEUE_SIZE 10
#define CALCULATE_ON_SERVER



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


// Class for smart paddle unified interface
class SmartPaddle {
    protected:
    PaddleSpecs specs;
    uint32_t KayakID;                // Kayak ID
    PaddleStatus status;
    bool isConnected;                  // Paddle connection status
    const int trustedDeviceAddr;

    public:
    SmartPaddle(int tdevAddr):specs(),KayakID(0),status(),isConnected(false),trustedDeviceAddr(tdevAddr){}
    virtual void begin(const char* deviceName)=0;
    virtual void setPaddleID(uint32_t id)=0;
    virtual void setFilterFrequency(uint32_t frequency)=0;

    
    virtual void updateIMU(IIMU* imu)=0;
    virtual void updateLoads(ILoadCell* right, ILoadCell* left=0)=0;

    virtual bool connect()=0;                   // Connect to paddle
    virtual void disconnect()=0;               // Disconnect paddle
    bool connected() {return isConnected;}
 
};


class SmartPaddleBLEServer : public SmartPaddle {
    friend class SPBLEServerCallbacks;
private:
    float calibrationFactorL;        // Calibration factor for left blade
    float calibrationFactorR;        // Calibration factor for right blade
    bool isConnect;                  // Paddle connection status
    uint16_t FilterFrequency;
    Madgwick filter;

    OverwritingQueue<loadData> loadsensorQueue;
    OverwritingQueue<OrientationData> orientationQueue;       
    OverwritingQueue<IMUData> imuQueue;
    OverwritingQueue<BladeData> bladeQueue;
    OverwritingQueue<PaddleStatus> statusQueue;

    int8_t log_imu_level;

    BLEServer* pServer;
    BLECharacteristic *forceLCharacteristic;
    BLECharacteristic *forceRCharacteristic;
    BLECharacteristic *imuCharacteristic;
    BLECharacteristic *statusCharacteristic;
    BLECharacteristic *specsCharacteristic;
    BLECharacteristic *orientationCharacteristic;
    BLECharacteristic *bladeCharacteristic;
    BLEAdvertising *pAdvertising;

    BLEAddress* trustedDevice;
    bool devicePaired;
    
    
public:
    void set_log_imu(int8_t level){
        log_imu_level = level;
    }

    int8_t get_log_imu(){
        return log_imu_level;
    }

    SmartPaddleBLEServer(int tdevAddr,uint16_t filterFrequency=50);

    void begin(const char* deviceName);
    void setPaddleID(uint32_t id){specs.PaddleID=id;}
    void setPaddleType(uint8_t type){specs.bladeType=type;}    
    void setFilterFrequency(uint32_t frequency){FilterFrequency=frequency;}

    void updateIMU(IIMU* imu);
    void updateLoads(ILoadCell* right, ILoadCell* left=0);
    void updateBLE();

    void startAdvertising(BLEAdvertising* advertising);

    bool connect();                   // Connect to paddle
    void disconnect();               // Disconnect paddle

    void setTrustedDevice(BLEAddress* address);
    void loadTrustedDevice();
    void clearTrustedDevice();
}; 

#endif

