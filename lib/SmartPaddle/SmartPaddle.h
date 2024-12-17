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
    extern const char* FORCE_UUID;
    extern const char* IMU_UUID;
    extern const char* STATUS_UUID;
    extern const char* SPECS_UUID;
    extern const char* ORIENTATION_UUID;
    extern const char* BLADE_UUID;    
}

enum PaddleType{
    ONE_BLADE,
    TWO_BLADES
};

enum BladeSideType{
    RIGHT_BLADE,
    LEFT_BLADE,
    ALL_BLADES
};

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
    BladeSideType bladeSide;
    float quaternion[4];
    float force;
    uint32_t timestamp;
};


// Интерфейсы для датчиков
class ILoadCell {
public:
    virtual float getForce() = 0;
    virtual void calibrate() = 0;
    virtual bool isCalibrationValid() = 0;
    virtual bool begin() = 0;
    virtual void setLogStream(Stream* stream = &Serial) = 0;
    virtual ~ILoadCell() = default; // Деструктор по умолчанию
};

class IIMU {
public:
    virtual bool begin() = 0;
    virtual IMUData getData() = 0;
    virtual OrientationData getOrientation() = 0;
    virtual void getData(IMUData& data) = 0;
    virtual void calibrate() = 0;
    virtual bool isCalibrationValid() = 0;
    virtual void setLogStream(Stream* stream = &Serial) = 0;
    virtual ~IIMU() = default;
};


struct PaddleSpecs{
    uint32_t PaddleID;                // Paddle ID
    PaddleType paddleType;                // Blade type (TWO_BLADES or ONE_BLADE)
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
    bool run_madgwick;
    SP_BLESerial* serial;
    BLESerialMessageHandler* messageHandler;
    
    public:
    SmartPaddle():
        specs(),
        KayakID(0),
        status(),
        isConnected(false),
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


    virtual bool connect()=0;                   // Connect to paddle
    virtual void disconnect()=0;               // Disconnect paddle
    bool connected() {return isConnected;}
    virtual void startPairing()=0;
    virtual SP_BLESerial* getSerial() {return serial;}
    virtual void calibrateIMU()=0;
    virtual void calibrateLoads(BladeSideType blade_side)=0;
    virtual ~SmartPaddle();
 
};

extern std::map<void*, SmartPaddle*> PaddleMap;
extern int BLEMTU;

#endif

