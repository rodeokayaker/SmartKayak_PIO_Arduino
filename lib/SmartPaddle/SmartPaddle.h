#ifndef SmartPaddle_h
#define SmartPaddle_h
#include <Arduino.h>
#include <MadgwickAHRS.h>

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

class IBleCommunication {
public:
    virtual void init(const char* name) = 0;
    virtual void sendData(const void* data, size_t length) = 0;
    virtual bool receiveData(void* data, size_t length) = 0;
    virtual bool isConnected() = 0;
    virtual ~IBleCommunication() = default;
};


struct PaddleSpecs{
    uint32_t PaddleID;                // Paddle ID
    uint8_t bladeType;                // Blade type (TWO_BLADES or ONE_BLADE)
    float featheredAngle;             // Feathered angle of the paddle
    float featheredQuaternion[4];     // Feathered orientation
};

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


// Class for smart paddle control
class SmartPaddle {
private:
    PaddleSpecs specs;               // Paddle specifications
    uint8_t batteryLevel;            // Battery level in percentage
    uint8_t bladeType;                // Blade type (TWO_BLADES or ONE_BLADE)
    float calibrationFactorL;           // Calibration factor for left blade
    float calibrationFactorR;           // Calibration factor for right blade
    bool isConnect;                  // Paddle connection status
    bool operateServer;                // Wether this instanse used for server or client
    uint16_t FilterFrequency;
    Madgwick filter;

    OverwritingQueue<loadData> loadsensorQueue;

    #ifdef CALCULATE_ON_SERVER
    OverwritingQueue<OrientationData> orientationQueue;   
    #else
    OverwritingQueue<IMUData> imuQueue;
    #endif

    IBleCommunication* bleComm;

    int8_t log_imu_level;

    public:

    void set_log_imu(int8_t level){
        log_imu_level = level;
    }

    int8_t get_log_imu(){
        return log_imu_level;
    }

    SmartPaddle(bool isServer=true, uint16_t filterFrequency=50);

    void begin();
    void beginClient();
    void beginServer(const char* deviceName);
    void setPaddleID(uint32_t id);
    void setFilterFrequency(uint32_t frequency){FilterFrequency=frequency;}

    void updateIMU(IIMU* imu);
    void updateLoads(ILoadCell* right, ILoadCell* left=0);

    bool connect();                   // Connect to paddle
    void disconnect();               // Disconnect paddle
    bool isConnected();                // Check if paddle is active
 
};

#endif

