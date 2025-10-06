#ifndef SMARTPADDLE_BLE_SERVER_H
#define SMARTPADDLE_BLE_SERVER_H

#include <SmartPaddle.h>
#include <LogInterface.h>
#include "BLEServer.h"
#include "OverwritingQueue.h"


#define HX711_SET_INTERRUPT
#define HX711_RESET_FREQUENCY 60000
#define RESET_UP_CHECK
#define HX711_USE_INTERRUPT

#define IMU_DEFAULT_FREQUENCY 100


namespace SPServer_Default_Frequencies {
    const uint16_t BLE_SEND_FREQUENCY = 10;
    const uint16_t BLE_RECEIVE_FREQUENCY = 10;
}


class SmartPaddleBLEServer : public SmartPaddle {
    friend class SPBLEServerCallbacks;
    friend class SPServer_MessageHandler;
    friend class SPServerRTOS;
private:

//Sensors
    ImuSensor* imu;
//    ILoadCell* loads[2];
    LoadCellSet* loads;
    loadData lastLoadData;

//Preferences
    std::string prefsName;

// Where should i write information    
    Stream* logStream;

//Queues
    OverwritingQueue<loadData> loadsensorQueue;
    OverwritingQueue<OrientationData> orientationQueue;       
    OverwritingQueue<IMUData> imuQueue;
    OverwritingQueue<BladeData> bladeQueue;

    bool send_specs;
    uint32_t time_to_send_specs;
    bool send_paddle_orientation;
    uint32_t time_to_send_paddle_orientation;

//BLE POINTERS
    BLEServer* pServer;
    BLECharacteristic *forceCharacteristic;
    BLECharacteristic *imuCharacteristic;
    BLECharacteristic *orientationCharacteristic;
    BLEAdvertising *pAdvertising;
    BLEAddress* trustedDevice;
    BLESecurity* pSecurity;

//BLE STATUS
    bool is_pairing;
    bool do_connect;
    uint32_t time_to_connect;
    uint16_t conn_id;
    bool sendData;

//RTOS
    TaskHandle_t bleSendTaskHandle;
    TaskHandle_t bleReceiveTaskHandle;
    TaskHandle_t loadCellTaskHandle;
    TaskHandle_t imuTaskHandle;
    TaskHandle_t orientationTaskHandle;
    TaskHandle_t magnetometerTaskHandle;

    uint16_t bleSendFrequency;           
    uint16_t bleReceiveFrequency;
    uint16_t loadCellFrequency;

    
    void setTrustedDevice(BLEAddress* address);
    void loadTrustedDevice();

    void printBondedDevices();
    void removeAllBondedDevices();
    void removeBondedDevice(BLEAddress address);
    bool isBonded(BLEAddress address);    

    bool connect();                   // Connect to paddle

    void startAdvertising(BLEAdvertising* advertising);

//    bool updateLoads();
    void updateBLE();    

    TimerHandle_t connectTimer;
    static const uint32_t CONNECT_DELAY_MS = 1000; // 1 секунда
    static void connectTimerCallback(TimerHandle_t timer);
    
public:

    SmartPaddleBLEServer(const char* prefs_Name);
    

    void begin(const char* deviceName);
    void setPaddleID(uint32_t id){specs.paddleID=id;}
    void setPaddleType(PaddleType type){specs.paddleType=type;}    

    void setIMU(ImuSensor* imuSensor){imu=imuSensor;}
//    void setLoads(ILoadCell* right, ILoadCell* left=0){loads[0]=right; loads[1]=left;}
    void setLoads(LoadCellSet* lcSet){loads=lcSet;}

    void disconnect() override;               // Disconnect paddle

    void startPairing();
    void sendSpecs(uint32_t delay_time = 0){time_to_send_specs=millis()+delay_time; send_specs=true;}
    void sendPaddleOrientation(uint32_t delay_time = 0){time_to_send_paddle_orientation=millis()+delay_time; send_paddle_orientation=true;}
    bool isPairing(){return is_pairing;}
    BLEServer* getBLEServer() { return pServer; } // Для доступа к BLE серверу
    void calibrateIMU() override;
    void calibrateLoads(BladeSideType blade_side) override;
    void setLogStream(Stream* stream=&Serial);
    void shutdown();

    IMUData getIMUData() override;
    loadData getLoadData() override;
    OrientationData getOrientationData() override;

    void calibrateBladeAngle(BladeSideType blade_side);
    bool loadBladeOrientation();

    uint32_t paddleMillis() override {return ::millis();}

    void SetYAxisDirection(signed char direction, bool save = true);

    void clearTrustedDevice();
    void saveSpecs();
    bool loadSpecs();
    void startTasks();



}; 

#endif // SMARTPADDLE_BLE_SERVER_H
