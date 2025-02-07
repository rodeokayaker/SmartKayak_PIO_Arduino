#ifndef SMARTPADDLE_BLE_SERVER_H
#define SMARTPADDLE_BLE_SERVER_H

#include <SmartPaddle.h>
#include <LogInterface.h>

namespace SPServer_Default_Frequencies {
    const uint16_t BLE_SEND_FREQUENCY = 10;
    const uint16_t BLE_RECEIVE_FREQUENCY = 10;
}


class SmartPaddleBLEServer : public SmartPaddle {
    friend class SPBLEServerCallbacks;
    friend class SerialServer_MessageHandler;
    friend class SPServerRTOS;
private:

//Sensors
    IIMU* imu;
    ILoadCell* loads[2];
    uint16_t FilterFrequency;

//Preferences
    std::string prefsName;
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
//    BLECharacteristic *bladeCharacteristic;
    BLEAdvertising *pAdvertising;
    BLEAddress* trustedDevice;
    BLESecurity* pSecurity;

//BLE STATUS
    bool is_pairing;
    bool do_connect;
    uint32_t time_to_connect;
    uint16_t conn_id;
    bool sendData;

//    ILogInterface* logInterface;

//RTOS
    TaskHandle_t bleSendTaskHandle;
    TaskHandle_t bleReceiveTaskHandle;
    TaskHandle_t loadCellTaskHandle;
    TaskHandle_t imuTaskHandle;
    TaskHandle_t orientationTaskHandle;

    uint16_t bleSendFrequency;
    uint16_t bleReceiveFrequency;
    uint16_t loadCellFrequency;
    uint16_t imuFrequency;
    uint16_t orientationFrequency;
    
    void setTrustedDevice(BLEAddress* address);
    void loadTrustedDevice();

private:
    void printBondedDevices();
    void removeAllBondedDevices();
    void removeBondedDevice(BLEAddress address);
    bool isBonded(BLEAddress address);    

    void startAdvertising(BLEAdvertising* advertising);
    bool connect();                   // Connect to paddle

    void updateIMU();
    void updateLoads();
    void updateBLE();    

    TimerHandle_t connectTimer;
    static const uint32_t CONNECT_DELAY_MS = 1000; // 1 секунда
    static void connectTimerCallback(TimerHandle_t timer);
    
public:

    SmartPaddleBLEServer(const char* prefs_Name);
    

    void begin(const char* deviceName);
    void setPaddleID(uint32_t id){specs.PaddleID=id;}
    void setPaddleType(PaddleType type){specs.paddleType=type;}    
    void setFilterFrequency(uint32_t frequency){FilterFrequency=frequency;}

    void setIMU(IIMU* imuSensor){imu=imuSensor;}
    void setLoads(ILoadCell* right, ILoadCell* left=0){loads[0]=right; loads[1]=left;}



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

    void SetYAxisDirection(signed char direction) {
        bladeOrientation.YAxisDirection = direction;
    };

    void clearTrustedDevice();



}; 

#endif // SMARTPADDLE_BLE_SERVER_H
