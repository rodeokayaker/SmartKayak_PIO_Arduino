#ifndef SMARTPADDLE_BLE_SERVER_H
#define SMARTPADDLE_BLE_SERVER_H

#include <SmartPaddle.h>
#include <LogInterface.h>


class SmartPaddleBLEServer : public SmartPaddle {
    friend class SPBLEServerCallbacks;
    friend class SerialServer_MessageHandler;
private:
    IIMU* imu;
    ILoadCell* loads[2];

    uint16_t FilterFrequency;

    std::string prefsName;
    Stream* logStream;

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

    ILogInterface* logInterface;

    void setTrustedDevice(BLEAddress* address);
    void loadTrustedDevice();
    int power_pin;

public:
    void clearTrustedDevice();
private:
    void printBondedDevices();
    void removeAllBondedDevices();
    void removeBondedDevice(BLEAddress address);
    bool isBonded(BLEAddress address);    
    bool connect();                   // Connect to paddle

    
public:

    SmartPaddleBLEServer(const char* prefs_Name,uint16_t filterFrequency=98);

    void begin(const char* deviceName);
    void setPaddleID(uint32_t id){specs.PaddleID=id;}
    void setPaddleType(PaddleType type){specs.paddleType=type;}    
    void setFilterFrequency(uint32_t frequency){FilterFrequency=frequency;}

    void setIMU(IIMU* imuSensor){imu=imuSensor;}
    void setLoads(ILoadCell* right, ILoadCell* left=0){loads[0]=right; loads[1]=left;}

    void updateIMU() override;
    void updateLoads() override;
    void updateBLE() override;

    void startAdvertising(BLEAdvertising* advertising);
    void disconnect();               // Disconnect paddle

    void startPairing();
    void sendSpecs(){send_specs=true;}
    bool isPairing(){return is_pairing;}
    BLEServer* getBLEServer() { return pServer; } // Для доступа к BLE серверу
    void calibrateIMU() override;
    void calibrateLoads(BladeSideType blade_side) override;
    void setLogStream(Stream* stream=&Serial);
    void setLogInterface(ILogInterface* logInterface){this->logInterface=logInterface;}
    void shutdown();
    void setPowerPin(int pin){power_pin=pin;}

    IMUData getIMUData() override;
    loadData getLoadData() override;
    OrientationData getOrientationData() override;

    uint32_t paddleMillis() override {return ::millis();}


}; 

#endif // SMARTPADDLE_BLE_SERVER_H
