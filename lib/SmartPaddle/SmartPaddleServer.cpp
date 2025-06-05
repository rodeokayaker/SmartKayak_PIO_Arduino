#include <SmartPaddleServer.h>
#include <Preferences.h>
#include <SP_BLESerialServer.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <MadgwickAHRS.h>
#include <esp_gatts_api.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <SP_Quaternion.h>

// FreeRTOS определения
#define SENSOR_STACK_SIZE 4096
#define IMU_STACK_SIZE 4096
#define ORIENTATION_STACK_SIZE 4096
#define MAGNETOMETER_STACK_SIZE 4096
#define BLE_STACK_SIZE 4096
#define BLE_RECEIVE_STACK_SIZE 8192

extern bool log_imu;
extern bool log_load;


class SPServer_MessageHandler: public SP_MessageHandler{
    private:
    SmartPaddleBLEServer* paddle;

    public:
    SPServer_MessageHandler(SmartPaddleBLEServer* paddle):paddle(paddle){}

    void onLog(SP_LogMessage* log) override{
        Serial.printf("Kayak log: %s\n", log->message.c_str());
    }

    void onCalibrateIMUCommand(SP_Command* command) override{
        Serial.printf("Calibrate IMU command\n");
        paddle->sendData=false;
        paddle->setLogStream(paddle->getSerial());
        paddle->calibrateIMU();
        paddle->setLogStream(&Serial);
        paddle->getSerial()->sendString(SP_MessageProcessor::createResponseMessage(command->command.c_str(), true, "IMU calibrated"));
        paddle->sendData=true;
    }

    void onCalibrateLoadsCommand(SP_Command* command, BladeSideType bladeSide) override{
        Serial.printf("Calibrate loads command\n");
        paddle->sendData=false;
        paddle->setLogStream(paddle->getSerial());
        paddle->calibrateLoads(bladeSide);
        paddle->setLogStream(&Serial);
        paddle->getSerial()->sendString(SP_MessageProcessor::createResponseMessage(command->command.c_str(), true, "Loads calibrated"));
        paddle->sendData=true;
    }

    void onTareLoadsCommand(SP_Command* command, BladeSideType bladeSide) override{
        Serial.printf("Tare loads command\n");
        paddle->sendData=false;
        paddle->setLogStream(paddle->getSerial());   
        if (bladeSide == LEFT_BLADE){
            paddle->loads[1]->tare();
        } else if (bladeSide == RIGHT_BLADE){
            paddle->loads[0]->tare();
        } else if (bladeSide == ALL_BLADES){
            paddle->loads[0]->tare();
            paddle->loads[1]->tare();
        }
        paddle->setLogStream(&Serial);
        paddle->getSerial()->sendString(SP_MessageProcessor::createResponseMessage(command->command.c_str(), true, "Loads tared"));
        paddle->sendData=true;
    }

    void onCalibrateBladeAngleCommand(SP_Command* command, BladeSideType bladeSide) override{
        Serial.printf("Calibrate blade angle command\n");
        paddle->setLogStream(paddle->getSerial());
        paddle->calibrateBladeAngle(bladeSide);
        paddle->setLogStream(&Serial);
        paddle->getSerial()->sendString(SP_MessageProcessor::createResponseMessage(command->command.c_str(), true, "Blade angle calibrated"));
    }

    void onCalibrateCompassCommand(SP_Command* command) override{
        Serial.printf("Calibrate compass command\n");
        paddle->setLogStream(paddle->getSerial());
        paddle->sendData=false;
        paddle->imu->calibrateCompass();
        paddle->sendData=true;
        paddle->setLogStream(&Serial);
        paddle->getSerial()->sendString(SP_MessageProcessor::createResponseMessage(command->command.c_str(), true, "Compass calibrated"));
    }

    void onSendSpecsCommand(SP_Command* command) override{
        Serial.printf("Send specs command\n");
        paddle->sendSpecs();
        paddle->getSerial()->sendString(SP_MessageProcessor::createResponseMessage(command->command.c_str(), true, ""));
    }

    void onStartPairCommand(SP_Command* command) override{
        Serial.printf("Start pair command\n");
        paddle->startPairing();
    }

    void onShutdownCommand(SP_Command* command) override{
        Serial.printf("Shutdown command\n");
        paddle->shutdown();
    }

    void onCommand(SP_Command* command) override{
        Serial.printf("Paddle got unknown command: %s\n", command->command.c_str());
        paddle->getSerial()->sendString(SP_MessageProcessor::createResponseMessage(command->command.c_str(), false, "Unknown command"));
    }

    void onResponse(SP_Response* response) override{
        Serial.printf("Kayak response: %s\n", response->command.c_str());
    }
    void onData(SP_Data* data) override{
        Serial.printf("Kayak data: %s\n", data->dataType.c_str());
    }
    void onStatus(SP_StatusMessage* status) override{
        Serial.printf("Got Kayak status\n");
    }

    void onSetMagnetometerCalibrationCommand(SP_Command* command, float* offset, float* softIron) override{
        Serial.printf("Set magnetometer calibration command\n");
        IMUCalibData calibData=paddle->imu->getCalibrationData();
        calibData.magOffset[0]=offset[0];
        calibData.magOffset[1]=offset[1];
        calibData.magOffset[2]=offset[2];
        calibData.magScale[0]=softIron[0];
        calibData.magScale[1]=softIron[1];
        calibData.magScale[2]=softIron[2];
        calibData.magSI[0]=softIron[3];
        calibData.magSI[1]=softIron[4];
        calibData.magSI[2]=softIron[5];
//        Serial.printf("Magnetometer calibration data: %f, %f, %f\n", calibData.magOffset[0], calibData.magOffset[1], calibData.magOffset[2]);
//        Serial.printf("Magnetometer soft iron data: %f, %f, %f\n", calibData.magScale[0], calibData.magScale[1], calibData.magScale[2]);
//        Serial.printf("Magnetometer soft iron data: %f, %f, %f\n", calibData.magSI[0], calibData.magSI[1], calibData.magSI[2]);
        paddle->imu->setCalibrationData(calibData,true);
        
    }

    void onSpecsData(SP_Data* data, const PaddleSpecs& specs) override{
        Serial.printf("Got Specs data\n");
        String padddleID=paddle->specs.paddleID;
        paddle->specs=specs;
        paddle->specs.paddleID=padddleID;
        paddle->saveSpecs();
    }

    void onSendCalibrationDataCommand(SP_Command* command) override{
        Serial.printf("Send calibration data command\n");
        paddle->getSerial()->sendString(SP_MessageProcessor::createMagnetometerCalibrationDataMessage(paddle->imu->getCalibrationData()));
    }
};


bool SmartPaddleBLEServer::updateIMU() {
    bool updated = false;
    if(imu) {
        // Получаем данные IMU только если он доступен
        IMUData imuData = imu->readData();
        if (imu->DMPValid()){
            imuQueue.send(imuData);
            updated = true;
            // Получаем данные ориентации только если IMU доступен
            OrientationData orientation = imu->getOrientation();
            orientationQueue.send(orientation);
        }
    }
    // Если IMU недоступен, не отправляем никаких данных в очереди
    return updated;
}

void interruptHandler(){
    Serial.printf("IMU interrupt\n");
}

class SPServerRTOS{
    private:
    static SmartPaddleBLEServer* mainPaddle;

    static void imuTask(void *pvParameters) {        
        SmartPaddleBLEServer* paddle = (SmartPaddleBLEServer*)pvParameters;
        if (!paddle->imu){
            vTaskDelete(NULL);
            return;
        }
        TickType_t xLastWakeTime = xTaskGetTickCount();
        uint32_t frequency = paddle->imuFrequency;
        TickType_t xFrequency = (frequency>0)?pdMS_TO_TICKS(1000/frequency):0;    
        while(1) {
            if (xFrequency==0){
               // Ждем уведомления от прерывания
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            }

            if (paddle->connected()) {
                if (paddle->sendData) {
                    if (paddle->updateIMU()){
                        // Разбудить BLESend задачу
                        xTaskNotifyGive(paddle->bleSendTaskHandle);
                    }
                }
            }

            if (xFrequency>0)
                xTaskDelayUntil(&xLastWakeTime, xFrequency);

        }
    }

// Задача чтения тензодатчиков
    static void loadCellTask(void *pvParameters) {

        SmartPaddleBLEServer* paddle = (SmartPaddleBLEServer*)pvParameters;
        TickType_t xLastWakeTime = xTaskGetTickCount();
        if (!paddle->loads[0]){
            vTaskDelete(NULL);
            return;
        }
        uint32_t frequency = paddle->loadCellFrequency;
        const TickType_t xFrequency = (frequency>0)?pdMS_TO_TICKS(1000/frequency):0;  
        uint32_t last_update = millis();      
        while(1) {
            if (paddle->connected()) {
                if (paddle->sendData){ 
                    if (paddle->updateLoads()){ 
                        last_update = millis();
                        // Разбудить BLESend задачу
                        xTaskNotifyGive(paddle->bleSendTaskHandle);
                    }
                }
            }
            if (xFrequency>0){
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
            }
            else{
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            }
        }
    }  

    static void orientationTask(void *pvParameters) {
        SmartPaddleBLEServer* paddle = (SmartPaddleBLEServer*)pvParameters;
        TickType_t xLastWakeTime = xTaskGetTickCount();
        uint32_t frequency = paddle->imu->getFrequency();
        if (frequency == 0){
            vTaskDelete(NULL);
            return;
        }
        const TickType_t xFrequency = pdMS_TO_TICKS(1000/frequency);
        while(1) {
            uint32_t start = millis();
            if (paddle->connected() && paddle->sendData) {
                paddle->imu->updateOrientation();
            }
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    static void magnetometerTask(void *pvParameters) {
        SmartPaddleBLEServer* paddle = (SmartPaddleBLEServer*)pvParameters;
        TickType_t xLastWakeTime = xTaskGetTickCount();
        if (!paddle->imu){
            vTaskDelete(NULL);
            return;
        }
        uint32_t frequency = paddle->imu->magnetometerFrequency();
        if (frequency == 0){
            vTaskDelete(NULL);
            return;
        }
        const TickType_t xFrequency = pdMS_TO_TICKS(1000/frequency);
        while(1) {
            uint32_t start = millis();
            paddle->imu->magnetometerUpdate();

//            Serial.printf("Magnetometer Task, Time: %d\n", millis()-start);
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    // Задача обработки BLE

    static void bleSendTask(void *pvParameters) {
        SmartPaddleBLEServer* paddle = (SmartPaddleBLEServer*)pvParameters;
        TickType_t xLastWakeTime = xTaskGetTickCount();
        uint32_t frequency = paddle->bleSendFrequency;
        if (frequency == 0){
            vTaskDelete(NULL);
            return;
        }
        const TickType_t xFrequency = pdMS_TO_TICKS(1000/frequency); 
        while(1) {
            //Waiting for wakeup
            ulTaskNotifyTake(pdTRUE, xFrequency);
            paddle->updateBLE();
        }
    }

    static void bleReceiveTask(void *pvParameters) {
        SmartPaddleBLEServer* paddle = (SmartPaddleBLEServer*)pvParameters;
        TickType_t xLastWakeTime = xTaskGetTickCount();
        uint32_t frequency = paddle->bleReceiveFrequency;
        if (frequency == 0){
            vTaskDelete(NULL);
            return;
        }
        const TickType_t xFrequency = pdMS_TO_TICKS(1000/frequency); 
        
        while(1) {
            if(paddle->getSerial()) 
                paddle->getSerial()->updateJSON();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }


    static void IRAM_ATTR dmpDataReady() {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(mainPaddle->imuTaskHandle, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }

    static void IRAM_ATTR loadCellDataReady() {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(mainPaddle->loadCellTaskHandle, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }

    public:
    //All tasks setup and start
    static void startTasks(SmartPaddleBLEServer* paddle) {
        mainPaddle = paddle;
        xTaskCreatePinnedToCore(
            SPServerRTOS::loadCellTask,
            "LoadCell",
            SENSOR_STACK_SIZE,
            paddle,
            1,
            &paddle->loadCellTaskHandle,
            0
        );
    
        xTaskCreatePinnedToCore(
            SPServerRTOS::imuTask,
            "IMU",
            IMU_STACK_SIZE,
            paddle,
            1,
            &paddle->imuTaskHandle,
            0
        );

        xTaskCreatePinnedToCore(
            SPServerRTOS::orientationTask,
            "Orientation",
            ORIENTATION_STACK_SIZE,
            paddle,
            1,
            &paddle->orientationTaskHandle,
            0
        );

        xTaskCreatePinnedToCore(
            SPServerRTOS::magnetometerTask,
            "Magnetometer",
            MAGNETOMETER_STACK_SIZE,
            paddle,
            1,
            &paddle->magnetometerTaskHandle,
            0
        );
        
        // Задачи на ядре 1
        xTaskCreatePinnedToCore(
            SPServerRTOS::bleSendTask,
            "BLESend",
            BLE_STACK_SIZE,
            paddle,
            2,  // Более высокий приоритет для send
            &paddle->bleSendTaskHandle,
            1
        );
    
        xTaskCreatePinnedToCore(
            SPServerRTOS::bleReceiveTask,
            "BLEReceive",
            BLE_RECEIVE_STACK_SIZE,
            paddle,
            1,
            &paddle->bleReceiveTaskHandle,
            1
        );

        // Установка прерывания для IMU на interruptPIN если он задан

        if (mainPaddle->imu && mainPaddle->imu->DMPEnabled() && mainPaddle->imu->interruptPIN()>=0){
            pinMode(mainPaddle->imu->interruptPIN(), INPUT);
            attachInterrupt(digitalPinToInterrupt(mainPaddle->imu->interruptPIN()), SPServerRTOS::dmpDataReady, RISING);
            Serial.printf("IMU interrupt pin initialized: %d\n", mainPaddle->imu->interruptPIN());
        }

        #ifdef HX711_USE_INTERRUPT
        if (mainPaddle->loads[0] && mainPaddle->loads[0]->getDRDYPin()>=0){
            attachInterrupt(digitalPinToInterrupt(mainPaddle->loads[0]->getDRDYPin()), SPServerRTOS::loadCellDataReady, FALLING);
            Serial.printf("Load cell 0 interrupt pin initialized: %d\n", mainPaddle->loads[0]->getDRDYPin());
        }
        if (mainPaddle->loads[1] && mainPaddle->loads[1]->getDRDYPin()>=0){
            attachInterrupt(digitalPinToInterrupt(mainPaddle->loads[1]->getDRDYPin()), SPServerRTOS::loadCellDataReady, FALLING);
            Serial.printf("Load cell 1 interrupt pin initialized: %d\n", mainPaddle->loads[1]->getDRDYPin());
        }
        #endif
    }
};

SmartPaddleBLEServer* SPServerRTOS::mainPaddle = nullptr;

void SmartPaddleBLEServer::calibrateIMU(){
    sendData=false; 
    if (logStream) logStream->println("CALIBRATE IMU COMMAND");
    if (imu) imu->calibrate();
    sendData=true;
}

void SmartPaddleBLEServer::calibrateLoads(BladeSideType blade_side){
    sendData=false;
    logStream->println("CALIBRATE LOADS COMMAND");
    switch(blade_side){
        case LEFT_BLADE:
            logStream->println("LEFT BLADE CALIBRATION");        
            if (loads[1])
                loads[1]->calibrate(); else
                logStream->println("No left blade");
            break;
        case RIGHT_BLADE:
            logStream->println("RIGHT BLADE CALIBRATION");
            if (loads[0])
                loads[0]->calibrate(); else
                logStream->println("No right blade");
            break;
        case ALL_BLADES:
            logStream->println("ALL BLADE CALIBRATION");        
            if (loads[0])
                loads[0]->calibrate(); else
                logStream->println("No right blade");
            if (loads[1])
                loads[1]->calibrate(); else
                logStream->println("No left blade");
            break;
    }
    sendData=true;
}

bool SmartPaddleBLEServer::updateLoads(){

    bool updated = false;
    if (loads[0] && (loads[0]->isDataReady())){
        loads[0]->read();
        lastLoadData.forceR = loads[0]->getForce();
        lastLoadData.forceR_raw = loads[0]->getRawForce();
        lastLoadData.timestamp = millis();
        updated=true;
    }
    if (loads[1] && (loads[1]->isDataReady())){
        loads[1]->read();
        lastLoadData.forceL = loads[1]->getForce();
        lastLoadData.forceL_raw = loads[1]->getRawForce();
        lastLoadData.timestamp = millis();
        updated=true;
    }

    if (0)
        Serial.printf("Loads: rigth: %f, left: %f, updated: %s\n",  lastLoadData.forceR, lastLoadData.forceL, updated?"true":"false");
    
    loadsensorQueue.send(lastLoadData);
    return updated;
}


// Конструктор класса SmartPaddle
SmartPaddleBLEServer::SmartPaddleBLEServer(const char* prefs_Name)
    : SmartPaddle(),
      orientationQueue(SENSOR_QUEUE_SIZE),
      imuQueue(SENSOR_QUEUE_SIZE),
      loadsensorQueue(SENSOR_QUEUE_SIZE),
      bladeQueue(SENSOR_QUEUE_SIZE),
      trustedDevice(nullptr),
      send_specs(false),
      is_pairing(false),
      imu(nullptr),
      loads{nullptr,nullptr},
      prefsName(prefs_Name),
      send_paddle_orientation(false),
      bleSendFrequency(SPServer_Default_Frequencies::BLE_SEND_FREQUENCY),
      bleReceiveFrequency(SPServer_Default_Frequencies::BLE_RECEIVE_FREQUENCY),
      loadCellFrequency(0),
      imuFrequency(0),
      orientationFrequency(0),
      time_to_send_specs(0),
      time_to_send_paddle_orientation(0),
      do_connect(false),
      time_to_connect(0),
      sendData(false),
      lastLoadData({0,0,0,0,0})
{
    //BLE Serial initialization
    serial=new SP_BLESerialServer(this);
    messageHandler=new SPServer_MessageHandler(this);
    serial->setMessageHandler(messageHandler);    
    connectTimer = xTimerCreate(
        "ConnectTimer",
        pdMS_TO_TICKS(CONNECT_DELAY_MS),
        pdFALSE,  // one-shot timer
        this,     // timer ID
        connectTimerCallback
    );

}

void SmartPaddleBLEServer::setLogStream(Stream* stream){
    logStream = stream;
    if (loads[0])
        loads[0]->setLogStream(stream);
    if (loads[1])
        loads[1]->setLogStream(stream);
    if (imu)
        imu->setLogStream(stream);
}

// класс для подключения и отключения устройства
class SPBLEServerCallbacks: public BLEServerCallbacks {
    private:
    SmartPaddleBLEServer* server;
    
    public:
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
//        Serial.println("On Connect");
        BLEAddress clientAddress(param->connect.remote_bda);
        server->conn_id=param->connect.conn_id;
        
        if (server->connected()){
            Serial.printf("Already connected to %s\n", server->trustedDevice->toString().c_str());
            return;
        }

        if (!server->trustedDevice || server->isPairing()) {
            // Если нет сохраненного доверенного устройства, сохраняем первое подключившееся
            Serial.println("No trusted device, allowing new connection");
            server->setTrustedDevice(&clientAddress);
            server->conn_id=param->connect.conn_id;
            server->connect();
        } 
        else if (server->trustedDevice->equals(clientAddress)) {
            // Если адрес совпадает с сохраненным доверенным устройством
            Serial.printf("Trusted device connected: %s\n", clientAddress.toString().c_str());
            server->conn_id=param->connect.conn_id;
            server->connect();
        } 
        else {
            // Если адрес не совпадает с доверенным устройством
            Serial.printf("Untrusted device rejected: %s\n", clientAddress.toString().c_str());
            pServer->disconnect(param->connect.conn_id);
        }
    }

    void onDisconnect(BLEServer* pServer) {
        server->disconnect();
    }

    SPBLEServerCallbacks(SmartPaddleBLEServer* serverPTR) : server(serverPTR) {}
};

class MySecurity : public BLESecurityCallbacks {
  
  bool onConfirmPIN(uint32_t pin){
    return false;
  }
  
	uint32_t onPassKeyRequest(){
        Serial.println("PassKeyRequest");
		return 123456;
	}

	void onPassKeyNotify(uint32_t pass_key){
        Serial.printf("On passkey Notify number:%d", pass_key);
	}

	bool onSecurityRequest(){
	    Serial.println("On Security Request");
		return true;
	}

	void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl){
		Serial.println("On Authentication Complete");
		if(cmpl.success){
			uint16_t length;
			esp_ble_gap_get_whitelist_size(&length);
			Serial.printf("size: %d", length);
		}
	}
};

void SmartPaddleBLEServer::begin(const char* deviceName) {

    // Load trusted device and blade orientation from EEPROM
    loadTrustedDevice();
    // Load blade orientation from EEPROM
    loadBladeOrientation();
    loadSpecs();
    // Initialize BLE
    Serial.println("Device name: " + String(deviceName));
    BLEDevice::init(deviceName);
    BLEDevice::setMTU(BLEMTU);
    BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
    BLEDevice::setSecurityCallbacks(new MySecurity());
    
    // Настройка безопасности с сохранением указателя
    pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
    pSecurity->setCapability(ESP_IO_CAP_NONE);
    pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    
    Serial.println("Creating server");
    pServer = BLEDevice::createServer();

    pServer->setCallbacks(new SPBLEServerCallbacks(this));

    BLEService *pService = pServer->createService(BLEUUID(SmartPaddleUUID::SERVICE_UUID),30,0);
    forceCharacteristic = pService->createCharacteristic(
        SmartPaddleUUID::FORCE_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    imuCharacteristic = pService->createCharacteristic(
        SmartPaddleUUID::IMU_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    orientationCharacteristic = pService->createCharacteristic(
        SmartPaddleUUID::ORIENTATION_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pService->start();

    // Start BLE serial communication
    serial->begin();

    // Start BLE advertising
    startAdvertising(BLEDevice::getAdvertising());
    Serial.println("Service started");
    Serial.printf("Bonded devices count: %d\n", esp_ble_get_bond_device_num());


    //Set default frequencies
    #ifdef HX711_USE_INTERRUPT
    loadCellFrequency = 0;
    #else
    loadCellFrequency = (loads[0])?loads[0]->getFrequency():0;
    if (loads[1])
        loadCellFrequency = (loads[1]->getFrequency()<loadCellFrequency)?loads[1]->getFrequency():loadCellFrequency;
    #endif

    if (imu){
        imuFrequency = imu->getFrequency();
        if (imu->DMPEnabled()&&imu->interruptPIN()>=0)
            imuFrequency = 0;
    }
    
    orientationFrequency = (imu)?imu->getFrequency():0;
    // Start RTOS tasks
 //   SPServerRTOS::startTasks(this);

}

void SmartPaddleBLEServer::startAdvertising(BLEAdvertising* advertising){
    advertising->addServiceUUID(SmartPaddleUUID::SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("Advertising started (SmartPaddleBLEServer::startAdvertising)");
}

void SmartPaddleBLEServer::connectTimerCallback(TimerHandle_t timer) {
    SmartPaddleBLEServer* server = static_cast<SmartPaddleBLEServer*>(pvTimerGetTimerID(timer));
    if (server->connected()){
        server->sendData = true;
        BLEDevice::stopAdvertising();
        server->sendSpecs(1500);
        server->sendPaddleOrientation(1500);
    }
}


// Метод для подключения к веслу
bool SmartPaddleBLEServer::connect() {
    // Логика подключения к веслу
    is_pairing = false; 
    Serial.println("Connected");
    Serial.printf("Bonded devices count: %d\n", esp_ble_get_bond_device_num());
    isConnected = true;
    sendData=false;
    xTimerStart(connectTimer, 0);
    if (eventHandler)
        eventHandler->onConnect(this);

    return true;
}

// Мето для отключения от весла
void SmartPaddleBLEServer::disconnect() {
    // Логика отключения от весла

    if (isConnected){
        isConnected = false;
        pServer->disconnect(conn_id);
        if (eventHandler)
            eventHandler->onDisconnect(this);
        vTaskDelay(pdMS_TO_TICKS(500));  // неблокирующая задержка
        startAdvertising(BLEDevice::getAdvertising());
    }
}

void SmartPaddleBLEServer::setTrustedDevice(BLEAddress* address) {

    Preferences prefs;
    prefs.begin(prefsName.c_str(), false);
    prefs.putString("trustedDevice_string", address->toString().c_str());
    prefs.putBytes("trustedDevice", address->getNative(), sizeof(esp_bd_addr_t));
    prefs.end();
    
    if (trustedDevice) {
        delete trustedDevice;
    }
    trustedDevice = new BLEAddress(*address);
}

void SmartPaddleBLEServer::loadTrustedDevice() {
    if (trustedDevice) {
        delete trustedDevice;
        trustedDevice = nullptr;
    }

    Preferences prefs;
    prefs.begin(prefsName.c_str(), true);
    if (prefs.isKey("trustedDevice")) {
        Serial.println("Loading trusted device");
        esp_bd_addr_t address;
        prefs.getBytes("trustedDevice", &address, sizeof(esp_bd_addr_t));
        trustedDevice = new BLEAddress(address);
        Serial.printf("Trusted device: %s\n", trustedDevice->toString().c_str());
    } else {
        Serial.println("No trusted device found in EEPROM");
    }
    prefs.end();
}

void SmartPaddleBLEServer::clearTrustedDevice() {
    Preferences prefs;
    prefs.begin(prefsName.c_str(), false);
    prefs.remove("trustedDevice");
    prefs.remove("trustedDevice_string");
    prefs.end();
    
    if (trustedDevice) {
        delete trustedDevice;
        trustedDevice = nullptr;
    }
}

void SmartPaddleBLEServer::printBondedDevices() {
    int deviceCount = esp_ble_get_bond_device_num();
    Serial.printf("Bonded devices count: %d\n", deviceCount);
    
    if (deviceCount > 0) {
        esp_ble_bond_dev_t* deviceList = (esp_ble_bond_dev_t*)malloc(sizeof(esp_ble_bond_dev_t) * deviceCount);
        esp_ble_get_bond_device_list(&deviceCount, deviceList);
        
        for (int i = 0; i < deviceCount; i++) {
            BLEAddress address(deviceList[i].bd_addr);
            Serial.printf("Device %d: %s\n", i, address.toString().c_str());
        }
        
        free(deviceList);
    }
}

void SmartPaddleBLEServer::removeAllBondedDevices() {
    int deviceCount = esp_ble_get_bond_device_num();
    
    if (deviceCount > 0) {
        esp_ble_bond_dev_t* deviceList = (esp_ble_bond_dev_t*)malloc(sizeof(esp_ble_bond_dev_t) * deviceCount);
        esp_ble_get_bond_device_list(&deviceCount, deviceList);
        
        for (int i = 0; i < deviceCount; i++) {
            esp_ble_remove_bond_device(deviceList[i].bd_addr);
        }
        
        free(deviceList);
        Serial.println("All bonded devices removed");
    }
}

void SmartPaddleBLEServer::removeBondedDevice(BLEAddress address) {
    if (esp_ble_remove_bond_device(*address.getNative()) == ESP_OK) {
        Serial.printf("Device %s removed from bonding list\n", address.toString().c_str());
    } else {
        Serial.printf("Failed to remove device %s\n", address.toString().c_str());
    }
}

bool SmartPaddleBLEServer::isBonded(BLEAddress address) {
    int deviceCount = esp_ble_get_bond_device_num();
    bool found = false;
    
    if (deviceCount > 0) {
        esp_ble_bond_dev_t* deviceList = (esp_ble_bond_dev_t*)malloc(sizeof(esp_ble_bond_dev_t) * deviceCount);
        esp_ble_get_bond_device_list(&deviceCount, deviceList);
        
        for (int i = 0; i < deviceCount; i++) {
            BLEAddress bondedAddress(deviceList[i].bd_addr);
            if (address.equals(bondedAddress)) {
                found = true;
                break;
            }
        }
        
        free(deviceList);
    }
    
    return found;
}

void SmartPaddleBLEServer::startPairing(){
    if (connected()){
        disconnect();
    } else {
        BLEDevice::stopAdvertising();
    }
    removeAllBondedDevices();
    vTaskDelay(pdMS_TO_TICKS(500));  // неблокирующая задержка
    is_pairing = true;
    startAdvertising(BLEDevice::getAdvertising());
}

void SmartPaddleBLEServer::updateBLE(){

    if(connected()&&sendData) {
        loadData loadSensorData;
        IMUData imuDataStruct;
        PaddleStatus statusData;
        OrientationData orientationData;
        BladeData bladeData;

        // Получение данных из очередей
        if(loadsensorQueue.receive(loadSensorData,0)) {
            forceCharacteristic->setValue((uint8_t*)&loadSensorData, sizeof(loadData));
            forceCharacteristic->notify();
        }

        if(imuQueue.receive(imuDataStruct,0)) {
            imuCharacteristic->setValue((uint8_t*)&imuDataStruct, sizeof(IMUData));
            imuCharacteristic->notify();
        }


        if(orientationQueue.receive(orientationData,0)) {
            orientationCharacteristic->setValue((uint8_t*)&orientationData, sizeof(OrientationData));
            orientationCharacteristic->notify();
        }

        if(send_specs && millis()>=time_to_send_specs) {
            serial->sendString(SP_MessageProcessor::createSpecsMessage(specs));
            send_specs = false;

        }

        if (send_paddle_orientation && millis()>=time_to_send_paddle_orientation) {
            serial->sendString(SP_MessageProcessor::createBladeOrientationMessage(bladeOrientation));
            send_paddle_orientation = false;
        }

    }
    if (connected()&&serial) serial->update();

}

void SmartPaddleBLEServer::shutdown() {
    if (logStream)
        logStream->println("Shutdown Paddle");
    if (eventHandler)
        eventHandler->onShutdown(this);
}

IMUData SmartPaddleBLEServer::getIMUData(){
    return imu->getData();
}

loadData SmartPaddleBLEServer::getLoadData(){
    return lastLoadData;
}

OrientationData SmartPaddleBLEServer::getOrientationData(){
    return imu->getOrientation();
}

void SmartPaddleBLEServer::calibrateBladeAngle(BladeSideType blade_side) {
    if (!imu) return;

    // Получаем текущий кватернион ориентации
    IMUData data = imu->getData();
    SP_Math::Quaternion q(data.q0, data.q1, data.q2, data.q3);

    // Вектор вертикально вверх в глобальной системе координат
    SP_Math::Vector global_up(0.0f, 0.0f, 1.0f);

    // Преобразуем в локальную систему координат устройства
    SP_Math::Vector local_up = q.conjugate().rotate(global_up);

    // Проекция вектора на плоскость XZ локальной системы (игнорируем Y компоненту)
    float x = local_up.x();
    float z = local_up.z();
    
    // Вычисляем угол между проекцией и осью Z в локальной системе координат
    float blade_angle = atan2(x, z);

    if (logStream) {
        logStream->printf("Calibrating %s blade angle\n", 
            blade_side == RIGHT_BLADE ? "right" : "left");
        logStream->printf("Global normal vector: %.3f, %.3f, %.3f\n", 
            local_up.x(), local_up.y(), local_up.z());
        logStream->printf("Blade angle around Y axis: %.1f degrees\n", 
            blade_angle * 180.0f / M_PI);
    }

    // Сохраняем угол
    if (blade_side == RIGHT_BLADE) {
        bladeOrientation.rightBladeAngle = blade_angle;
        bladeOrientation.rightBladeVector[0] = x;
        bladeOrientation.rightBladeVector[1] = 0;
        bladeOrientation.rightBladeVector[2] = z;
    } else if (blade_side == LEFT_BLADE) {
        bladeOrientation.leftBladeAngle = blade_angle;
        bladeOrientation.leftBladeVector[0] = x;
        bladeOrientation.leftBladeVector[1] = 0;
        bladeOrientation.leftBladeVector[2] = z;
    } else {
        Serial.printf("Calibrate blade angle command with no side\n");
        bladeOrientation.rightBladeAngle = blade_angle;
        bladeOrientation.rightBladeVector[0] = x;
        bladeOrientation.rightBladeVector[1] = 0;
        bladeOrientation.rightBladeVector[2] = z;        
        bladeOrientation.leftBladeAngle = blade_angle;
        bladeOrientation.leftBladeVector[0] = x;
        bladeOrientation.leftBladeVector[1] = 0;
        bladeOrientation.leftBladeVector[2] = z;
    }

    // Сохраняем в память
    Preferences prefs;
    prefs.begin(prefsName.c_str(), false);
    if (blade_side == RIGHT_BLADE) {
        prefs.putFloat("rightBladeAngle", blade_angle);
        prefs.putFloat("rightBVectorX", x);
        prefs.putFloat("rightBVectorY", 0);
        prefs.putFloat("rightBVectorZ", z);
    } else if (blade_side == LEFT_BLADE) {
        prefs.putFloat("leftBladeAngle", blade_angle);
        prefs.putFloat("leftBVectorX", x);
        prefs.putFloat("leftBVectorY", 0);
        prefs.putFloat("leftBVectorZ", z);
    }
    prefs.end();
    sendPaddleOrientation(0);
}

bool SmartPaddleBLEServer::loadBladeOrientation(){
    Preferences prefs;
    prefs.begin(prefsName.c_str(), true);
     bladeOrientation.YAxisDirection = prefs.getInt("yAxisDirection", 1);
    bladeOrientation.rightBladeAngle = prefs.getFloat("rightBladeAngle", 0);
    bladeOrientation.leftBladeAngle = prefs.getFloat("leftBladeAngle", 0);
    bladeOrientation.rightBladeVector[0] = prefs.getFloat("rightBVectorX", 0);
    bladeOrientation.rightBladeVector[1] = prefs.getFloat("rightBVectorY", 0);
    bladeOrientation.rightBladeVector[2] = prefs.getFloat("rightBVectorZ", 1);
    bladeOrientation.leftBladeVector[0] = prefs.getFloat("leftBVectorX", 0);
    bladeOrientation.leftBladeVector[1] = prefs.getFloat("leftBVectorY", 0);
    bladeOrientation.leftBladeVector[2] = prefs.getFloat("leftBVectorZ", 1);
    prefs.end();
    sendPaddleOrientation(1000);
    return true;
}

void SmartPaddleBLEServer::saveSpecs(){
    Preferences prefs;
    prefs.begin(prefsName.c_str(), false);
    prefs.putString("paddleID", specs.paddleID);
    prefs.putInt("paddleType", specs.paddleType);
    prefs.putFloat("length", specs.length);
    prefs.putFloat("imuDistance", specs.imuDistance);
    prefs.putFloat("bladeWeight", specs.bladeWeight);
    prefs.putFloat("bladeCenter", specs.bladeCenter);
    prefs.putFloat("bladeMomeInert", specs.bladeMomentInertia);
    prefs.putInt("firmwareVersion", specs.firmwareVersion);
    prefs.putString("paddleModel", specs.paddleModel);
    prefs.putBool("hasLeftBlade", specs.hasLeftBlade);
    prefs.putBool("hasRightBlade", specs.hasRightBlade);
    prefs.putInt("imuFrequency", specs.imuFrequency);
    prefs.end();
}

bool SmartPaddleBLEServer::loadSpecs(){
    Preferences prefs;
    if (!prefs.begin(prefsName.c_str(), true)) {
        return false;
    }
//    specs.paddleID = prefs.getString("paddleID", "");
    specs.paddleType = (PaddleType)prefs.getInt("paddleType", 0);
    specs.length = prefs.getFloat("length", 0);
    specs.imuDistance = prefs.getFloat("imuDistance", 0);
    specs.bladeWeight = prefs.getFloat("bladeWeight", 0);
    specs.bladeCenter = prefs.getFloat("bladeCenter", 0);
    specs.bladeMomentInertia = prefs.getFloat("bladeMomeInert", 0);
    specs.firmwareVersion = prefs.getInt("firmwareVersion", 0);
    specs.paddleModel = prefs.getString("paddleModel", "");
    specs.hasLeftBlade = prefs.getBool("hasLeftBlade", false);
    specs.hasRightBlade = prefs.getBool("hasRightBlade", false);
    specs.imuFrequency = prefs.getInt("imuFrequency", 0);
    prefs.end();
    return true;
}

void SmartPaddleBLEServer::startTasks() {
    SPServerRTOS::startTasks(this);
}




