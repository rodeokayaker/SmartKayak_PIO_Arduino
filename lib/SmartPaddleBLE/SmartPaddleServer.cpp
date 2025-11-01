#include <SmartPaddleServer.h>
#include <Preferences.h>
#include <SP_BLESerialServer.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_gatts_api.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <SP_Quaternion.h>
#include <SP_CommandParams.h>


// FreeRTOS определения
#define SENSOR_STACK_SIZE 4096
#define IMU_STACK_SIZE 4096
#define ORIENTATION_STACK_SIZE 4096
#define MAGNETOMETER_STACK_SIZE 4096
#define BLE_STACK_SIZE 4096
#define BLE_RECEIVE_STACK_SIZE 8192

/**
 * @brief Обработчик сообщений для SmartPaddleServer (мигрирован на callback подход)
 */
class SPServer_MessageHandler: public SP_MessageHandler{
private:
    SmartPaddleBLEServer* paddle;

    void setupHandlers() {
        // ============================================
        // КОМАНДЫ
        // ============================================
        
        // Калибровка IMU
        registerCommand(SP_Protocol::Commands::CALIBRATE_IMU,
            [this](SP_Command* cmd) {
                Serial.println("Calibrate IMU command");
                paddle->sendData = false;
                paddle->setLogStream(paddle->getSerial());
                paddle->calibrateIMU();
                paddle->setLogStream(&Serial);
                paddle->getSerial()->sendString(
                    SP_MessageProcessor::createSuccessResponse(cmd->command.c_str(), "IMU calibrated")
                );
                paddle->sendData = true;
            });

        // Калибровка датчиков нагрузки
        registerCommand(SP_Protocol::Commands::CALIBRATE_LOADS,
            [this](SP_Command* cmd) {
                Serial.println("Calibrate loads command");
                SP_CommandParams params(cmd);
                BladeSideType bladeSide = params.getBladeSide();
                
                paddle->sendData = false;
                paddle->setLogStream(paddle->getSerial());
                paddle->calibrateLoads(bladeSide);
                paddle->setLogStream(&Serial);
                paddle->getSerial()->sendString(
                    SP_MessageProcessor::createSuccessResponse(cmd->command.c_str(), "Loads calibrated")
                );
                paddle->sendData = true;
            });

        // Тарирование датчиков
        registerCommand(SP_Protocol::Commands::TARE_LOADS,
            [this](SP_Command* cmd) {
                Serial.println("Tare loads command");
                SP_CommandParams params(cmd);
                BladeSideType bladeSide = params.getBladeSide();
                
                paddle->sendData = false;
                paddle->setLogStream(paddle->getSerial());
                paddle->loads->tare(bladeSide);
                paddle->setLogStream(&Serial);
                paddle->getSerial()->sendString(
                    SP_MessageProcessor::createSuccessResponse(cmd->command.c_str(), "Loads tared")
                );
                paddle->sendData = true;
            });

        // Калибровка угла лопасти
        registerCommand(SP_Protocol::Commands::CALIBRATE_BLADE_ANGLE,
            [this](SP_Command* cmd) {
                Serial.println("Calibrate blade angle command");
                SP_CommandParams params(cmd);
                BladeSideType bladeSide = params.getBladeSide();
                
                paddle->setLogStream(paddle->getSerial());
                paddle->calibrateBladeAngle(bladeSide);
                paddle->setLogStream(&Serial);
                paddle->getSerial()->sendString(
                    SP_MessageProcessor::createSuccessResponse(cmd->command.c_str(), "Blade angle calibrated")
                );
            });

        // Калибровка компаса
        registerCommand(SP_Protocol::Commands::CALIBRATE_COMPASS,
            [this](SP_Command* cmd) {
                Serial.println("Calibrate compass command");
                paddle->setLogStream(paddle->getSerial());
                paddle->sendData = false;
                // paddle->imu->calibrateCompass();
                paddle->sendData = true;
                paddle->setLogStream(&Serial);
                paddle->getSerial()->sendString(
                    SP_MessageProcessor::createSuccessResponse(cmd->command.c_str(), "Compass calibrated")
                );
            });

        // Отправка спецификаций
        registerCommand(SP_Protocol::Commands::SEND_SPECS,
            [this](SP_Command* cmd) {
                Serial.println("Send specs command");
                paddle->sendSpecs();
            });

        registerCommand(SP_Protocol::Commands::SEND_PADDLE_ORIENTATION,
            [this](SP_Command* cmd) {
                Serial.println("Send paddle orientation command");
                paddle->sendPaddleOrientation();
            });

        // Старт сопряжения
        registerCommand(SP_Protocol::Commands::START_PAIR,
            [this](SP_Command* cmd) {
                Serial.println("Start pair command");
                paddle->startPairing();
            });

        // Выключение
        registerCommand(SP_Protocol::Commands::SHUTDOWN,
            [this](SP_Command* cmd) {
                Serial.println("Shutdown command");
                paddle->shutdown();
            });

        // Установка калибровки магнитометра
        registerCommand(SP_Protocol::Commands::SET_MAGNETOMETER_CALIBRATION,
            [this](SP_Command* cmd) {
                Serial.println("Set magnetometer calibration command");
                SP_CommandParams params(cmd);
                
                float offset[3];
                float softIron[6];
                params.getMagnetometerCalibration(offset, softIron);
                
                // TODO: Раскомментировать когда будет реализация
                // IMUCalibData calibData = paddle->imu->getCalibrationData();
                // calibData.magOffset[0] = offset[0];
                // calibData.magOffset[1] = offset[1];
                // calibData.magOffset[2] = offset[2];
                // calibData.magScale[0] = softIron[0];
                // calibData.magScale[1] = softIron[1];
                // calibData.magScale[2] = softIron[2];
                // calibData.magSI[0] = softIron[3];
                // calibData.magSI[1] = softIron[4];
                // calibData.magSI[2] = softIron[5];
                // paddle->imu->setCalibrationData(calibData, true);
            });

        // Отправка калибровочных данных
        registerCommand(SP_Protocol::Commands::SEND_CALIBRATION_DATA,
            [this](SP_Command* cmd) {
                Serial.println("Send calibration data command");
                // TODO: Раскомментировать когда будет реализация
                // paddle->getSerial()->sendString(
                //     SP_MessageProcessor::createMagnetometerCalibrationDataMessage(
                //         paddle->imu->getCalibrationData()
                //     )
                // );
            });

        // ============================================
        // ДАННЫЕ
        // ============================================
        
        // Обработка спецификаций
        registerData(SP_Protocol::DataTypes::SPECS,
            [this](SP_Data* data) {
                Serial.println("Got Specs data");
                SP_DataValue value(data);
                
                String paddleID = paddle->specs.paddleID;
                paddle->specs.paddleID = value.get<String>(SP_Protocol::DataTypes::Specs::PADDLE_ID, "");
                paddle->specs.paddleType = (PaddleType)value.get<int>(SP_Protocol::DataTypes::Specs::PADDLE_TYPE, 0);
                paddle->specs.paddleModel = value.get<String>(SP_Protocol::DataTypes::Specs::PADDLE_MODEL, "");
                paddle->specs.length = value.get<float>(SP_Protocol::DataTypes::Specs::LENGTH, 2.0f);
                paddle->specs.imuFrequency = value.get<int>(SP_Protocol::DataTypes::Specs::IMU_FREQUENCY, 0);
                paddle->specs.hasLeftBlade = value.get<bool>(SP_Protocol::DataTypes::Specs::HAS_LEFT_BLADE, false);
                paddle->specs.hasRightBlade = value.get<bool>(SP_Protocol::DataTypes::Specs::HAS_RIGHT_BLADE, false);
                paddle->specs.firmwareVersion = value.get<int>(SP_Protocol::DataTypes::Specs::FIRMWARE_VERSION, 0);
                paddle->specs.imuDistance = value.get<float>(SP_Protocol::DataTypes::Specs::IMU_DISTANCE, 0.0f);
                paddle->specs.axisDirection = (AxisDirection)value.get<int>(SP_Protocol::DataTypes::Specs::AXIS_DIRECTION, (int)Y_AXIS_RIGHT);
                paddle->specs.bladeWeight = value.get<float>(SP_Protocol::DataTypes::Specs::BLADE_WEIGHT, 0.3f);
                paddle->specs.bladeCenter = value.get<float>(SP_Protocol::DataTypes::Specs::BLADE_CENTER, 0.20f);
                paddle->specs.bladeMomentInertia = value.get<float>(SP_Protocol::DataTypes::Specs::BLADE_MOMENT_INERTIA, 0.01f);
                paddle->specs.paddleID = paddleID; // Восстанавливаем оригинальный ID
                
                paddle->saveSpecs();
            });
    }

public:
    SPServer_MessageHandler(SmartPaddleBLEServer* p) : paddle(p) {
        setupHandlers();
    }

    // ============================================
    // ПЕРЕОПРЕДЕЛЕНИЕ БАЗОВЫХ ОБРАБОТЧИКОВ
    // ============================================
    
    void onLog(SP_LogMessage* log) override {
        Serial.printf("Kayak log: %s\n", log->message.c_str());
    }

    void onUnknownCommand(SP_Command* command) override {
        Serial.printf("Paddle got unknown command: %s\n", command->command.c_str());
        paddle->getSerial()->sendString(
            SP_MessageProcessor::createErrorResponse(command->command.c_str(), "Unknown command")
        );
    }

    void onResponse(SP_Response* response) override {
        Serial.printf("Kayak response: %s\n", response->command.c_str());
    }

    void onUnknownData(SP_Data* data) override {
        Serial.printf("Kayak data: %s\n", data->dataType.c_str());
    }

    void onStatus(SP_StatusMessage* status) override {
        Serial.println("Got Kayak status");
    }
};


void interruptHandler(){
    Serial.printf("IMU interrupt\n");
}

class SPServerRTOS{
    private:
    static uint32_t lastLoadData;
    static uint32_t lastIMUData;
    static uint32_t lastOrientationData;
    static SmartPaddleBLEServer* mainPaddle;

    static void onIMUData(const IMUData &data){
        if (mainPaddle->connected()) {
            if (mainPaddle->sendData) {
                if (millis()-lastIMUData > 5) {
                    mainPaddle->imuQueue.send(data);
                    lastIMUData = millis();
                    xTaskNotifyGive(mainPaddle->bleSendTaskHandle);
                }
            }
        }
    }

    static void onOrientationData(const OrientationData &data){
        if (mainPaddle->connected()) {
            if (mainPaddle->sendData) {
                if (millis()-lastOrientationData > 5) {
                    mainPaddle->orientationQueue.send(data);
                    lastOrientationData = millis();
                    xTaskNotifyGive(mainPaddle->bleSendTaskHandle);
                }
            }

        }

    }

    static void onLoadData(const loadData &data, BladeSideType bladeSide){
        if (mainPaddle->connected()) {
            if (mainPaddle->sendData) {
                if (millis()-lastLoadData > 5) {
                    mainPaddle->loadsensorQueue.send(data);
                    lastLoadData = millis();
                    xTaskNotifyGive(mainPaddle->bleSendTaskHandle);
                }
            }
        }
    }   

// Задача чтения тензодатчиков
/*
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
        */
/*
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
        */

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

/*
    static void IRAM_ATTR loadCellDataReady() {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(mainPaddle->loadCellTaskHandle, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }

    */

    public:
    //All tasks setup and start
    static void startTasks(SmartPaddleBLEServer* paddle) {
        mainPaddle = paddle;

        paddle->imu->onOrientation(SPServerRTOS::onOrientationData);
        paddle->imu->onIMUData(SPServerRTOS::onIMUData);
        Serial.printf("Starting IMU services\n");
        paddle->imu->startServices();

        paddle->loads->onLoadData(SPServerRTOS::onLoadData, false);
        Serial.printf("Starting Load cell services\n");
        paddle->loads->startServices();

/*        xTaskCreatePinnedToCore(
            SPServerRTOS::loadCellTask,
            "LoadCell",
            SENSOR_STACK_SIZE,
            paddle,
            1,
            &paddle->loadCellTaskHandle,
            0
        );
*/    

/*
        xTaskCreatePinnedToCore(
            SPServerRTOS::magnetometerTask,
            "Magnetometer",
            MAGNETOMETER_STACK_SIZE,
            paddle,
            1,
            &paddle->magnetometerTaskHandle,
            0
        );
        */
        
        // Задачи на ядре 1
        Serial.printf("Starting BLE send task\n");
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

        /*

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
        */
    }
};

SmartPaddleBLEServer* SPServerRTOS::mainPaddle = nullptr;
uint32_t SPServerRTOS::lastLoadData = 0;
uint32_t SPServerRTOS::lastIMUData = 0;
uint32_t SPServerRTOS::lastOrientationData = 0;

void SmartPaddleBLEServer::calibrateIMU(){
    sendData=false; 
    logStream->println("CALIBRATE IMU COMMAND");
    if (imu) imu->calibrate();
    sendData=true;
}

void SmartPaddleBLEServer::calibrateLoads(BladeSideType blade_side){
    sendData=false;
    logStream->println("CALIBRATE LOADS COMMAND");
    switch(blade_side){
        case LEFT_BLADE:
            logStream->println("LEFT BLADE CALIBRATION");        
            if (loads)
                loads->calibrate(BladeSideType::LEFT_BLADE,1000); else
                logStream->println("No left blade");
            break;
        case RIGHT_BLADE:
            logStream->println("RIGHT BLADE CALIBRATION");
            if (loads)
                loads->calibrate(BladeSideType::RIGHT_BLADE,1000); else
                logStream->println("No right blade");
            break;
        case ALL_BLADES:
            logStream->println("ALL BLADE CALIBRATION");        
            if (loads)
                loads->calibrate(BladeSideType::ALL_BLADES,1000); else
                logStream->println("No all blades");

            break;
    }
    sendData=true;
}

/*

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

*/

// Конструктор класса SmartPaddle
SmartPaddleBLEServer::SmartPaddleBLEServer(const char* prefs_Name)
    : SmartPaddleBLE(),
      orientationQueue(SENSOR_QUEUE_SIZE),
      imuQueue(SENSOR_QUEUE_SIZE),
      loadsensorQueue(SENSOR_QUEUE_SIZE),
      bladeQueue(SENSOR_QUEUE_SIZE),
      trustedDevice(nullptr),
      send_specs(false),
      is_pairing(false),
      imu(nullptr),
      loads(nullptr),
      prefsName(prefs_Name),
      send_paddle_orientation(false),
      bleSendFrequency(SPServer_Default_Frequencies::BLE_SEND_FREQUENCY),
      bleReceiveFrequency(SPServer_Default_Frequencies::BLE_RECEIVE_FREQUENCY),
      loadCellFrequency(0),
      time_to_send_specs(0),
      time_to_send_paddle_orientation(0),
      do_connect(false),
      time_to_connect(0),
      sendData(false),
      lastLoadData({0,0,0,0,0}),
      logStream(&Serial)
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

    specs.axisDirection = Y_AXIS_LEFT;
    specs.length = 2.0f;
    specs.imuDistance = 0.00f;
    specs.bladeWeight = 0.3f;
    specs.bladeCenter = 0.20f;
    specs.bladeMomentInertia = 0.01;
    specs.firmwareVersion = 0;
    specs.paddleModel = "No MODEL";
    specs.hasLeftBlade = true;
    specs.hasRightBlade = true;
    specs.imuFrequency = 0;

}

void SmartPaddleBLEServer::setLogStream(Stream* stream){
    logStream = stream;
    if (loads)
        loads->setLogStream(stream);
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
    Serial.println("Starting SmartPaddleBLEServer::begin()");
    
 /*   // Инициализация NVS если еще не инициализирован
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        Serial.println("Erasing and reinitializing NVS...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    Serial.println("NVS ready for SmartPaddle");
*/
    // Load trusted device and blade orientation from EEPROM
    Serial.println("Loading trusted device...");
    loadTrustedDevice();
    
    // Load blade orientation from EEPROM
    Serial.println("Loading blade orientation...");
    loadBladeOrientation();
    
    Serial.println("Loading specs...");
    loadSpecs();
    // Initialize BLE с дополнительными проверками
    Serial.println("Device name: " + String(deviceName));
    
    // Проверяем свободную память перед инициализацией BLE
    Serial.printf("Free heap before BLE init: %d bytes\n", ESP.getFreeHeap());
    
    BLEDevice::init(deviceName);
    delay(200); // Даем время BLE стеку инициализироваться
    
    // Устанавливаем MTU с проверкой
    Serial.printf("Setting MTU to %d\n", BLEMTU);
    BLEDevice::setMTU(BLEMTU);
    delay(100);
    
    BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
    BLEDevice::setSecurityCallbacks(new MySecurity());
    delay(100);
    
    Serial.printf("Free heap after BLE init: %d bytes\n", ESP.getFreeHeap());
    
    // Настройка безопасности с сохранением указателя
    pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
    pSecurity->setCapability(ESP_IO_CAP_NONE);
    pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    
    Serial.println("Creating server");
    pServer = BLEDevice::createServer();
    if (!pServer) {
        Serial.println("❌ Failed to create BLE server");
        return;
    }

    // Используем статический callback вместо создания нового объекта
    static SPBLEServerCallbacks serverCallbacks(this);
    pServer->setCallbacks(&serverCallbacks);
    Serial.println("✓ Created server with callbacks");

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
    
    // Безопасная проверка количества связанных устройств
    delay(100); // Даем время BLE стеку стабилизироваться
    try {
        int bondedCount = esp_ble_get_bond_device_num();
        Serial.printf("Bonded devices count: %d\n", bondedCount);
    } catch (...) {
        Serial.println("⚠️ Failed to get bonded devices count");
    }


    //Set default frequencies
    #ifdef HX711_USE_INTERRUPT
    loadCellFrequency = 0;
    #else
    loadCellFrequency = (loads[0])?loads[0]->getFrequency():0;
    if (loads[1])
        loadCellFrequency = (loads[1]->getFrequency()<loadCellFrequency)?loads[1]->getFrequency():loadCellFrequency;
    #endif

    if (specs.imuFrequency == 0)
        specs.imuFrequency = IMU_DEFAULT_FREQUENCY;
    imu->setIMUFrequency(specs.imuFrequency);
    imu->setOrientationFrequency(specs.imuFrequency);

    // Start RTOS tasks
 //   SPServerRTOS::startTasks(this);

    Serial.printf("SmartPaddleBLEServer initialization completed. Free heap: %d bytes\n", ESP.getFreeHeap());
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
        server->sendSpecs(1000);
        vTaskDelay(pdMS_TO_TICKS(50));
        server->sendPaddleOrientation(1500);
        vTaskDelay(pdMS_TO_TICKS(50));
        server->sendData = true;

    }
}


// Метод для подключения к веслу
bool SmartPaddleBLEServer::connect() {
    // Логика подключения к веслу
    is_pairing = false; 
    Serial.println("Connected");
    
    // Безопасная проверка количества связанных устройств
    try {
        int bondedCount = esp_ble_get_bond_device_num();
        Serial.printf("Bonded devices count: %d\n", bondedCount);
    } catch (...) {
        Serial.println("⚠️ Failed to get bonded devices count");
    }
    isConnected = true;
    sendData=false;

    BLEDevice::stopAdvertising();

    xTimerStart(connectTimer, 0);
    for (int i = 0; i < eventHandlerCount; i++) 
        eventHandler[i]->onConnect(this);

    return true;
}

// Мето для отключения от весла
void SmartPaddleBLEServer::disconnect() {
    // Логика отключения от весла

    if (isConnected){
        isConnected = false;
        pServer->disconnect(conn_id);
        for (int i = 0; i < eventHandlerCount; i++) 
            eventHandler[i]->onDisconnect(this);
        vTaskDelay(pdMS_TO_TICKS(500));  // неблокирующая задержка
        startAdvertising(BLEDevice::getAdvertising());
    }
}

void SmartPaddleBLEServer::setTrustedDevice(BLEAddress* address) {

    Preferences prefs;
    prefs.begin(prefsName.c_str(), false);
    prefs.putString("trustedDevStr", address->toString().c_str());
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
    prefs.remove("trustedDevStr");
    prefs.end();
    
    if (trustedDevice) {
        delete trustedDevice;
        trustedDevice = nullptr;
    }
}

void SmartPaddleBLEServer::printBondedDevices() {
    try {
        int deviceCount = esp_ble_get_bond_device_num();
        Serial.printf("Bonded devices count: %d\n", deviceCount);
        
        if (deviceCount > 0) {
            esp_ble_bond_dev_t* deviceList = (esp_ble_bond_dev_t*)malloc(sizeof(esp_ble_bond_dev_t) * deviceCount);
            if (deviceList && esp_ble_get_bond_device_list(&deviceCount, deviceList) == ESP_OK) {
                for (int i = 0; i < deviceCount; i++) {
                    BLEAddress address(deviceList[i].bd_addr);
                    Serial.printf("Device %d: %s\n", i, address.toString().c_str());
                }
            }
            if (deviceList) free(deviceList);
        }
    } catch (...) {
        Serial.println("⚠️ Failed to print bonded devices");
    }
}

void SmartPaddleBLEServer::removeAllBondedDevices() {
    try {
        int deviceCount = esp_ble_get_bond_device_num();
        
        if (deviceCount > 0) {
            esp_ble_bond_dev_t* deviceList = (esp_ble_bond_dev_t*)malloc(sizeof(esp_ble_bond_dev_t) * deviceCount);
            if (deviceList && esp_ble_get_bond_device_list(&deviceCount, deviceList) == ESP_OK) {
                for (int i = 0; i < deviceCount; i++) {
                    esp_ble_remove_bond_device(deviceList[i].bd_addr);
                }
            }
            if (deviceList) free(deviceList);
            Serial.println("All bonded devices removed");
        }
    } catch (...) {
        Serial.println("⚠️ Failed to remove bonded devices");
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
    try {
        int deviceCount = esp_ble_get_bond_device_num();
        bool found = false;
        
        if (deviceCount > 0) {
            esp_ble_bond_dev_t* deviceList = (esp_ble_bond_dev_t*)malloc(sizeof(esp_ble_bond_dev_t) * deviceCount);
            if (deviceList && esp_ble_get_bond_device_list(&deviceCount, deviceList) == ESP_OK) {
                for (int i = 0; i < deviceCount; i++) {
                    BLEAddress bondedAddress(deviceList[i].bd_addr);
                    if (address.equals(bondedAddress)) {
                        found = true;
                        break;
                    }
                }
            }
            if (deviceList) free(deviceList);
        }
        
        return found;
    } catch (...) {
        Serial.println("⚠️ Failed to check if device is bonded");
        return false;
    }
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
//            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if(imuQueue.receive(imuDataStruct,0)) {
            imuCharacteristic->setValue((uint8_t*)&imuDataStruct, sizeof(IMUData));
            imuCharacteristic->notify();
//            vTaskDelay(pdMS_TO_TICKS(1));
        }


        if(orientationQueue.receive(orientationData,0)) {
            orientationCharacteristic->setValue((uint8_t*)&orientationData, sizeof(OrientationData));
            orientationCharacteristic->notify();
//            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (send_specs) {
            if (millis() > time_to_send_specs) {
                serial->sendString(SP_MessageProcessor::createSpecsMessage(specs));
                send_specs = false;
            }
        }
        if (send_paddle_orientation) {
            if (millis() > time_to_send_paddle_orientation) {
                serial->sendString(SP_MessageProcessor::createBladeOrientationMessage(bladeOrientation));
                send_paddle_orientation = false;
            }
        }
    }
    if (connected()&&serial) serial->update();

}
/*
void SmartPaddleBLEServer::sendPaddleOrientation(){
    if (connected()&&serial) {
        serial->sendString(SP_MessageProcessor::createBladeOrientationMessage(bladeOrientation));
    }
}

void SmartPaddleBLEServer::sendSpecs(){
    if (connected()&&serial) {
        serial->sendString(SP_MessageProcessor::createSpecsMessage(specs));
    }
}
    */

void SmartPaddleBLEServer::shutdown() {
    logStream->println("Shutdown Paddle");
    for (int i = 0; i < eventHandlerCount; i++) 
        eventHandler[i]->onShutdown(this);
}

IMUData SmartPaddleBLEServer::getIMUData(){
    IMUData data;
    imu->getData(data);
    return data;
}

loadData SmartPaddleBLEServer::getLoadData(){
    return lastLoadData;
}

OrientationData SmartPaddleBLEServer::getOrientationData(){
    OrientationData data;
    imu->getOrientation(data);
    return data;
}

void SmartPaddleBLEServer::calibrateBladeAngle(BladeSideType blade_side) {
    if (!imu) return;

    // Получаем текущий кватернион ориентации
    IMUData data;
    imu->getData(data);
    SP_Math::Quaternion q(data.q0, data.q1, data.q2, data.q3);

    // Вектор вертикально вверх в глобальной системе координат
    SP_Math::Vector global_up(0.0f, 0.0f, 1.0f);

    // Преобразуем в локальную систему координат устройства
    SP_Math::Vector local_up = q.conjugate().rotate(global_up);

    // Проекция вектора на плоскость XZ локальной системы (игнорируем Y компоненту)

    if (specs.axisDirection==Y_AXIS_LEFT || specs.axisDirection==Y_AXIS_RIGHT) {
        local_up.y() = 0;
    }
    if (specs.axisDirection==Z_AXIS_LEFT || specs.axisDirection==Z_AXIS_RIGHT) {
        local_up.z() = 0;
    }
    if (specs.axisDirection==X_AXIS_LEFT || specs.axisDirection==X_AXIS_RIGHT) {
        local_up.x() = 0;
    }
    local_up.normalize();

    
    // Вычисляем угол между проекцией и осью Z в локальной системе координат
    float blade_angle = 0;
    if (specs.axisDirection==Y_AXIS_LEFT || specs.axisDirection==Y_AXIS_RIGHT) {
        blade_angle = atan2(local_up.x(), local_up.z());
    }
    if (specs.axisDirection==Z_AXIS_LEFT || specs.axisDirection==Z_AXIS_RIGHT) {
        blade_angle = atan2(local_up.x(), local_up.y());
    }
    if (specs.axisDirection==X_AXIS_LEFT || specs.axisDirection==X_AXIS_RIGHT) {
        blade_angle = atan2(local_up.y(), local_up.z());
    }

    #if 0
        logStream->printf("Calibrating %s blade angle\n", 
            blade_side == RIGHT_BLADE ? "right" : "left");
        logStream->printf("Global normal vector: %.3f, %.3f, %.3f\n", 
            local_up.x(), local_up.y(), local_up.z());
        logStream->printf("Blade angle around Y axis: %.1f degrees\n", 
            blade_angle * 180.0f / M_PI);

    #endif

    // Сохраняем угол
    if (blade_side == RIGHT_BLADE) {
        bladeOrientation.rightBladeAngle = blade_angle;
        bladeOrientation.rightBladeVector[0] = local_up.x();
        bladeOrientation.rightBladeVector[1] = local_up.y();
        bladeOrientation.rightBladeVector[2] = local_up.z();
    } else if (blade_side == LEFT_BLADE) {
        bladeOrientation.leftBladeAngle = blade_angle;
        bladeOrientation.leftBladeVector[0] = local_up.x();
        bladeOrientation.leftBladeVector[1] = local_up.y();
        bladeOrientation.leftBladeVector[2] = local_up.z();
    } else {
        Serial.printf("Calibrate blade angle command with no side\n");
        bladeOrientation.rightBladeAngle = blade_angle;
        bladeOrientation.rightBladeVector[0] = local_up.x();
        bladeOrientation.rightBladeVector[1] = local_up.y();
        bladeOrientation.rightBladeVector[2] = local_up.z();        
        bladeOrientation.leftBladeAngle = blade_angle;
        bladeOrientation.leftBladeVector[0] = local_up.x();
        bladeOrientation.leftBladeVector[1] = local_up.y();
        bladeOrientation.leftBladeVector[2] = local_up.z();
    }

    // Сохраняем в память
    Preferences prefs;
    prefs.begin(prefsName.c_str(), false);
    if (blade_side == RIGHT_BLADE) {
        prefs.putFloat("rightBladeAngle", blade_angle);
        prefs.putFloat("rightBVectorX", local_up.x());
        prefs.putFloat("rightBVectorY", local_up.y());
        prefs.putFloat("rightBVectorZ", local_up.z());
    } else if (blade_side == LEFT_BLADE) {
        prefs.putFloat("leftBladeAngle", blade_angle);
        prefs.putFloat("leftBVectorX", local_up.x());
        prefs.putFloat("leftBVectorY", local_up.y());
        prefs.putFloat("leftBVectorZ", local_up.z());
    }
    prefs.end();
    sendPaddleOrientation(0);
}

bool SmartPaddleBLEServer::loadBladeOrientation(){
    Preferences prefs;
    prefs.begin(prefsName.c_str(), true);
    // YAxisDirection теперь в specs.axisDirection, а не здесь
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

void SmartPaddleBLEServer::SetAxisDirection(signed char direction, bool save) {
    // YAxisDirection теперь хранится в specs.axisDirection
    specs.axisDirection = (AxisDirection)direction;
    if (save) {
        Preferences prefs;
        prefs.begin(prefsName.c_str(), false);
        prefs.putInt("axisDirection", (int)specs.axisDirection);
        prefs.end();
    }
}

void SmartPaddleBLEServer::saveSpecs(){
    
    Preferences prefs;
    if (!prefs.begin(prefsName.c_str(), false)) {
        Serial.println("⚠️ Failed to open preferences for saving specs");
        return;
    }
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
    prefs.putInt("axisDirection", (int)specs.axisDirection);  // Сохраняем направление оси IMU
    prefs.end();
    Serial.println("Specs saved");
    Serial.printf("paddleType: %d\n", specs.paddleType);
    Serial.printf("length: %.2f\n", specs.length);
    Serial.printf("imuDistance: %.2f\n", specs.imuDistance);
    Serial.printf("bladeWeight: %.2f\n", specs.bladeWeight);
    Serial.printf("bladeCenter: %.2f\n", specs.bladeCenter);
    Serial.printf("bladeMomentInertia: %.2f\n", specs.bladeMomentInertia);
    Serial.printf("firmwareVersion: %d\n", specs.firmwareVersion);
    Serial.printf("paddleModel: %s\n", specs.paddleModel.c_str());
    Serial.printf("hasLeftBlade: %d\n", specs.hasLeftBlade);
    Serial.printf("hasRightBlade: %d\n", specs.hasRightBlade);
    Serial.printf("imuFrequency: %d\n", specs.imuFrequency);
    Serial.printf("axisDirection: %d\n", specs.axisDirection);
}

bool SmartPaddleBLEServer::loadSpecs(){
    Serial.println("Loading specs...");
    Preferences prefs;
    if (!prefs.begin(prefsName.c_str(), true)) {
        Serial.println("⚠️ Failed to open preferences for loading specs");
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
    specs.axisDirection = (AxisDirection)prefs.getInt("axisDirection", (int)Y_AXIS_RIGHT);  // Загружаем направление оси IMU
    prefs.end();
    Serial.println("Specs loaded");
    Serial.printf("paddleType: %d\n", specs.paddleType);
    Serial.printf("length: %.2f\n", specs.length);
    Serial.printf("imuDistance: %.2f\n", specs.imuDistance);
    Serial.printf("bladeWeight: %.2f\n", specs.bladeWeight);
    Serial.printf("bladeCenter: %.2f\n", specs.bladeCenter);
    Serial.printf("bladeMomentInertia: %.2f\n", specs.bladeMomentInertia);
    Serial.printf("firmwareVersion: %d\n", specs.firmwareVersion);
    Serial.printf("paddleModel: %s\n", specs.paddleModel.c_str());
    Serial.printf("hasLeftBlade: %d\n", specs.hasLeftBlade);
    Serial.printf("hasRightBlade: %d\n", specs.hasRightBlade);
    Serial.printf("imuFrequency: %d\n", specs.imuFrequency);
    Serial.printf("axisDirection: %d\n", specs.axisDirection);
    return true;
}

void SmartPaddleBLEServer::startTasks() {
    SPServerRTOS::startTasks(this);
}




