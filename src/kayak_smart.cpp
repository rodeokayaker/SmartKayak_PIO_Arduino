#include "SmartPaddleClient.h"

#include "Wire.h"
#include "esp_log.h"
#include "Peripherals.h"
#include "SmartKayak.h"

#include "SPI.h"
#include "SD.h"
#include "AmperikaCRLog.h"
#include <ESP32Servo.h>
//#include "IMUSensor_BNO055.h"
#include "ImuBNO08X.h"
//#include "IMUSensor_BNO085.h"
//#include "IMUSensor_GY87.h"
//#include "IMUSensor_ICM20948.h"

#include "ChinaMotor.h"
#include "TFTSmallDisplay.h"
#include "SP_Types.h"
#include "LoadCellHX711.h"


#define INCLUDE_vTaskDelayUntil 1

// Define pins
#define IMU_INTERRUPT_PIN IMU_INTA
//#define IMU_INTERRUPT_PIN -1
//#define IMU_I2C_ADDRESS 0x29
#define IMU_I2C_ADDRESS 0x4B
#define IMU_RESET_PIN IMU_RST

// Define PWM output and duty Cycle mode
#define PWM1_Ch    0
#define PWM1_Res   8
#define PWM1_Freq  1000
#define PWM1_HIGH  3.0 
#define PWM1_MED   2.0
#define PWM1_LOW   1.0


KayakDisplay* kayakDisplay=nullptr;
ILogInterface* SD_Logger=nullptr;


// Текущее значение
volatile int currentForce = 1500;

// define initial power state, mode and force threshold
const int FORCE_THRESHOLD = 20; // минимальный уровень чувствительности тензодатчиков
const int MAX_FORCE = 100; // максимальное усилие на весле
int PWM1_DutyCycle = 0;

LoadCellHX711 loadCell("LOAD", HX711_DOUT_PIN, HX711_SCK_PIN);
TaskHandle_t loadCellTaskHandle = NULL;
int loadCellValue = 0;

// FreeRTOS определения
#define PROCESS_STACK_SIZE 4192
#define PROCESS_FREQUENCY 100  // Частота обработки данных в Гц
#define IMU_FREQUENCY 100
#define LOG_FREQUENCY 100
#define LOAD_CELL_FREQUENCY -1
#define VIZUALIZE_FREQUENCY 30

static bool log_paddle = false;
static bool SDCardReady = false;


// Структуры для хранения последних полученных данных
struct {
    loadData load;
    IMUData imu;
    OrientationData orientation;
    BladeData blade;
    PaddleStatus status;
    bool loadValid = false;
    bool imuValid = false;
    bool orientationValid = false;
    bool bladeValid = false;
    bool statusValid = false;
} paddleData;


#define N_MODES_SCENARIO 12
class DebugScenario {
    private:
    int *currentForce;
    bool running;
    struct modeScenario {
        int force;
        uint32_t time;
    } scenario[N_MODES_SCENARIO];
    int mode;
    int nModes;
    uint32_t nextSwitch;
    
    public:
    DebugScenario(int *currentF): currentForce(currentF), running(false),nModes(N_MODES_SCENARIO), mode(0), nextSwitch(0) 
    {
        scenario[0]={1500, 10000};
        scenario[1]={2000, 10000};
        scenario[2]={1500, 10000};
        scenario[3]={1800, 10000};
        scenario[4]={1500, 10000};
        scenario[5]={1700, 10000};
        scenario[6]={1500, 10000};
        scenario[7]={1600, 10000};
        scenario[8]={1700, 10000};
        scenario[9]={1800, 10000};
        scenario[10]={1900, 10000};
        scenario[11]={2000, 10000};
    };
    void start()
    {
        mode = 0;
        nextSwitch = millis() + scenario[mode].time;
        running = true;
        *currentForce = scenario[mode].force;
    }
    void stop(){
        running = false;
        *currentForce = 1500;
    }
    void update(){
        if (running) {
            if (millis() >= nextSwitch) {
                mode = (mode + 1);
                if (mode >= nModes) {
                    mode = 0;
                    stop();
                    return;
                }
                nextSwitch = millis() + scenario[mode].time;
                *currentForce = scenario[mode].force;
            }
        }
    }
    bool isRunning() { return running; }
};

DebugScenario debugScenario((int*)&currentForce);

void switchLogToDebugMode();

class PowerButton : public ButtonDriver, public IModeSwitch {
private:

    MotorPowerMode currentMode;
    bool MotorDebugMode;
public:
    PowerButton(int pin) : 
        ButtonDriver(pin), 
        currentMode(MOTOR_OFF), 

        MotorDebugMode(false) {}

    void begin(){
        ButtonDriver::begin();

    }

    bool debugMotorChandeForce(int increment){
        bool changed = false;
        int newForce = currentForce + increment;
        
        // Проверка границ
        if (newForce >= 1000 && newForce <= 2000) {
            currentForce = newForce;
            changed=true;
        }
        return changed;
    }


    void onPress() override{};
    void onRelease() override;
    void onLongPress() override {
        MotorDebugMode = !MotorDebugMode;
        if (MotorDebugMode) {
            kayakDisplay->switchDebugScreen(true);
            currentForce = 1500;
            debugMotorChandeForce(0);
            switchLogToDebugMode();
            kayakDisplay->setDebugData(currentForce, loadCell.getRawForce(), debugScenario.isRunning());
        }
        else {
            kayakDisplay->switchDebugScreen(false);
        }
    }
    MotorPowerMode getMode() { if (MotorDebugMode) return MOTOR_DEBUG; else return currentMode; }
    bool getMotorDebugMode() { return MotorDebugMode; }
    
};


void PowerButton::onRelease() {
    if (isLongPressed()) {
        return;
    }

    if (getMotorDebugMode()) {
        debugMotorChandeForce(-50);
        kayakDisplay->setDebugData(currentForce, loadCell.getRawForce(), debugScenario.isRunning());
        return;
    }

    currentMode = (MotorPowerMode)((((int)currentMode+1) % 4));


}

// Глобальные объекты

SmartPaddleBLEClient paddle("Paddle_1");
//IMUSensor_BNO055 imu_bno055("imu_bno055_2", IMU_I2C_ADDRESS, -1, &Serial);
//IMUSensor_BNO085 imu_bno085("imu_bno085");
ImuBNO08X imu_bno08x("imu_bno08x");
//IMUSensor_GY87 imu_bno055("imu_gy87", true, IMU_INTERRUPT_PIN);
//IMUSensor_ICM20948 imu_icm20948("imu_icm20948", ICM20948_I2C_ADDRESS, IMU_INTERRUPT_PIN, &Serial);
SmartKayak kayak;
PowerButton powerButton(BUTTON1_PIN);
AmperikaCRLog SDlog(SD_CS, SD_SCK, SD_MISO, SD_MOSI);
ChinaMotor motor(MOTOR_PWM);
ImuSensor* imu_sensor = &imu_bno08x;

class LogButton: public ButtonDriver, public ILogSwitch {
    private:
    BladeSideType side;
    LogMode logMode;
    bool logStarted = false;

    public:
    LogButton(int pin): ButtonDriver(pin), 
        side(BladeSideType::RIGHT_BLADE),
        logMode(LogMode::LOG_MODE_ALL),
        logStarted(false) {}
        
    void begin() {
        ButtonDriver::begin();
    }

    void startNewLog(){
        switch (logMode) {
            case LogMode::LOG_MODE_OFF:
                break;
            case LogMode::LOG_MODE_PADDLE_MAG:
                SD_Logger->StartLog("PMAG");
                break;
            case LogMode::LOG_MODE_KAYAK_MAG:
                SD_Logger->StartLog("KMAG");
                break;
            case LogMode::LOG_MODE_DEBUG:
                SD_Logger->StartLog("DBG");
                break;
            case LogMode::LOG_MODE_ALL:
                SD_Logger->StartLog("ALL");
                break;
        }    
    }

    void onRelease() override {
        if (isLongPressed()) {
            return;
        }
        if (powerButton.getMotorDebugMode()) {
            powerButton.debugMotorChandeForce(50);
            kayakDisplay->setDebugData(currentForce, loadCell.getRawForce(), debugScenario.isRunning());
            return;
        }

        if (logMode != LogMode::LOG_MODE_OFF) {
            if (!logStarted) {
                startNewLog();
            }
            logStarted = !logStarted;
            if (!logStarted) {
                vTaskDelay(pdMS_TO_TICKS(100));
                SD_Logger->StopLog();
            }
        }
    }

    void onLongPress() override {
        if (powerButton.getMotorDebugMode()) {
            if (!debugScenario.isRunning()) {
                debugScenario.start();
            } else {
                debugScenario.stop();
            }
            return;
        }
        logMode = (LogMode)((((int)logMode+1) % nLogModes));
        SD_Logger->StopLog();
        logStarted = false;
        startNewLog();
    }

    void switchToDebugMode() {
        if (logMode == LogMode::LOG_MODE_DEBUG) {
            if (!logStarted) {
                startNewLog();
                logStarted = true;
            }
        }
    }

    LogMode getLogMode() override { return logMode; }
    bool getLogStarted() override { return logStarted; }
    void onPress() override {}
};

LogButton logButton(BUTTON2_PIN);

void switchLogToDebugMode() {
    logButton.switchToDebugMode();
}

class DisplayButton: public ButtonDriver {
    private:
    bool displayMode;
    public:
    DisplayButton(int pin): ButtonDriver(pin), displayMode(false) {}
    void onRelease() override {
        if (isLongPressed()) {
            return;
        }

        Serial.printf("Display button pressed\n");        
//        String cmd = SP_MessageProcessor::createSendCalibrationDataCommand();
//        Serial.printf("Command: %s\n", cmd.c_str());
//        paddle.getSerial()->sendString(cmd);
        if (powerButton.getMotorDebugMode()) {
            powerButton.debugMotorChandeForce(-50);
            kayakDisplay->setDebugData(currentForce, loadCell.getRawForce(), debugScenario.isRunning());
            return;
        }
        Serial.printf("Out of display button pressed\n");
    }
    void onLongPress() override {
        if (powerButton.getMotorDebugMode()) {
            if (!debugScenario.isRunning()) {
                debugScenario.start();
            } else {
                debugScenario.stop();
            }
        }
        Serial.printf("Display button long pressed\n");
    }
    void onPress() override {
    }
};

DisplayButton displayButton(BUTTON3_PIN);



// Задача обработки данных с весла
void processDataTask(void *pvParameters) {
//    Serial.println("=== PROCESS TASK STARTED ===");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/PROCESS_FREQUENCY);

    while(1) {
        
            if (powerButton.getMotorDebugMode()) {
                debugScenario.update();
                motor.runRaw(currentForce);
            } else {
                kayak.update();
            }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void loadCellTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    #if LOAD_CELL_FREQUENCY > 0
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/LOAD_CELL_FREQUENCY);
    #endif
    int32_t lastCall = 0;
    while(1) {
        #if LOAD_CELL_FREQUENCY > 0
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        #else
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        #endif
        loadCell.read();
//        Serial.printf("Load cell value: %d dt: %d\n", loadCell.getRawForce(), millis() - lastCall);
        kayakDisplay->setDebugData(currentForce, loadCell.getRawForce(), debugScenario.isRunning());
        lastCall = millis();
    }
}

void logTask(void *pvParameters) {
    Serial.println("=== LOG TASK STARTED ===");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/LOG_FREQUENCY);

    if (!SD_Logger->Started()) {
        Serial.println("SD Logger not started, deleting log task");
        vTaskDelete(NULL);
        return;
    }

    while(1) {
        if (logButton.getLogMode() != LogMode::LOG_MODE_OFF) {
            if (logButton.getLogStarted()) {
                if (!SD_Logger->Opened()) {
                    SD_Logger->Open();
                }
                if (logButton.getLogMode() == LogMode::LOG_MODE_DEBUG) {
                    SD_Logger->printf("%d,%d,%d\n", millis(), loadCell.getRawForce(), currentForce);
                } else {
                    loadCellValue = loadCell.getRawForce();
                    kayak.logCall(SD_Logger, logButton.getLogMode(), &loadCellValue, powerButton.getMotorDebugMode()?(int*)&currentForce:nullptr);
                }
            } 
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void vizualizeSerialTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/VIZUALIZE_FREQUENCY);

    while(1) {

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Команды управления
#define CMD_HELP "help"
#define CMD_STATUS "status"
#define CMD_PAIR "pair"
#define CMD_IMU_CALIBRATE "calib_imu"
#define CMD_MAG_CALIBRATE "calib_mag"
#define CMD_PADDLE_IMU_CALIBRATE "calib_p_imu"
#define CMD_PADDLE_LOADS_CALIBRATE "calib_p_loads"
#define CMD_PADDLE_LOADS_CALIBRATE_LEFT "calib_p_left"
#define CMD_PADDLE_LOADS_CALIBRATE_RIGHT "calib_p_right"
#define CMD_PADDLE_COMPASS_CALIBRATE "calib_p_mag"
#define CMD_PADDLE_PAIR "paddle_pair"
#define CMD_PADDLE_SHUTDOWN "shutdown"
#define CMD_PADDLE_SEND_SPECS "send_specs"
#define CMD_PADDLE_NEXT_FILE "next_file"
#define CMD_PADDLE_SD_START "sd_start"
#define CMD_PADDLE_SD_STOP "sd_stop"
#define CMD_PADDLE_TARE_LOADS "tare_loads"
#define CMD_PADDLE_CALIBRATE_BLADE_ANGLE_LEFT "calib_blade_l"
#define CMD_PADDLE_CALIBRATE_BLADE_ANGLE_RIGHT "calib_blade_r"


// Обработка команд
void processCommand(const char* cmd) {
    if(strcmp(cmd, CMD_HELP) == 0) {
        Serial.println("Available commands:");
        Serial.println("pair          - Start pairing with paddle");
        Serial.println("status        - Show current paddle status");
        Serial.println("calib_imu - Start IMU calibration");
        Serial.println("calib_mag - Start Magnetometer calibration");
        Serial.println("calib_p_imu - Start Paddle IMU calibration");
        Serial.println("calib_p_loads - Start Paddle loads calibration");
        Serial.println("calib_p_left - Start Paddle left load calibration");
        Serial.println("calib_p_right - Start Paddle right load calibration");
        Serial.println("calib_p_mag - Start PaddleMagnetometer calibration");
        Serial.println("calib_blade_r - Start Paddle right blade angle calibration");
        Serial.println("calib_blade_l - Start Paddle left blade angle calibration");
        Serial.println("tare_loads - Tare all loads");
        Serial.println("paddle_pair - Start Paddle pairing");
        Serial.println("send_specs - Send paddle specifications");
        Serial.println("shutdown - Shutdown Paddle");
        Serial.println("sd_start - Start SD logging");
        Serial.println("sd_stop - Stop SD logging");
        Serial.println("next_file - SD open next file");
        Serial.println("help          - Show this help");
    }
    else if(strcmp(cmd, CMD_STATUS) == 0) {
        Serial.printf("Connection status: %s\n", 
                     paddle.connected() ? "Connected" : "Disconnected");
        
        if(paddleData.statusValid) {
            Serial.printf("Battery: %d%%\n", paddleData.status.batteryLevel);
            Serial.printf("Temperature: %d°C\n", paddleData.status.temperature);
        }
        
        PaddleSpecs specs = paddle.getSpecs();
        Serial.printf("Paddle ID: %s\n", specs.paddleID);
        Serial.printf("Blade type: %s\n", 
                     specs.paddleType == TWO_BLADES ? "Double" : "Single");
    }
    else if(strcmp(cmd, CMD_PAIR) == 0) {
        Serial.println("Starting pairing mode...");
        paddle.startPairing();
    } else if(strcmp(cmd, CMD_IMU_CALIBRATE) == 0) {
        imu_sensor->calibrate();
    }
    else if(strcmp(cmd, CMD_PADDLE_IMU_CALIBRATE) == 0) {
        paddle.calibrateIMU();
    }
    else if(strcmp(cmd, CMD_PADDLE_LOADS_CALIBRATE) == 0) {
        paddle.calibrateLoads(ALL_BLADES);
    }
    else if(strcmp(cmd, CMD_PADDLE_LOADS_CALIBRATE_LEFT) == 0) {
        paddle.calibrateLoads(LEFT_BLADE);
    }
    else if(strcmp(cmd, CMD_PADDLE_LOADS_CALIBRATE_RIGHT) == 0) {
        paddle.calibrateLoads(RIGHT_BLADE);
    }
    else if(strcmp(cmd, CMD_PADDLE_PAIR) == 0) {
        paddle.getSerial()->sendString(SP_MessageProcessor::createStartPairCommand());
    }
    else if(strcmp(cmd, CMD_PADDLE_SHUTDOWN) == 0) {
        paddle.getSerial()->sendString(SP_MessageProcessor::createShutdownCommand());
    }
    else if(strcmp(cmd, CMD_PADDLE_SD_START) == 0) {

    }
    else if(strcmp(cmd, CMD_PADDLE_SD_STOP) == 0) {

    }
    else if(strcmp(cmd, CMD_PADDLE_NEXT_FILE) == 0) {

    }
    else if(strcmp(cmd, CMD_PADDLE_SEND_SPECS) == 0) {
        if (paddle.getSerial()) {
            paddle.getSerial()->sendString(SP_MessageProcessor::createSendSpecsCommand());
        }
    }
    else if(strcmp(cmd, CMD_PADDLE_COMPASS_CALIBRATE) == 0) {
        if (paddle.getSerial()) {
            paddle.getSerial()->sendString(SP_MessageProcessor::createCalibrateCompassCommand());
        }
    }
    else if(strcmp(cmd, CMD_MAG_CALIBRATE) == 0) {
//        imu_sensor->calibrateCompass();
    }
    else if(strcmp(cmd, CMD_PADDLE_CALIBRATE_BLADE_ANGLE_LEFT) == 0) {
        paddle.calibrateBladeAngle(LEFT_BLADE);
    }
    else if(strcmp(cmd, CMD_PADDLE_CALIBRATE_BLADE_ANGLE_RIGHT) == 0) {
        paddle.calibrateBladeAngle(RIGHT_BLADE);
    }
    else if(strcmp(cmd, CMD_PADDLE_TARE_LOADS) == 0) {
        if (paddle.getSerial()) {
            paddle.getSerial()->sendString(SP_MessageProcessor::createTareLoadsCommand(ALL_BLADES));    
        }
    }
    else {
        Serial.println("Unknown command. Type 'help' for available commands.");
    }

    Serial.print("Smart Kayak > ");
}

// Задача обработки команд через серийный порт
void serialCommandTask(void *pvParameters) {
    char cmdBuffer[32];
    int cmdIndex = 0;
    
    while(1) {
        if(Serial.available()) {
            char c = Serial.read();
            Serial.print(c);

            if(c == '\n' || c == '\r') {
                if(cmdIndex > 0) {
                    cmdBuffer[cmdIndex] = '\0';
                    processCommand(cmdBuffer);
                    cmdIndex = 0;
                }
            } else if(cmdIndex < 31) {
                cmdBuffer[cmdIndex++] = c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

class PaddleEventHandler: public SP_EventHandler {
    private:
    float imu_freq;
    float load_freq;
    uint32_t last_imu_ts;
    uint32_t last_load_ts;
    public:
    PaddleEventHandler(): imu_freq(100), load_freq(10), last_imu_ts(0), last_load_ts(0) {}
    void onUpdateIMU(IMUData& imuData, SmartPaddle* paddle) override {
        uint32_t dt = imuData.timestamp - last_imu_ts;
        if (dt > 0) {
            imu_freq = 0.001 * (1000.0/dt) + 0.999 * imu_freq;
        } else {
            Serial.printf("IMU timestamp error: %d, %d\n", dt, imuData.timestamp);
        }
        last_imu_ts = imuData.timestamp;
//        Serial.printf("IMU Freq:  %.2f\n",imu_freq);
    }
    void onUpdateLoad(loadData& ld, SmartPaddle* paddle) override {
        last_load_ts = ld.timestamp;
//        Serial.printf("Load Freq:  %.2f\n",                     load_freq);
    }
    void onConnect(SmartPaddle* paddle) override {
        kayakDisplay->paddleConnected(true);

    }

    void onDisconnect(SmartPaddle* paddle) override {
        Serial.printf("DisconnectedON\n");
        kayakDisplay->paddleConnected(false);

    }
    
};

PaddleEventHandler paddleEventHandler;
TFTSmallDisplay kayakTFTDisplay;

/*
void setMagnitometerCalibration() {
    float offset[3] = {0, 0, 0};
    float scale[3] = {1, 1, 1};
    float SI[3] = {1, 1, 1};
    IMUCalibData imuCalibrationData = imu_sensor->getCalibrationData();
    imuCalibrationData.magOffset[0] = -5832.785182;
    imuCalibrationData.magOffset[1] = -4309.213456;
    imuCalibrationData.magOffset[2] = -288.716420;
    imuCalibrationData.magScale[0] = 0.164958;
    imuCalibrationData.magScale[1] = 0.164547;
    imuCalibrationData.magScale[2] = 0.173941;
    imuCalibrationData.magSI[0] = -0.003708;
    imuCalibrationData.magSI[1] = -0.004617;
    imuCalibrationData.magSI[2] = 0.002007;
    imu_sensor->setCalibrationData(imuCalibrationData,true);
}
    */

static void IRAM_ATTR loadCellDataReady() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(loadCellTaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    // Проверка памяти на каждом этапе
    Serial.printf("Initial free heap: %d\n", ESP.getFreeHeap());
    
    Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setClock(400000);
    // Инициализация дисплеев
    motor.begin();

    // Инициализация дисплея

    kayakDisplay = &kayakTFTDisplay;
    kayakDisplay->setLogSwitch(&logButton);
    kayakDisplay->setMotorSwitch(&powerButton);
    kayakDisplay->setMotorDriver(&motor);
    kayakDisplay->begin();
   
   SDCardReady = SDlog.begin("SD_LOG");

    SD_Logger = &SDlog;


    if (loadCell.begin()) {
    }


    Serial.println("Starting paddle initialization...");
    paddle.setEventHandler(&paddleEventHandler);
    
    paddle.begin("SmartKayak 1.0");
    Serial.println("Paddle initialized");
    
    Serial.println("Starting IMU initialization...");
    imu_bno08x.begin(&Wire,IMU_I2C_ADDRESS, IMU_INTERRUPT_PIN, IMU_RESET_PIN);
    imu_bno08x.setOrientationFrequency(IMU_FREQUENCY);
    Serial.println("IMU initialized");
    
    Serial.println("Starting kayak initialization...");
    kayak.setPaddle(&paddle);
    kayak.setDisplay(kayakDisplay);
    kayak.setIMU(imu_sensor, IMU_FREQUENCY);
    kayak.setModeSwitch(&powerButton);
    kayak.setMotorDriver(&motor);
    kayak.begin();
    Serial.println("Kayak initialized");
    
    Serial.println("Starting buttons initialization...");
    logButton.begin();
    powerButton.begin();
    displayButton.begin();
    Serial.println("Buttons initialized");



    Serial.println("Kayak Smart System Ready!");
    Serial.println("Type 'help' for available commands");
    Serial.print("Smart Kayak > ");
    Serial.println("Creating tasks...");
    
    Serial.printf("Before task creation: %d\n", ESP.getFreeHeap());
    
    // Создание задач
    BaseType_t result = xTaskCreatePinnedToCore(
        processDataTask,
        "Process",
        PROCESS_STACK_SIZE,
        NULL,
        2,
        NULL,
        1  // Обработка на ядре 2
    );

    
    xTaskCreatePinnedToCore(
        serialCommandTask,
        "SerialCmd",
        4096,
        NULL,
        1,
        NULL,
        1  // Команды на ядре 1
    );
 

    xTaskCreatePinnedToCore(
        logTask,
        "Log",
        4096,
        NULL,
        1,
        NULL,
        1  
    );

    xTaskCreatePinnedToCore(
        vizualizeSerialTask,
        "Vizualize",
        2048,
        NULL,
        1,
        NULL,
        1  
    );
        xTaskCreatePinnedToCore(
            loadCellTask,
            "LoadCell",
            2048,
            NULL,
            1,
            &loadCellTaskHandle,
            1  
        );        
    #if LOAD_CELL_FREQUENCY <= 0
    attachInterrupt(digitalPinToInterrupt(loadCell.getDRDYPin()), loadCellDataReady, FALLING);
    #endif


    paddle.startTasks();
    kayak.startTasks();
    kayakTFTDisplay.startTasks();

}

void loop() {
    vTaskDelete(NULL);
} 