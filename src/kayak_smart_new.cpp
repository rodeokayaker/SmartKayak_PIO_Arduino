#include "SmartPaddleClient.h"

#include "Wire.h"
#include "esp_log.h"
#include "Peripherals.h"
#include "SmartKayak.h"

#include "SPI.h"
#include "SD.h"
#include "AmperikaCRLog.h"
#include "ImuBNO08X.h"

#include "ChinaMotor.h"
#include "GFXKayakDisplay.h"
#include "CustomILI9341.h"
#include "../lib/Core/Types.h"
#include "LoadCellHX711.h"


#define INCLUDE_vTaskDelayUntil 1

// Define pins
#define IMU_INTERRUPT_PIN IMU_INTA
#define IMU_I2C_ADDRESS 0
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
#define PROCESS_STACK_SIZE 4096  // Увеличено для предотвращения переполнения стека
#define PROCESS_FREQUENCY 100  // Частота обработки данных в Гц
#define IMU_FREQUENCY 100
#define LOG_FREQUENCY 100
#define LOAD_CELL_FREQUENCY -1
#define VIZUALIZE_FREQUENCY 30

static bool log_paddle = false;
static bool SDCardReady = false;


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


SmartPaddleBLEClient paddle("SmartKayak 1.1");
ImuBNO08X imu_bno08x("imu_bno08x");
SmartKayak kayak("SmartKayak1");
PowerButton powerButton(BUTTON1_PIN);
AmperikaCRLog SDlog(SD_CS, SD_SCK, SD_MISO, SD_MOSI);
ChinaMotor motor(MOTOR_PWM);
IIMUSensor* imu_sensor = &imu_bno08x;

Arduino_ESP32SPI bus(TFT_DC, TFT_CS, TFT_SCLK, TFT_MOSI, TFT_MISO);
CustomILI9341 gfx(&bus, TFT_RST, 0, false);
GFXKayakDisplay gfxKayakDisplay(&gfx, TFT_BL);

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
        if (!paddle.connected()) {
            paddle.startPairing();
            return;
        }
//        logMode = (LogMode)((((int)logMode+1) % nLogModes));
//        SD_Logger->StopLog();
//        logStarted = false;
//        startNewLog();
//        logStarted = true;
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
        kayak.setPredictorMode(kayak.getPredictorMode() == 1 ? 0 : 1);
        Serial.printf("Predictor mode: %d\n", kayak.getPredictorMode());
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


void defragmentMemory() {
    Serial.println("🧹 Starting memory defragmentation...");
    
    uint32_t freeBefore = ESP.getFreeHeap();
    uint32_t maxAllocBefore = ESP.getMaxAllocHeap();
    
    // 1. Принудительная сборка мусора через malloc/free
    Serial.println("Step 1: Forcing garbage collection...");
    void* tempPtr = malloc(1024);
    if (tempPtr) {
        free(tempPtr);
        tempPtr = malloc(2048);
        if (tempPtr) {
            free(tempPtr);
        }
    }
    
    // 2. Очистка BLE буферов если возможно
    if (paddle.connected()) {
        Serial.println("Step 2: Clearing BLE buffers...");
        // Очистка внутренних буферов paddle
        // paddle.clearBuffers(); // Если такая функция есть
    }
    
    // 3. Принудительная пауза для завершения операций
    Serial.println("Step 3: Finalizing operations...");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 4. Дополнительная очистка через создание/удаление объектов
    Serial.println("Step 4: Memory consolidation...");
    {
        // Создаем временные объекты для принудительной дефрагментации
        char tempBuffer[512];
        memset(tempBuffer, 0, sizeof(tempBuffer));
        
        // Дополнительная очистка через множественные malloc/free
        void* ptrs[10];
        for (int i = 0; i < 10; i++) {
            ptrs[i] = malloc(256);
            if (ptrs[i]) {
                memset(ptrs[i], 0, 256);
            }
        }
        
        // Освобождаем в обратном порядке для лучшей дефрагментации
        for (int i = 9; i >= 0; i--) {
            if (ptrs[i]) {
                free(ptrs[i]);
            }
        }
        
        // Объект автоматически уничтожается при выходе из блока
    }
    
    // 5. Финальная пауза
    vTaskDelay(pdMS_TO_TICKS(50));
    
    uint32_t freeAfter = ESP.getFreeHeap();
    uint32_t maxAllocAfter = ESP.getMaxAllocHeap();
    
    Serial.printf("Defragmentation results:\n");
    Serial.printf("Free heap: %d -> %d bytes (+%d)\n", freeBefore, freeAfter, freeAfter - freeBefore);
    Serial.printf("Max alloc: %d -> %d bytes (+%d)\n", maxAllocBefore, maxAllocAfter, maxAllocAfter - maxAllocBefore);
    
    float fragmentationBefore = (1.0 - (float)maxAllocBefore / freeBefore) * 100;
    float fragmentationAfter = (1.0 - (float)maxAllocAfter / freeAfter) * 100;
    Serial.printf("Fragmentation: %.1f%% -> %.1f%%\n", fragmentationBefore, fragmentationAfter);
    Serial.println("✅ Defragmentation completed\n");
}

void printMemoryInfo(const char* stage) {
    Serial.printf("\n=== MEMORY INFO: %s ===\n", stage);
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("Min free heap: %d bytes\n", ESP.getMinFreeHeap());
    Serial.printf("Max alloc heap: %d bytes\n", ESP.getMaxAllocHeap());
    Serial.printf("Heap size: %d bytes\n", ESP.getHeapSize());
    Serial.printf("PSRAM size: %d bytes\n", ESP.getPsramSize());
    Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());
    
    // Расчет фрагментации памяти
    uint32_t freeHeap = ESP.getFreeHeap();
    uint32_t maxAlloc = ESP.getMaxAllocHeap();
    float fragmentation = (1.0 - (float)maxAlloc / freeHeap) * 100;
    Serial.printf("Memory fragmentation: %.1f%%\n", fragmentation);
    
    // Информация о стеках задач
    Serial.printf("\nTask stack info:\n");
    Serial.printf("PROCESS_STACK_SIZE: %d bytes\n", PROCESS_STACK_SIZE);
    Serial.printf("SerialCmd stack: 2048 bytes\n");
    Serial.printf("Log stack: 6144 bytes\n");
    Serial.printf("Vizualize stack: 2048 bytes\n");
    Serial.printf("LoadCell stack: 2048 bytes\n");
    Serial.printf("Total task stacks: %d bytes\n", PROCESS_STACK_SIZE + 2048*3 + 6144);  // 3 задачи по 2048 + Log 6144
    
    // Расчет статической памяти
    uint32_t staticMemory = ESP.getHeapSize() - ESP.getFreeHeap();
    Serial.printf("Static memory used: %d bytes\n", staticMemory);
    Serial.printf("================================\n\n");
}


void processDataTask(void *pvParameters) {
//    Serial.println("=== PROCESS TASK STARTED ===");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/PROCESS_FREQUENCY);
    uint32_t lastMemoryCheck = 0;
    uint32_t updateCount = 0;

    while(1) {
        
            if (powerButton.getMotorDebugMode()) {
                debugScenario.update();
                motor.runRaw(currentForce);
            } else {
                kayak.update();
            }

        updateCount++;

            // Каждые 30 секунд выводим подробную статистику
/*        if (updateCount % 1000 == 0) {
            printMemoryInfo("RUNTIME");
        }
      */  
        // Проверка памяти каждую секунду для быстрого обнаружения утечек
        if (millis() - lastMemoryCheck > 1000) {
            uint32_t freeHeap = ESP.getFreeHeap();
            
 
            
            lastMemoryCheck = millis();
            
/*            // Критический уровень памяти - принудительная очистка
            if (freeHeap < 20000) {
                Serial.printf("⚠️ LOW MEMORY: %d bytes - triggering defragmentation\n", freeHeap);
                defragmentMemory();
            }
            
            // Критический уровень - перезагрузка
            if (freeHeap < 10000) {
                Serial.println("❌ CRITICAL MEMORY! Restarting...");
                delay(1000);
                ESP.restart();
            }*/
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
#define CMD_MEMORY_DEFRAG "defrag"
#define CMD_MEMORY_INFO "meminfo"
#define CMD_MEMORY_CLEAN "clean"


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
        Serial.println("defrag - Defragment memory");
        Serial.println("meminfo - Show memory information");
        Serial.println("clean - Force memory cleanup");
        Serial.println("help          - Show this help");
    }
    else if(strcmp(cmd, CMD_STATUS) == 0) {
        Serial.printf("Connection status: %s\n", 
                     paddle.connected() ? "Connected" : "Disconnected");
        
        
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
    else if(strcmp(cmd, CMD_MEMORY_DEFRAG) == 0) {
        Serial.println("Starting manual memory defragmentation...");
        defragmentMemory();
    }
    else if(strcmp(cmd, CMD_MEMORY_INFO) == 0) {
        printMemoryInfo("MANUAL CHECK");
    }
    else if(strcmp(cmd, CMD_MEMORY_CLEAN) == 0) {
        Serial.println("🧽 Starting aggressive memory cleanup...");
        
        // Агрессивная очистка памяти
        uint32_t freeBefore = ESP.getFreeHeap();
        
        // 1. Множественные malloc/free для принудительной дефрагментации
        void* ptrs[20];
        for (int i = 0; i < 20; i++) {
            ptrs[i] = malloc(512);
            if (ptrs[i]) {
                memset(ptrs[i], 0, 512);
            }
        }
        
        // Освобождаем в случайном порядке для максимальной дефрагментации
        for (int i = 19; i >= 0; i--) {
            if (ptrs[i]) {
                free(ptrs[i]);
            }
        }
        
        // 2. Дополнительная очистка
        vTaskDelay(pdMS_TO_TICKS(200));
        
        uint32_t freeAfter = ESP.getFreeHeap();
        Serial.printf("Memory cleanup: %d -> %d bytes (+%d)\n", 
                     freeBefore, freeAfter, freeAfter - freeBefore);
        Serial.println("✅ Memory cleanup completed");
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
        while(Serial.available()) {
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
        vTaskDelay(pdMS_TO_TICKS(50));
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
//        Serial.printf("Load: %d, %d\n", ld.forceL, ld.forceR);

    }
    void onConnect(SmartPaddle* paddle) override {


    }

    void onDisconnect(SmartPaddle* paddle) override {


    }

    void onUpdateBladeAngle(BladeOrientation& bladeOrientation, SmartPaddle* paddle) override {
        PaddleSpecs specs = paddle->getSpecs();
 /*       specs.imuDistance = - 0.18f;
        specs.length = 1.65f *2;
        specs.bladeWeight = 0.20f;
        specs.bladeCenter = 0.17f;
        specs.bladeMomentInertia = 0.01;
        specs.imuFrequency = 100;
        specs.firmwareVersion = 1.2;
        specs.paddleModel = "SUP-175";
        specs.hasLeftBlade = true;
        specs.hasRightBlade = false;
        specs.axisDirection = Y_AXIS_RIGHT;
        specs.axisDirectionSign = 1;
        specs.imuFrequency = 100;


        specs.paddleType = PaddleType::TWO_BLADES;
        specs.length = 2.2f;
        specs.imuDistance = 0.00f;
        specs.bladeWeight = 0.3f;
        specs.bladeCenter = 0.20f;
        specs.bladeMomentInertia = 0.01;
    
        specs.firmwareVersion = 1.1;
        specs.paddleModel = "RST-220-KM";
        specs.hasLeftBlade = true;
        specs.hasRightBlade = true;
        specs.imuFrequency = 100;
    
        specs.axisDirection = Y_AXIS_LEFT;
    
        paddle->setSpecs(specs, true);*/
    }
    
};

PaddleEventHandler paddleEventHandler;

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
    while (!Serial && (millis() < 2000)) 
    {
        delay(10);
    }
    
    // Детальная проверка памяти на каждом этапе
//    printMemoryInfo("INITIAL");
    
    Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setClock(400000);
    // Инициализация дисплеев
    motor.begin();

    // Инициализация дисплея

    kayakDisplay = &gfxKayakDisplay;
    gfxKayakDisplay.setKayak(&kayak);
    kayakDisplay->setLogSwitch(&logButton);
    kayakDisplay->setMotorSwitch(&powerButton);
    kayakDisplay->setMotorDriver(&motor);
    kayakDisplay->begin();

    Serial.printf("PSRAM: %d\n", ESP.getPsramSize());

   
    SDCardReady = SDlog.begin("SD_LOG");

    SD_Logger = &SDlog;


    if (loadCell.begin()) {
    }


    Serial.println("Starting paddle initialization...");
    paddle.setEventHandler(&paddleEventHandler);
    
    paddle.begin("SmartKayak 1.0");
    printMemoryInfo("AFTER PADDLE INIT");
    Serial.println("Paddle initialized");
    
    Serial.println("Starting IMU initialization...");
    imu_bno08x.begin(&Wire,IMU_I2C_ADDRESS, IMU_INTERRUPT_PIN, IMU_RESET_PIN);
    imu_bno08x.setOrientationFrequency(IMU_FREQUENCY);
    Serial.println("IMU initialized");
    
    Serial.println("Starting kayak initialization...");
    KayakSpecs specs;
    specs.axisDirection = Y_AXIS_FORWARD;
    kayak.setSpecs(specs, true);
    kayak.setPaddle(&paddle);
    kayak.setDisplay(kayakDisplay);
    kayak.setIMU(imu_sensor, IMU_FREQUENCY);
    kayak.setModeSwitch(&powerButton);
    kayak.setMotorDriver(&motor);
    kayak.begin();
    kayak.setPredictorMode(1);
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
    
//    printMemoryInfo("BEFORE TASK CREATION");
    
    // Создание задач с проверкой успешности
    BaseType_t result = xTaskCreatePinnedToCore(
        processDataTask,
        "Process",
        PROCESS_STACK_SIZE,
        NULL,
        1,
        NULL,
        1  // Обработка на ядре 2
    );
    
    if (result != pdPASS) {
        Serial.println("ERROR: Failed to create Process task!");
        return;
    }

    result = xTaskCreatePinnedToCore(
        serialCommandTask,
        "SerialCmd",
        2048,
        NULL,
        1,
        NULL,
        1  // Команды на ядре 1
    );
    
    if (result != pdPASS) {
        Serial.println("ERROR: Failed to create SerialCmd task!");
        return;
    }

    result = xTaskCreatePinnedToCore(
        logTask,
        "Log",
        4096,  // Увеличено с 2048 до 4096 для предотвращения переполнения стека при SD операциях и printf
        NULL,
        1,
        NULL,
        1  
    );
    
    if (result != pdPASS) {
        Serial.println("ERROR: Failed to create Log task!");
        return;
    }

/*    result = xTaskCreatePinnedToCore(
        vizualizeSerialTask,
        "Vizualize",
        2048,
        NULL,
        1,
        NULL,
        1  
    );*/
    
    if (result != pdPASS) {
        Serial.println("ERROR: Failed to create Vizualize task!");
        return;
    }
    
    result = xTaskCreatePinnedToCore(
        loadCellTask,
        "LoadCell",
        2048,
        NULL,
        1,
        &loadCellTaskHandle,
        1  
    );
    
    if (result != pdPASS) {
        Serial.println("ERROR: Failed to create LoadCell task!");
        return;
    }        
    #if LOAD_CELL_FREQUENCY <= 0
    attachInterrupt(digitalPinToInterrupt(loadCell.getDRDYPin()), loadCellDataReady, FALLING);
    #endif


    paddle.startTasks();
    kayak.startTasks();
    gfxKayakDisplay.startTasks();
    
//    printMemoryInfo("AFTER ALL INITIALIZATION");

}

void loop() {
    vTaskDelete(NULL);
} 