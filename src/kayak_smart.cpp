#include "SmartPaddle.h"
#include <EEPROM.h>
#include "Wire.h"
#include "esp_log.h"
#include "IMUSensor_GY85.h"
#include "MadgwickAHRS.h"

#define INCLUDE_vTaskDelayUntil 1

// Адреса в EEPROM
constexpr int CALIB_IMU_FLAG_ADDR = 16;
constexpr int IMU_CALIB_ADDR = 20;
constexpr int TRUSTED_DEV_ADDR = 128;


// FreeRTOS определения
#define BLE_STACK_SIZE 4096
#define PROCESS_STACK_SIZE 4096
#define PROCESS_FREQUENCY 98  // Частота обработки данных в Гц
#define IMU_FREQUENCY 98
#define BLE_FREQUENCY 100

static bool log_paddle = false;

// Глобальные объекты
SmartPaddleBLEClient paddle(TRUSTED_DEV_ADDR);

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

class SmartKayak {
private:
    IIMU* imu;
    Madgwick filter;
    IMUData imu_data;


public:
    SmartKayak(IIMU* imu) : imu(imu) {};
    void begin(int frequency=IMU_FREQUENCY)
    {
        if (imu){ if (!imu->begin()) Serial.println("IMU init failed");} else {
            Serial.println("IMU not initialized");
        }
        filter.begin(frequency);
    };

    void updateIMU() {
        imu->getData(imu_data);
        filter.update(imu_data.ax, imu_data.ay, imu_data.az, imu_data.gx, imu_data.gy, imu_data.gz, imu_data.mx, imu_data.my, imu_data.mz);
    }

    OrientationData getOrientation() {
        return filter.getOrientation();
    }
};

ADXL345 accel;
ITG3200 gyro;
MechaQMC5883 qmc;

IMUSensor imu(accel, gyro, qmc, 
                       IMU_CALIB_ADDR,    // Адрес калибровки IMU
                       CALIB_IMU_FLAG_ADDR); // Адрес флага калибровки IMU
SmartKayak kayak(&imu);


// Задача обработки BLE
void bleTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/BLE_FREQUENCY);

    while(1) {
        paddle.updateBLE();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Задача обработки данных с весла
void processDataTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/PROCESS_FREQUENCY);
    
    while(1) {
        // Получение данных из очередей
        loadData load;
        IMUData imu;
        OrientationData orientation;
        BladeData blade;
        PaddleStatus status;
        
        if(paddle.getLoadData(load)) {
            paddleData.load = load;
            paddleData.loadValid = true;
            if(log_paddle) {
                Serial.printf("Load - L: %d, R: %d\n", 
                            load.forceL, load.forceR);
            }
        }
        
        if(paddle.getIMUData(imu)) {
            paddleData.imu = imu;
            paddleData.imuValid = true;
            if(log_paddle) {
                Serial.printf("IMU - Accel: %.2f,%.2f,%.2f Gyro: %.2f,%.2f,%.2f Mag: %.2f,%.2f,%.2f\n",
                            imu.ax, imu.ay, imu.az,
                            imu.gx, imu.gy, imu.gz,
                            imu.mx, imu.my, imu.mz);
            }
        }
        
        if(paddle.getOrientationData(orientation)) {
            paddleData.orientation = orientation;
            paddleData.orientationValid = true;
            if(log_paddle) {
                Serial.printf("Orientation - Q: %.2f,%.2f,%.2f,%.2f\n",
                            orientation.q0, orientation.q1, 
                            orientation.q2, orientation.q3);
            }
        }
        
        if(paddle.getBladeData(blade)) {
            paddleData.blade = blade;
            paddleData.bladeValid = true;
            if(log_paddle) {
                Serial.printf("Blade - Side: %d Force: %.2f\n",
                            blade.bladeSide, blade.force);
            }
        }
        
        if(paddle.getStatusData(status)) {
            paddleData.status = status;
            paddleData.statusValid = true;
            if(log_paddle) {
                Serial.printf("Status - Battery: %d%% Temp: %d°C\n",
                            status.batteryLevel, status.temperature);
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void imuTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/IMU_FREQUENCY);
    IMUData imu_data;

    while(1) {  
        kayak.updateIMU();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void 

// Команды управления
#define CMD_HELP "help"
#define CMD_STATUS "status"
#define CMD_PAIR "pair"
#define CMD_LOG_START "log_start"
#define CMD_LOG_STOP "log_stop"
#define CMD_IMU_CALIBRATE "calibrate_imu"
// Обработка команд
void processCommand(const char* cmd) {
    if(strcmp(cmd, CMD_HELP) == 0) {
        Serial.println("Available commands:");
        Serial.println("pair          - Start pairing with paddle");
        Serial.println("status        - Show current paddle status");
        Serial.println("log_start     - Start logging paddle data");
        Serial.println("log_stop      - Stop logging paddle data");
        Serial.println("calibrate_imu - Start IMU calibration");
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
        Serial.printf("Paddle ID: %08X\n", specs.PaddleID);
        Serial.printf("Blade type: %s\n", 
                     specs.bladeType == TWO_BLADES ? "Double" : "Single");
    }
    else if(strcmp(cmd, CMD_PAIR) == 0) {
        Serial.println("Starting pairing mode...");
        paddle.startPairing();
    }
    else if(strcmp(cmd, CMD_LOG_START) == 0) {
        log_paddle = true;
        Serial.println("Logging started");
    }
    else if(strcmp(cmd, CMD_LOG_STOP) == 0) {
        log_paddle = false;
        Serial.println("Logging stopped");
    } else if(strcmp(cmd, CMD_IMU_CALIBRATE) == 0) {
        imu.calibrate();
    }
    else {
        Serial.println("Unknown command. Type 'help' for available commands.");
    }
}

// Задача обработки команд через серийный порт
void serialCommandTask(void *pvParameters) {
    char cmdBuffer[32];
    int cmdIndex = 0;
    
    while(1) {
        if(Serial.available()) {
            char c = Serial.read();
            
            if(c == '\n' || c == '\r' || c == ' ' || c == '!') {
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

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    delay(1000);
    Wire.begin();
    
    Serial.println("\nKayak Smart System Initializing...");

    filter.begin(IMU_FREQUENCY);
    kayak.begin();
    // Инициализация BLE клиента
    paddle.begin("KayakClient");
    
    // Создание задач
    xTaskCreatePinnedToCore(
        bleTask,
        "BLE",
        BLE_STACK_SIZE,
        NULL,
        2,
        NULL,
        1  // BLE на ядре 1
    );
    
    xTaskCreatePinnedToCore(
        processDataTask,
        "Process",
        PROCESS_STACK_SIZE,
        NULL,
        1,
        NULL,
        0  // Обработка на ядре 0
    );
    
    xTaskCreatePinnedToCore(
        serialCommandTask,
        "SerialCmd",
        4096,
        NULL,
        1,
        NULL,
        0  // Команды на ядре 0
    );
    
    Serial.println("Kayak Smart System Ready!");
    Serial.println("Type 'help' for available commands");
}

void loop() {
    vTaskDelete(NULL);
} 