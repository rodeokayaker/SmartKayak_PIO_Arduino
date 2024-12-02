#include "SmartPaddle.h"
#include <EEPROM.h>
#include "Wire.h"
#include "esp_log.h"

// Адреса в EEPROM
constexpr int TRUSTED_DEV_ADDR = 74;

// FreeRTOS определения
#define BLE_STACK_SIZE 4096
#define PROCESS_STACK_SIZE 4096
#define PROCESS_FREQUENCY 50  // Частота обработки данных в Гц

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

// Задача обработки BLE
void bleTask(void *pvParameters) {
    while(1) {
        paddle.updateBLE();
        vTaskDelay(pdMS_TO_TICKS(10));
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

// Команды управления
#define CMD_HELP "help"
#define CMD_STATUS "status"
#define CMD_PAIR "pair"
#define CMD_LOG_START "log_start"
#define CMD_LOG_STOP "log_stop"

// Обработка команд
void processCommand(const char* cmd) {
    if(strcmp(cmd, CMD_HELP) == 0) {
        Serial.println("Available commands:");
        Serial.println("pair      - Start pairing with paddle");
        Serial.println("status    - Show current paddle status");
        Serial.println("log_start - Start logging paddle data");
        Serial.println("log_stop  - Stop logging paddle data");
        Serial.println("help      - Show this help");
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
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    esp_log_level_set("*", ESP_LOG_ERROR);
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    delay(1000);
    
    Serial.println("\nKayak Smart System Initializing...");
    
    // Инициализация BLE клиента
    paddle.begin("KayakClient");
    
    // Создание задач
    xTaskCreatePinnedToCore(
        bleTask,
        "BLE",
        BLE_STACK_SIZE,
        NULL,
        1,
        NULL,
        0  // BLE на ядре 0
    );
    
    xTaskCreatePinnedToCore(
        processDataTask,
        "Process",
        PROCESS_STACK_SIZE,
        NULL,
        1,
        NULL,
        1  // Обработка на ядре 1
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
    
    Serial.println("Kayak Smart System Ready!");
    Serial.println("Type 'help' for available commands");
}

void loop() {
    vTaskDelete(NULL);
} 