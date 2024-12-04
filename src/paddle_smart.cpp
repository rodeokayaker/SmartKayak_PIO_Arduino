#include "SmartPaddle.h"
#include "IMUSensor_GY85.h"
#include <EEPROM.h>
#include "HX711.h"
#include "Wire.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_intr_alloc.h"
#include "esp_event.h"
#include "esp_private/usb_console.h"  // для CDC событий

#define INCLUDE_vTaskDelayUntil 1


// Определения пинов
constexpr int RIGHT_LOADCELL_DOUT_PIN = 2;
constexpr int RIGHT_LOADCELL_SCK_PIN = 3;
constexpr int LEFT_LOADCELL_DOUT_PIN = 5;
constexpr int LEFT_LOADCELL_SCK_PIN = 6;
constexpr int I2C_SDA = 8;
constexpr int I2C_SCL = 9;

// Адреса в EEPROM

constexpr int PADDLE_ID_ADDR = 0;
constexpr int CALIB_LOAD_FLAG_ADDR = 4;
constexpr int CALIB_L_ADDR = 8;
constexpr int CALIB_R_ADDR = 12;
constexpr int CALIB_IMU_FLAG_ADDR = 16;
constexpr int IMU_CALIB_ADDR = 20;
constexpr int TRUSTED_DEV_ADDR = 74;

// Размер EEPROM
constexpr int EEPROM_SIZE = 512;

constexpr byte VALID_CALIB_FLAG = 0x42;

// FreeRTOS определения
#define SENSOR_STACK_SIZE 4096
#define IMU_STACK_SIZE 4096
#define LOAD_FREQUENCY 10
#define IMU_FREQUENCY 50
#define BLE_FREQUENCY 100

#define BLE_STACK_SIZE 4096

static bool log_imu = false;
static bool log_load = false;

// Глобальная переменная для хранения handle задачи
TaskHandle_t serialTaskHandle = NULL;

// Класс для работы с тензодатчиками
class LoadCell : public ILoadCell {
private:
    HX711& scale;

    bool calibValid;
    int eepromAddr;
    
public:

    float calibrationFactor;

    LoadCell(HX711& scale, int eepromCalibAddr) 
        : scale(scale), eepromAddr(eepromCalibAddr), calibValid(false) {}
    
    bool begin() override {
        if(!scale.wait_ready_timeout(1000)) {
            Serial.println("HX711 not found!");
            return false;
        }
        
        // Чтение калибровки из EEPROM
        EEPROM.begin(512);
        if(EEPROM.read(CALIB_LOAD_FLAG_ADDR) == VALID_CALIB_FLAG) {
            calibrationFactor = EEPROM.readFloat(eepromAddr);
            scale.set_scale(calibrationFactor);
            calibValid = true;
        } else {
            scale.set_scale();
            calibValid = false;
        }
        EEPROM.end();
        scale.tare();
        return true;
    }
    
    float getForce() override {
        return scale.get_units(1);
    }
    
    void calibrate() override {
        scale.set_scale();
        scale.tare();
        Serial.println("Place 1kg weight and wait...");
        delay(5000);
        
        float reading = scale.get_units(10);
        calibrationFactor = reading / 1000.0; // 1kg = 1000g
        
        scale.set_scale(calibrationFactor);
        EEPROM.begin(512);
        EEPROM.writeFloat(eepromAddr, calibrationFactor);
        EEPROM.write(CALIB_LOAD_FLAG_ADDR, VALID_CALIB_FLAG);
        EEPROM.commit();
        EEPROM.end();
        
        calibValid = true;
    }
    
    bool isCalibrationValid() override {
        return calibValid;
    }
};



// Глобальные объекты
HX711 scaleR, scaleL;
ADXL345 accel;
ITG3200 gyro;
MechaQMC5883 qmc;
SmartPaddleBLEServer paddle(IMU_FREQUENCY); //  работаем как сервер

IMUSensor imuSensor(accel, gyro, qmc, 
                       IMU_CALIB_ADDR,    // Адрес калибровки IMU
                       CALIB_IMU_FLAG_ADDR); // Адрес флага калибровки IMU

LoadCell leftCell(scaleL, CALIB_L_ADDR);
LoadCell rightCell(scaleR, CALIB_R_ADDR);

// Генерация уникального ID весла
uint32_t generatePaddleID() {
    uint32_t chipId = ESP.getEfuseMac();
    return chipId ^ 0xDEADBEEF; // XOR с константой для уникальности
}


static uint32_t last_load_time = millis();
static float frequency_load = 10;

// Задача чтения тензодатчиков
void loadCellTask(void *pvParameters) {

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/LOAD_FREQUENCY);
    
    while(1) {
        int32_t time_diff = millis()-last_load_time;
        last_load_time = millis();
        frequency_load = 100.0/(99.0/frequency_load+time_diff/1000.0);
//        Serial.printf("Load cell callback time = %d, frequency = %f\n", time_diff, frequency_load);
        if (paddle.connected()) {
            paddle.updateLoads(&rightCell, &leftCell);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Задача чтения IMU

static uint32_t last_imu_time = millis();
static float frequency_imu = 50;

void imuTask(void *pvParameters) {
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/IMU_FREQUENCY);
    
    while(1) {
        int32_t time_diff = millis()-last_imu_time;
        last_imu_time = millis();
        frequency_imu = 100.0/(99.0/frequency_imu+time_diff/1000.0);
//        Serial.printf("IMU callbck time = %d, frequency = %f\n", time_diff, frequency_imu);

        if (paddle.connected()) {
            paddle.updateIMU(&imuSensor);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

    }
}


// Задача обработки BLE
static uint32_t last_ble_time = millis();
static float frequency_ble = 100;

void bleTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/BLE_FREQUENCY);
    
    while(1) {
        int32_t time_diff = millis()-last_ble_time;
        last_ble_time = millis();
        frequency_ble = 100.0/(99.0/frequency_ble+time_diff/1000.0);
//        Serial.printf("BLE callbck time = %d, frequency = %f\n", time_diff, frequency_ble);

        paddle.updateBLE();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Команды калибровки
#define CMD_CALIBRATE_LOAD "calibrate_load"
#define CMD_CALIBRATE_IMU "calibrate_imu"
#define CMD_CALIBRATE_ALL "calibrate_all"
#define CMD_HELP "help"
#define CMD_STATUS "status"
#define CMD_LOG_IMU "log_imu"
#define CMD_LOG_LOAD "log_load"
#define CMD_LOG_STOP "log_stop"
#define CMD_PAIR "pair"
#define CMD_UNPAIR "unpair"
#define CMD_BLE_STATUS "ble_status"

// Обработка команд
void processCommand(const char* cmd) {
    if(strcmp(cmd, CMD_CALIBRATE_LOAD) == 0) {
        leftCell.calibrate();
        rightCell.calibrate();  
    }
    else if(strcmp(cmd, CMD_CALIBRATE_IMU) == 0) {
        imuSensor.calibrate();
    }
    else if(strcmp(cmd, CMD_CALIBRATE_ALL) == 0) {
        leftCell.calibrate();
        rightCell.calibrate();
        imuSensor.calibrate();
    }
    else if(strcmp(cmd, CMD_HELP) == 0) {
        Serial.println("Available commands:");
        Serial.println("calibrate_load - Calibrate load cells");
        Serial.println("calibrate_imu  - Calibrate IMU sensors");
        Serial.println("calibrate_all  - Calibrate all sensors");
        Serial.println("status         - Show current sensor status");
        Serial.println("pair          - Start BLE pairing mode");
        Serial.println("unpair        - Clear paired device");
        Serial.println("ble_status    - Show BLE connection status");
        Serial.println("help           - Show this help");
        Serial.println("log_imu        - Start logging IMU data");
        Serial.println("log_load       - Start logging load data");
        Serial.println("log_stop       - Stop logging data");
    }
    else if(strcmp(cmd, CMD_STATUS) == 0) {
       Serial.println("LOAD Calibration: ");
        Serial.print("Left: ");
        Serial.print(leftCell.calibrationFactor);
        Serial.print(" Right: ");
        Serial.println(rightCell.calibrationFactor);
        Serial.println("IMU Calibration: ");
        Serial.print("Accel Scale: ");
        Serial.print(imuSensor.getCalibrationData().accelScale[0]);
        Serial.print(", ");
        Serial.print(imuSensor.getCalibrationData().accelScale[1]);
        Serial.print(", ");
        Serial.println(imuSensor.getCalibrationData().accelScale[2]);
        Serial.print("Accel Offset: ");
        Serial.print(imuSensor.getCalibrationData().accelOffset[0]);
        Serial.print(", ");
        Serial.print(imuSensor.getCalibrationData().accelOffset[1]);
        Serial.print(", ");
        Serial.println(imuSensor.getCalibrationData().accelOffset[2]);
        Serial.print("Gyro Scale: ");
        Serial.print(imuSensor.getCalibrationData().gyroScale[0]);
        Serial.print(", ");
        Serial.print(imuSensor.getCalibrationData().gyroScale[1]);
        Serial.print(", ");
        Serial.println(imuSensor.getCalibrationData().gyroScale[2]);
        Serial.print("Gyro Offset: ");
        Serial.print(imuSensor.getCalibrationData().gyroOffset[0]);
        Serial.print(", ");
        Serial.print(imuSensor.getCalibrationData().gyroOffset[1]);
        Serial.print(", ");
        Serial.println(imuSensor.getCalibrationData().gyroOffset[2]);
        Serial.print("Mag Scale: ");
        Serial.print(imuSensor.getCalibrationData().magScale[0]);
        Serial.print(", ");
        Serial.print(imuSensor.getCalibrationData().magScale[1]);
        Serial.print(", ");
        Serial.println(imuSensor.getCalibrationData().magScale[2]);    
        Serial.print("Mag Offset: ");
        Serial.print(imuSensor.getCalibrationData().magOffset[0]);
        Serial.print(", ");
        Serial.print(imuSensor.getCalibrationData().magOffset[1]);
        Serial.print(", ");
        Serial.println(imuSensor.getCalibrationData().magOffset[2]);
        
        // Добавляем статус BLE
        Serial.println("\nBLE Status:");
        Serial.printf("Connected: %s\n", paddle.connected() ? "Yes" : "No");
        Serial.printf("Pairing Mode: %s\n", paddle.isPairing() ? "Yes" : "No");
    }
    else if(strcmp(cmd, CMD_PAIR) == 0) {
        Serial.println("Starting BLE pairing mode...");
        paddle.startPairing();
    }
    else if(strcmp(cmd, CMD_UNPAIR) == 0) {
        Serial.println("Clearing paired device... ");
        paddle.clearTrustedDevice();
    }
    else if(strcmp(cmd, CMD_BLE_STATUS) == 0) {
        Serial.println("BLE Status:");
        Serial.printf("Connected: %s\n", paddle.connected() ? "Yes" : "No");
        Serial.printf("Pairing Mode: %s\n", paddle.isPairing() ? "Yes" : "No");
        if(paddle.connected()) {
            Serial.println("Client Info:");
            // Можно добавить дополнительную информацию о подключенном клиенте
        }
    }
    else if(strcmp(cmd, CMD_LOG_IMU) == 0)  {
        log_imu = true;
        paddle.set_log_imu(1);
        imuSensor.setLogLevel(1);
    }
    else if(strcmp(cmd, CMD_LOG_LOAD) == 0)  {
        log_load = true;
    }
    else if(strcmp(cmd, CMD_LOG_STOP) == 0)  {
        log_imu = false;
        log_load = false;
        imuSensor.setLogLevel(0);
        paddle.set_log_imu(0);
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
            
            if(c == '\n' || c == '\r'|| c == ' '|| c == '!') {
                if(cmdIndex > 0) {
                    cmdBuffer[cmdIndex] = '\0';
                    processCommand(cmdBuffer);
                    cmdIndex = 0;
                }
            } else if(cmdIndex < 31) {
                cmdBuffer[cmdIndex++] = c;
            }
        } else {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
    }
}


// Обработчик прерывания для получения данных через Serial
void IRAM_ATTR onSerialReceive(void* arg, esp_event_base_t base, int32_t event_id, void* event_data) {
    // Проверяем, что это событие получения данных
    if (event_id == ARDUINO_HW_CDC_RX_EVENT) {  // или CDC_EVENT_RX
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(serialTaskHandle, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
    // Другие события можно игнорировать
}

// Установка прерывания для получения данных через Serial
void setupSerialInterrupt() {
    // Устанавливаем функцию обработки прерывания для Serial
    Serial.onEvent(ARDUINO_HW_CDC_RX_EVENT, onSerialReceive);
    Serial.println("Serial interrupt setup complete");
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    delay(1000);
    Wire.begin(I2C_SDA, I2C_SCL);
    
    Serial.println("\nSmart Paddle Initializing...");
    
    // Инициализация тензодатчиков
    scaleR.begin(RIGHT_LOADCELL_DOUT_PIN, RIGHT_LOADCELL_SCK_PIN);
    scaleL.begin(LEFT_LOADCELL_DOUT_PIN, LEFT_LOADCELL_SCK_PIN);
    
    // Инициализация IMU

    imuSensor.begin();

    // Инициализация Весла
    
    // Проверка/генерация ID весла
    uint32_t paddleId;
    paddleId = generatePaddleID();
    paddle.setPaddleID(paddleId);
    paddle.setFilterFrequency(IMU_FREQUENCY);

    // Инициализация Весла
    paddle.begin("SmartPaddle v. 1.0");


    Serial.printf("Paddle ID: %08X\n", paddleId);
    
    // Проверка калибровки
    if(!imuSensor.isCalibrationValid()) {
        Serial.println("\n*** WARNING: IMU calibration required! ***");
        Serial.println("Current calibration data is missing or outdated.");
        Serial.println("Please run 'calibrate_imu' command to perform calibration.");
    } 

    if(!leftCell.isCalibrationValid() || !rightCell.isCalibrationValid()) {
        Serial.println("\n*** WARNING: Load cell calibration required! ***");
        Serial.println("Current calibration data is missing or outdated.");
        Serial.println("Please run 'calibrate_load' command to perform calibration.");
    }

    Serial.println("Type 'help' for available commands.\n");        
    
    // Создание задач на ядре 0
    xTaskCreatePinnedToCore(
        loadCellTask,
        "LoadCell",
        SENSOR_STACK_SIZE,
        NULL,
        1,
        NULL,
        0
    );
    
    xTaskCreatePinnedToCore(
        imuTask,
        "IMU",
        IMU_STACK_SIZE,
        NULL,
        1,
        NULL,
        0
    );

    
    // Задачи на ядре 1
    xTaskCreatePinnedToCore(
        bleTask,
        "BLE",
        BLE_STACK_SIZE,
        NULL,
        2,  // Более высокий приоритет для BLE
        NULL,
        1
    );
    
    xTaskCreatePinnedToCore(
        serialCommandTask,
        "SerialCmd",
        4096,
        NULL,
        1,
        &serialTaskHandle,
        1
    );

    setupSerialInterrupt();
    
/*    // Инициализация BLE с уникальным ID весла
    char deviceName[32];
    snprintf(deviceName, sizeof(deviceName), "SmartPaddle-%08X", paddleId);
    paddle.begin(deviceName);
*/  

    Serial.println("Smart Paddle Ready!");
    Serial.println("Type 'help' for available commands");
}


void loop() {
    vTaskDelete(NULL);
} 