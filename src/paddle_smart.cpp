#include "SmartPaddle.h"
#include "IMUSensor_GY85.h"
#include <EEPROM.h>
#include "HX711.h"
#include "Wire.h"

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


// Размер EEPROM
constexpr int EEPROM_SIZE = 512;

constexpr byte VALID_CALIB_FLAG = 0x42;

// FreeRTOS определения
#define SENSOR_STACK_SIZE 4096
#define IMU_STACK_SIZE 4096
#define LOAD_FREQUENCY 10
#define IMU_FREQUENCY 50

static bool log_imu = false;
static bool log_load = false;


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
        if(EEPROM.read(CALIB_LOAD_FLAG_ADDR) == VALID_CALIB_FLAG) {
            calibrationFactor = EEPROM.readFloat(eepromAddr);
            scale.set_scale(calibrationFactor);
            calibValid = true;
        } else {
            scale.set_scale();
            calibValid = false;
        }
        
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
        EEPROM.writeFloat(eepromAddr, calibrationFactor);
        EEPROM.write(CALIB_LOAD_FLAG_ADDR, VALID_CALIB_FLAG);
        EEPROM.commit();
        
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
SmartPaddle paddle(true, IMU_FREQUENCY); // true - работаем как сервер

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


// Задача чтения тензодатчиков
void loadCellTask(void *pvParameters) {

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/LOAD_FREQUENCY);
    
    while(1) {
//        Serial.println("Load cell task is running!");
        paddle.updateLoads(&rightCell, &leftCell);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Задача чтения IMU
void imuTask(void *pvParameters) {
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/IMU_FREQUENCY);
    
    while(1) {
//        Serial.println("IMU task is running!");
        paddle.updateIMU(&imuSensor);
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
  //      Serial.println("Serial command task is running!");
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
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    delay(1000);

    EEPROM.begin(EEPROM_SIZE);
    Wire.begin(I2C_SDA, I2C_SCL);
    
    Serial.println("\nSmart Paddle Initializing...");
    
    // Инициализация тензодатчиков
    scaleR.begin(RIGHT_LOADCELL_DOUT_PIN, RIGHT_LOADCELL_SCK_PIN);
    scaleL.begin(LEFT_LOADCELL_DOUT_PIN, LEFT_LOADCELL_SCK_PIN);
    
    // Инициализация IMU

    imuSensor.begin();
    paddle.begin();
    
    // Проверка/генерация ID весла
    uint32_t paddleId;
    paddleId = generatePaddleID();
 
    Serial.printf("Paddle ID: %08X\n", paddleId);
    
    // Проверка калибровки
    if(!imuSensor.isCalibrationValid() || !leftCell.isCalibrationValid() || !rightCell.isCalibrationValid()) {
        Serial.println("\n*** WARNING: Calibration required! ***");
        Serial.println("Current calibration data is missing or outdated.");
        Serial.println("Please run 'calibrate_all' command to perform full calibration.");
        Serial.println("Type 'help' for available commands.\n");
    } 
        
    // Создание задач
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
    
    paddle.begin();
    
    // Добавляем задачу обработки команд
    xTaskCreatePinnedToCore(
        serialCommandTask,
        "SerialCmd",
        4096,
        NULL,
        1,
        NULL,
        1  // На втором ядре
    );
    
    Serial.println("Smart Paddle Ready!");
    Serial.println("Type 'help' for available commands");
}

void loop() {
    // Второе ядро пока свободно для BLE
    vTaskDelete(NULL);
} 