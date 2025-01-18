#include "SmartPaddleClient.h"
#include "SmartPaddleServer.h"

#include "Wire.h"
#include "esp_log.h"
#include "IMUSensor_GY85.h"
#include "MadgwickAHRS.h"
#include "Peripherals.h"
#include "SmartKayak.h"

#include "SPI.h"
#include "SD.h"
//#include "TroykaDHT.h"



#define INCLUDE_vTaskDelayUntil 1

// Define pins
#define BUTTON_PIN    18 // ESP32 pin GPIO18, which connected to power mode changing button
#define MOTOR_PIN     16 // PWM output for motor driver
#define REVERSE_PIN   19 // output for reverce direction PIN
#define LED_PIN       25 // PWM MOTOR Duty LED 
#define LOW_LED_PIN   33 // Green LED
#define MED_LED_PIN   27 // Blue LED
#define HIGH_LED_PIN  26 // Red LED


// Define PWM output and duty Cycle mode
#define PWM1_Ch    0
#define PWM1_Res   8
#define PWM1_Freq  1000
#define PWM1_HIGH  3.0 
#define PWM1_MED   2.0
#define PWM1_LOW   1.0

#define SD_MISO     12    // GPIO19
#define SD_MOSI     14    // GPIO23  
#define SD_SCK      13    // GPIO18
#define SD_CS       2     // GPIO5 (CS

// define initial power state, mode and force threshold
const int FORCE_THRESHOLD = 20; // минимальный уровень чувствительности тензодатчиков
const int MAX_FORCE = 100; // максимальное усилие на весле
int PWM1_DutyCycle = 0;



// FreeRTOS определения
#define BLE_STACK_SIZE 4096
#define PROCESS_STACK_SIZE 4096
#define PROCESS_FREQUENCY 100  // Частота обработки данных в Гц
#define IMU_FREQUENCY 100
#define BLE_FREQUENCY 100
#define BLE_SERIAL_FREQUENCY 10
static bool log_paddle = false;

extern bool log_imu;
extern bool log_load;


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

class PowerButton : public ButtonDriver, public IModeSwitch {
private:

    LEDDriver greenLED;
    LEDDriver blueLED;
    LEDDriver redLED;
    MotorPowerMode currentMode;
public:
    PowerButton(int pin) : 
        ButtonDriver(pin), 
        currentMode(MOTOR_LOW_POWER), 
        greenLED(LOW_LED_PIN), 
        blueLED(MED_LED_PIN), 
        redLED(HIGH_LED_PIN) {}

    void begin(){
        Serial.printf("PowerButton begin\n");
        ButtonDriver::begin();
        greenLED.begin();
        blueLED.begin();
        redLED.begin();
        switch(currentMode) {
            case MOTOR_LOW_POWER: greenLED.on(); blueLED.off(); redLED.off(); break;
            case MOTOR_MEDIUM_POWER: greenLED.off(); blueLED.on(); redLED.off(); break;
            case MOTOR_HIGH_POWER: greenLED.off(); blueLED.off(); redLED.on(); break;
        }

    }
    void onPress() override{};
    void onRelease() override;
    void onLongPress() override {}
    MotorPowerMode getMode() { return currentMode; }
};

// Константы для режимов мощности
namespace MotorConstants {
    constexpr int PWM_CHANNEL = 0;
    constexpr int PWM_RESOLUTION = 8;  // 8 бит (0-255)
    constexpr int PWM_FREQUENCY = 1000; // 1 кГц
    
    constexpr int POWER_LOW = 50;    // 50% мощности
    constexpr int POWER_MEDIUM = 75;  // 75% мощности
    constexpr int POWER_HIGH = 100;   // 100% мощности
}


void PowerButton::onRelease() {
    currentMode = (MotorPowerMode)((((int)currentMode+1) % 3) + 1);
    switch(currentMode) {
        case 3: greenLED.on(); blueLED.off(); redLED.off(); break;
        case 2: greenLED.off(); blueLED.on(); redLED.off(); break;
        case 1: greenLED.off(); blueLED.off(); redLED.on(); break;
    }
}

ADXL345 accel;
ITG3200 gyro;
MechaQMC5883 qmc;

// Глобальные объекты
SmartPaddleBLEClient paddle("Paddle_1");
IMUSensor_GY85 imu("imu_gy85");
SmartKayak kayak;
PowerButton powerButton(BUTTON_PIN);



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

        if(paddle.receiveLoadData(load)) {
            paddleData.load = load;
            paddleData.loadValid = true;
            if(log_paddle) {
                Serial.printf("Load - L: %d, R: %d, ts: %d\n", 
                            load.forceL, load.forceR, load.timestamp);
            }
        }
        if(paddle.receiveIMUData(imu)) {
            paddleData.imu = imu;
            paddleData.imuValid = true;
            if(log_paddle) {
                uint32_t start_ts = millis();
                Serial.printf("Accel: %.4f, %.4f, %.4f Gyro: %.4f, %.4f, %.4f Mag: %.4f, %.4f, %.4f, ts: %d\n",
                            imu.ax, imu.ay, imu.az,
                            imu.gx, imu.gy, imu.gz,
                            imu.mx, imu.my, imu.mz, imu.timestamp);

                
                Serial.printf("DMP: %.4f, %.4f, %.4f, %.4f, ts=%d\n",
                            imu.q0, imu.q1, imu.q2, imu.q3, imu.timestamp);
                uint32_t end_ts = millis();
                if (end_ts - start_ts > 8) {
                    Serial.printf("Print time: %d ms\n", end_ts - start_ts);
                }
            }
        }

        if(paddle.receiveOrientationData(orientation)) {
            paddleData.orientation = orientation;
            paddleData.orientationValid = true;
            if(log_paddle) {
                Serial.printf("Madgwick: %.4f, %.4f, %.4f, %.4f, ts: %d\n",
                            orientation.q0, orientation.q1, 
                            orientation.q2, orientation.q3, orientation.timestamp);
            }
        }
        
        if(paddle.receiveBladeData(blade)) {
            paddleData.blade = blade;
            paddleData.bladeValid = true;
            if(log_paddle) {
                Serial.printf("Blade - Side: %d Force: %.2f\n",
                            blade.bladeSide, blade.force);
            }
        }
        
        if(paddle.receiveStatusData(status)) {
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


void bleSerialTasks(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/BLE_SERIAL_FREQUENCY);

    while(1) {
        paddle.getSerial()->updateJSON(true);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Команды управления
#define CMD_HELP "help"
#define CMD_STATUS "status"
#define CMD_PAIR "pair"
#define CMD_LOG_START "log_start"
#define CMD_LOG_STOP "log_stop"
#define CMD_IMU_CALIBRATE "calibrate_imu"
#define CMD_PADDLE_IMU_CALIBRATE "calibrate_paddle_imu"
#define CMD_PADDLE_LOADS_CALIBRATE "calibrate_paddle_loads"
#define CMD_PADDLE_LOADS_CALIBRATE_LEFT "calibrate_paddle_left"
#define CMD_PADDLE_LOADS_CALIBRATE_RIGHT "calibrate_paddle_right"
#define CMD_PADDLE_SEND_SPECS "send_specs"
#define CMD_PADDLE_PAIR "paddle_pair"
#define CMD_PADDLE_SHUTDOWN "shutdown"

// Обработка команд
void processCommand(const char* cmd) {
    if(strcmp(cmd, CMD_HELP) == 0) {
        Serial.println("Available commands:");
        Serial.println("pair          - Start pairing with paddle");
        Serial.println("status        - Show current paddle status");
        Serial.println("log_start     - Start logging paddle data");
        Serial.println("log_stop      - Stop logging paddle data");
        Serial.println("calibrate_imu - Start IMU calibration");
        Serial.println("calibrate_paddle_imu - Start Paddle IMU calibration");
        Serial.println("calibrate_paddle_loads - Start Paddle loads calibration");
        Serial.println("calibrate_paddle_left - Start Paddle left load calibration");
        Serial.println("calibrate_paddle_right - Start Paddle right load calibration");
        Serial.println("send_specs - Send Paddle specs");
        Serial.println("paddle_pair - Start Paddle pairing");
        Serial.println("shutdown - Shutdown Paddle");
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
                     specs.paddleType == TWO_BLADES ? "Double" : "Single");
    }
    else if(strcmp(cmd, CMD_PAIR) == 0) {
        Serial.println("Starting pairing mode...");
        paddle.startPairing();
    }
    else if(strcmp(cmd, CMD_LOG_START) == 0) {
        log_paddle = true;
//        log_imu = true;
//        log_load = true;
        Serial.println("Logging started");
    }
    else if(strcmp(cmd, CMD_LOG_STOP) == 0) {
        log_paddle = false;
        log_imu = false;
        log_load = false;
        Serial.println("Logging stopped");
    } else if(strcmp(cmd, CMD_IMU_CALIBRATE) == 0) {
        imu.calibrate();
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
    else if(strcmp(cmd, CMD_PADDLE_SEND_SPECS) == 0) {
        paddle.getSerial()->sendCommand("send_specs");
    }
    else if(strcmp(cmd, CMD_PADDLE_PAIR) == 0) {
        paddle.getSerial()->sendCommand("start_pair");
    }
    else if(strcmp(cmd, CMD_PADDLE_SHUTDOWN) == 0) {
        paddle.getSerial()->sendCommand("shutdown");
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

void setup() {
    Serial.begin(230400);
    while (!Serial) delay(10);
    
    delay(1000);

    Wire.begin();
    
    Serial.println("\nKayak Smart System Initializing...");

    // Инициализация BLE клиента
    paddle.begin("SmartKayak 1.0");
    kayak.setPaddle(&paddle);
    kayak.setIMU(&imu, IMU_FREQUENCY);
    kayak.setModeSwitch(&powerButton);

    // Создание задач
    xTaskCreatePinnedToCore(
        bleTask,
        "BLE",
        BLE_STACK_SIZE,
        NULL,
        2,
        NULL,
        0  // BLE на ядре 1
    );
    
    xTaskCreatePinnedToCore(
        processDataTask,
        "Process",
        PROCESS_STACK_SIZE,
        NULL,
        2,
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
        0  // Команды на ядре 0
    );
 
    xTaskCreatePinnedToCore(
        bleSerialTasks,
        "BLESerial",
        4096,
        NULL,
        1,
        NULL,
        0  
    );
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI);

    // Инициализация SD карты
    if(!SD.begin(SD_CS)) {
        Serial.println("Card Mount Failed");
//        return;
    } else {
   // Проверка типа карты
    uint8_t cardType = SD.cardType();
    if(cardType == CARD_NONE) {
            Serial.println("No SD card attached");
            return;
        }            
    }

    Serial.println("Kayak Smart System Ready!");
    Serial.println("Type 'help' for available commands");
    Serial.print("Smart Kayak > ");
}

void loop() {
    vTaskDelete(NULL);
} 