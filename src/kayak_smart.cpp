#include "SmartPaddleClient.h"
#include "SmartPaddleServer.h"

#include "Wire.h"
#include "esp_log.h"
#include "IMUSensor_GY85.h"
#include "MadgwickAHRS.h"
#include "Peripherals.h"

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
#define PROCESS_FREQUENCY 50  // Частота обработки данных в Гц
#define IMU_FREQUENCY 100
#define BLE_FREQUENCY 100
#define BLE_SERIAL_FREQUENCY 10
static bool log_paddle = false;

extern bool log_imu;
extern bool log_load;

// Глобальные объекты
SmartPaddleBLEClient paddle("Paddle_1");



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


class SmartKayak;
class PowerButton : public ButtonDriver {
private:

    LEDDriver greenLED;
    LEDDriver blueLED;
    LEDDriver redLED;
    int currentMode;
    SmartKayak* kayak;
public:
    PowerButton(int pin, SmartKayak* kayak) : 
        ButtonDriver(pin), 
        currentMode(3), 
        greenLED(LOW_LED_PIN), 
        blueLED(MED_LED_PIN), 
        redLED(HIGH_LED_PIN),
        kayak(kayak) {}

    void begin() {
        Serial.printf("PowerButton begin\n");
        ButtonDriver::begin(200);
        greenLED.begin();
        blueLED.begin();
        redLED.begin();
        switch(currentMode) {
            case 3: greenLED.on(); blueLED.off(); redLED.off(); break;
            case 2: greenLED.off(); blueLED.on(); redLED.off(); break;
            case 1: greenLED.off(); blueLED.off(); redLED.on(); break;
        }

    }
    void onPress() override;
    void onRelease() override {
    
    }
    void onLongPress() override {}
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

enum MotorDirection {
    FORWARD = 1,
    REVERSE = -1
};

enum MotorPowerMode {
    LOW_POWER = 1,
    MEDIUM_POWER = 2,
    HIGH_POWER = 3
};

class IMotorDriver {
protected:
    int currentDutyCycle;
    int maxDutyCycle;
    MotorDirection direction;
    MotorPowerMode powerMode;
    bool isEnabled;

public:
    IMotorDriver() : 
        currentDutyCycle(0), 
        maxDutyCycle(255),
        direction(FORWARD),
        powerMode(LOW_POWER),
        isEnabled(false) {}
        
    virtual void begin() = 0;
    virtual void setDutyCycle(int dutyCycle) = 0;
    virtual void setDirection(MotorDirection dir) = 0;
    
    // Новые методы для универсальности
    virtual void setPowerMode(MotorPowerMode mode) {
        powerMode = mode;
        updatePower();
    }
    
    virtual void enable() { 
        isEnabled = true; 
        updatePower();
    }
    
    virtual void disable() { 
        isEnabled = false;
        setDutyCycle(0);
    }
    
    // Установка силы в процентах (0-100)
    virtual void setForce(int forcePercent) {
        if (!isEnabled) return;
        
        int dutyCycle = map(forcePercent, 0, 100, 0, maxDutyCycle);
        setDutyCycle(dutyCycle);
    }
    
    // Получение текущих параметров
    MotorPowerMode getPowerMode() { return powerMode; }
    MotorDirection getDirection() { return direction; }
    int getDutyCycle() { return currentDutyCycle; }
    bool isMotorEnabled() { return isEnabled; }

protected:
    virtual void updatePower() {
        if (!isEnabled) {
            setDutyCycle(0);
            return;
        }

        int powerPercent;
        switch (powerMode) {
            case LOW_POWER:
                powerPercent = MotorConstants::POWER_LOW;
                break;
            case MEDIUM_POWER:
                powerPercent = MotorConstants::POWER_MEDIUM;
                break;
            case HIGH_POWER:
                powerPercent = MotorConstants::POWER_HIGH;
                break;
            default:
                powerPercent = 0;
                break;
        }
        
        setForce(powerPercent);
    }
};

class PWMMotorDriver : public IMotorDriver {
private:
    const int motorPin;
    const int reversePin;
    const int pwmChannel;
    const int pwmResolution;
    const int pwmFrequency;

public:
    PWMMotorDriver(int mPin, int rPin) : 
        motorPin(mPin),
        reversePin(rPin),
        pwmChannel(MotorConstants::PWM_CHANNEL),
        pwmResolution(MotorConstants::PWM_RESOLUTION),
        pwmFrequency(MotorConstants::PWM_FREQUENCY) {}

    void begin() override {
        pinMode(motorPin, OUTPUT);
        pinMode(reversePin, OUTPUT);
        
        // Настройка ШИМ
        ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
        ledcAttachPin(motorPin, pwmChannel);
        
        // Начальные значения
        setDirection(FORWARD);
        setDutyCycle(0);
    }

    void setDutyCycle(int dutyCycle) override {
        currentDutyCycle = constrain(dutyCycle, 0, maxDutyCycle);
        ledcWrite(pwmChannel, currentDutyCycle);
    }

    void setDirection(MotorDirection dir) override {
        direction = dir;
        digitalWrite(reversePin, direction == REVERSE ? HIGH : LOW);
    }
};

class SmartKayak {
    friend class PowerButton;
private:
    IIMU* imu;
    Madgwick filter;
    IMUData imu_data;
    loadData load_data;
    OrientationData orientation;
    BladeData blade;
    SmartPaddle* paddles[4];
    int paddle_count;
    int force_mode = 3; // Starting from LOW mode
    PWMMotorDriver motor;
    PowerButton powerButton;


public:
    SmartKayak(IIMU* imu) : imu(imu), paddle_count(0), force_mode(3), motor(MOTOR_PIN, REVERSE_PIN), powerButton(BUTTON_PIN, this) {};
    void begin(int frequency=IMU_FREQUENCY)
    {
        if (imu){ if (!imu->begin()) Serial.println("IMU init failed");} else {
            Serial.println("IMU not initialized");
        }
        filter.begin(frequency);
        powerButton.begin();
        motor.begin();
        motor.setPowerMode(LOW_POWER);
        motor.enable();
    };

    void updateIMU() {
        imu->getData(imu_data);
        filter.update(imu_data.ax, imu_data.ay, imu_data.az, imu_data.gx, imu_data.gy, imu_data.gz, imu_data.mx, imu_data.my, imu_data.mz);
    }

    AHRSQuaternion getOrientation() {
        return filter.getQuaternion();
    }

    void addPaddle(SmartPaddle* paddle) {
        paddles[paddle_count++] = paddle;
    }

    void setForceMode(int mode) {
        force_mode = mode;
    }

    int getForceMode() {
        return force_mode;
    }


};

void PowerButton::onPress() {
//    Serial.printf("PowerButton onPress %d to ", currentMode);
    currentMode = ((currentMode+1) % 3) + 1;
//    Serial.printf("%d\n", currentMode);
    switch(currentMode) {
        case 3: greenLED.on(); blueLED.off(); redLED.off(); kayak->setForceMode(LOW_POWER); break;
        case 2: greenLED.off(); blueLED.on(); redLED.off(); kayak->setForceMode(MEDIUM_POWER); break;
        case 1: greenLED.off(); blueLED.off(); redLED.on(); kayak->setForceMode(HIGH_POWER); break;
    }
}

ADXL345 accel;
ITG3200 gyro;
MechaQMC5883 qmc;

IMUSensor_GY85 imu("imu_gy85");
SmartKayak kayak(&imu);


// Задача обработки BLE
void bleTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/BLE_FREQUENCY);

    while(1) {
        uint32_t start_ts = millis();
        paddle.updateBLE();
        uint32_t end_ts = millis();
//        Serial.printf("BLE update time: %u ms\n", end_ts - start_ts);
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
                Serial.printf("Load - L: %d, R: %d, ts: %d\n", 
                            load.forceL, load.forceR, load.timestamp);
            }
        }
        paddle.getIMUData(imu); //downgrade to 50HZ
        if(paddle.getIMUData(imu)) {
            paddleData.imu = imu;
            paddleData.imuValid = true;
            if(log_paddle) {
                uint32_t start_ts = millis();
/*                int tmp_int;
                Serial.print("IMU - Accel: ");
                tmp_int = imu.ax*1000;
                Serial.print(tmp_int);
                Serial.print(",");
                tmp_int = imu.ay*1000;
                Serial.print(tmp_int);
                Serial.print(",");
                tmp_int = imu.az*1000;
                Serial.print(tmp_int);
                Serial.print(" Gyro: ");
                tmp_int = imu.gx*1000;
                Serial.print(tmp_int);
                Serial.print(",");
                tmp_int = imu.gy*1000;
                Serial.print(tmp_int);
                Serial.print(",");
                tmp_int = imu.gz*1000;
                Serial.print(tmp_int);
                Serial.print(" Mag: ");
                tmp_int = imu.mx*1000;
                Serial.print(tmp_int);
                Serial.print(",");
                tmp_int = imu.my*1000;
                Serial.print(tmp_int);
                Serial.print(",");
                tmp_int = imu.mz*1000;
                Serial.print(tmp_int);
                Serial.print(" ts: ");
                Serial.println(imu.timestamp);*/
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

        paddle.getOrientationData(orientation); //downgrade to 50HZ
        if(paddle.getOrientationData(orientation)) {
            paddleData.orientation = orientation;
            paddleData.orientationValid = true;
            if(log_paddle) {
                Serial.printf("Madgwick: %.4f, %.4f, %.4f, %.4f, ts: %d\n",
                            orientation.q0, orientation.q1, 
                            orientation.q2, orientation.q3, orientation.timestamp);
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

    kayak.begin(IMU_FREQUENCY);
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