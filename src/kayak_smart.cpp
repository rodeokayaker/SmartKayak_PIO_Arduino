#include "SmartPaddle.h"
#include <EEPROM.h>
#include "Wire.h"
#include "esp_log.h"
#include "IMUSensor_GY85.h"
#include "MadgwickAHRS.h"

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

// define initial power state, mode and force threshold
const int FORCE_THRESHOLD = 20; // минимальный уровень чувствительности тензодатчиков
const int MAX_FORCE = 100; // максимальное усилие на весле
int PWM1_DutyCycle = 0;

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
#define BUTTON_FREQUENCY 10
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


enum LED_MODE {
    LED_OFF,
    LED_ON,
    LED_CYCLE
};

class LEDDriver {
protected:
    const int pin;
    bool is_on;
    bool in_cycle;
    uint32_t cycle_period_on;
    uint32_t cycle_period_off;
    uint32_t stop_cycle;
    LED_MODE cycle_finish_mode;
    TaskHandle_t ledTaskHandle;
    TimerHandle_t ledTimer;
    
    // Статический метод для задачи FreeRTOS
    static void ledTask(void* parameter) {
        LEDDriver* led = static_cast<LEDDriver*>(parameter);
        while(1) {
            uint32_t notification;
            // Ждем уведомления от таймера
            if(xTaskNotifyWait(0, ULONG_MAX, &notification, portMAX_DELAY)) {
                led->handleTimerEvent();
            }
        }
    }
    
    // Обработка события таймера (вызывается из задачи)
    void handleTimerEvent() {
        if(in_cycle) {
            if(stop_cycle > 0 && millis() >= stop_cycle) {
                in_cycle = false;
                xTimerStop(ledTimer, 0);
                if(cycle_finish_mode == LED_ON) {
                    on();
                } else {
                    off();
                }
                return;
            }
            
            // Переключение состояния
            is_on = !is_on;
            digitalWrite(pin, is_on ? HIGH : LOW);
            
            // Установка следующего периода
            uint32_t next_period = is_on ? cycle_period_on : cycle_period_off;
            xTimerChangePeriod(ledTimer, pdMS_TO_TICKS(next_period), 0);
        }
    }
    
    // Статический колбэк для таймера
    static void timerCallback(TimerHandle_t timer) {
        LEDDriver* led = static_cast<LEDDriver*>(pvTimerGetTimerID(timer));
        // Отправляем уведомление задаче
        if(led->ledTaskHandle != nullptr) {
            xTaskNotifyFromISR(led->ledTaskHandle, 1, eSetValueWithOverwrite, nullptr);
        }
    }


public:
    LEDDriver(int pin) : 
        pin(pin), 
        is_on(false), 
        in_cycle(false),
        ledTaskHandle(nullptr),
        ledTimer(nullptr) {}
    
    void begin() {
        Serial.printf("LEDDriver begin\n");
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
        is_on = false;
        
        // Создаем задачу
        xTaskCreate(
            ledTask,
            "LED_Task",
            2048,
            this,
            1,
            &ledTaskHandle
        );
        
        // Создаем программный таймер
        ledTimer = xTimerCreate(
            "LED_Timer",
            pdMS_TO_TICKS(100), // Начальный период не важен
            pdFALSE,  // Одноразовый таймер
            this,     // ID таймера - указатель на объект
            timerCallback
        );
    }
    
    virtual void on() {
        Serial.printf("LEDDriver on\n");
        if(ledTimer) {
            xTimerStop(ledTimer, 0);
        }
        digitalWrite(pin, HIGH);
        is_on = true;
        in_cycle = false;
    }
    
    virtual void off() {
        Serial.printf("LEDDriver off\n");
        if(ledTimer) {
            xTimerStop(ledTimer, 0);
        }
        digitalWrite(pin, LOW);
        is_on = false;
        in_cycle = false;
    }
    
    void Blink(uint32_t period_on_ms, uint32_t period_off_ms, 
               uint32_t duration_ms = 0, LED_MODE finish_mode = LED_CYCLE) {
        if(!ledTimer) return;
        
        cycle_period_on = period_on_ms;
        cycle_period_off = period_off_ms;
        cycle_finish_mode = finish_mode;
        in_cycle = true;
        
        if(duration_ms > 0) {
            stop_cycle = millis() + duration_ms;
        } else {
            stop_cycle = 0;
        }
        
        // Запуск с включенного состояния
        digitalWrite(pin, HIGH);
        is_on = true;
        
        xTimerChangePeriod(ledTimer, pdMS_TO_TICKS(period_on_ms), 0);
        xTimerStart(ledTimer, 0);
    }
    
    bool isOn() { return is_on; }
    LED_MODE getMode() { 
        if(in_cycle) return LED_CYCLE;
        return is_on ? LED_ON : LED_OFF;
    }
    
    ~LEDDriver() {
        if(ledTimer) {
            xTimerDelete(ledTimer, 0);
        }
        if(ledTaskHandle) {
            vTaskDelete(ledTaskHandle);
        }
    }
};

enum BUTTON_STATE {
    BUTTON_PRESSED,
    BUTTON_RELEASED
};

class ButtonDriver {
protected:
    int pin;
    uint32_t last_state_change;
    BUTTON_STATE state;
    int main_frequency;

public:
    ButtonDriver(int pin) : pin(pin), state(BUTTON_RELEASED), last_state_change(0) {}
    
    void begin(int frequency=100);
    
    virtual void update() {
        // В этой реализации основная обработка происходит в прерывании
    }
    
    bool isPressed() { return state == BUTTON_PRESSED; }
    BUTTON_STATE getState() { return state; }
    int getPin() { return pin; }
    int getFrequency() { return main_frequency; }
    
    // Виртуальные функции для обработки событий
    virtual void onPress() {}
    virtual void onRelease() {}
    
    friend void IRAM_ATTR buttonISR(void* arg); // Для доступа к protected полям из прерывания
};

// Обработчик прерывания для кнопки
void IRAM_ATTR buttonISR(void* arg) {
    ButtonDriver* button = (ButtonDriver*)arg;
    // Используем millis() для защиты от дребезга
    uint32_t now = millis();
    if (now - button->last_state_change >= button->main_frequency) {
        button->last_state_change = now;
        if (digitalRead(button->getPin()) == HIGH) {
            button->state = BUTTON_PRESSED;
            button->onPress();
        } else {
            button->state = BUTTON_RELEASED;
            button->onRelease();
        }
    }
}

void ButtonDriver::begin(int frequency) {
    main_frequency = frequency;
    pinMode(pin, INPUT);
        
    // Настройка прерывания
    attachInterruptArg(
            digitalPinToInterrupt(pin),
            buttonISR,
            this,
            CHANGE
    );
}


class PowerButton : public ButtonDriver {
private:

LEDDriver greenLED;
LEDDriver blueLED;
LEDDriver redLED;
int currentMode;
    
public:
    PowerButton(int pin) : 
        ButtonDriver(pin), 
        currentMode(3), 
        greenLED(LOW_LED_PIN), 
        blueLED(MED_LED_PIN), 
        redLED(HIGH_LED_PIN) {}

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
    void onPress() override {
        Serial.printf("PowerButton onPress %d to ", currentMode);
        currentMode = ((currentMode+1) % 3) + 1;
        Serial.printf("%d\n", currentMode);
        switch(currentMode) {
            case 3: greenLED.on(); blueLED.off(); redLED.off(); break;
            case 2: greenLED.off(); blueLED.on(); redLED.off(); break;
            case 1: greenLED.off(); blueLED.off(); redLED.on(); break;
        }
    }
    void onRelease() override {
    
    }
};

class IMotorDriver {
public:
    virtual void begin() = 0;
    virtual void setDutyCycle(int dutyCycle) = 0;
    virtual void setDirection(int direction) = 0;
};

class SmartKayak {
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

public:
    SmartKayak(IIMU* imu) : imu(imu), paddle_count(0), force_mode(3) {};
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

    Quaternion getOrientation() {
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


void buttonLEDTasks(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/BUTTON_FREQUENCY);

    while(1) {
        // Обновление состояния светодиодов в зависимости от режима
 /*
        switch(kayak.getForceMode()) {
            case 1: // LOW
                greenLED.on();
                blueLED.off();
                redLED.off();
                break;
            case 2: // MEDIUM
                greenLED.off();
                blueLED.on();
                redLED.off();
                break;
            case 3: // HIGH
                greenLED.off();
                blueLED.off();
                redLED.on();
                break;
        }
 */       
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

PowerButton powerButton(BUTTON_PIN);

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    delay(1000);
    Wire.begin();
    
    Serial.println("\nKayak Smart System Initializing...");

    kayak.begin(IMU_FREQUENCY);
    // Инициализация BLE клиента
    paddle.begin("KayakClient");
    powerButton.begin();

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
 
    xTaskCreatePinnedToCore(
        buttonLEDTasks,
        "ButtonLED",
        4096,
        NULL,
        1,
        NULL,
        0  // LED на ядре 0
    );
    Serial.println("Kayak Smart System Ready!");
    Serial.println("Type 'help' for available commands");
}

void loop() {
    vTaskDelete(NULL);
} 