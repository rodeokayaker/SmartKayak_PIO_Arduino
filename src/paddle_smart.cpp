#include "SmartPaddle.h"
#include "SmartPaddleServer.h"
//#include "IMUSensor_GY85.h"
//#include "IMUSensor_GY87.h"
#include "IMUSensor_BNO055.h"
#include "HX711.h"
#include "Wire.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_intr_alloc.h"
#include "esp_event.h"
#include "esp_private/usb_console.h"  // для CDC событий
#include "LoadCellHX711.h"
#include "SP_BLESerial.h"
#include "LogInterface.h"
#include "Peripherals.h"


#define INCLUDE_vTaskDelayUntil 1

#define Y_AXIS_DIRECTION -1   //Ось Y IMU датчика направлена на правую лопатку


// Определения пинов
constexpr int RIGHT_LOADCELL_DOUT_PIN = 2;
constexpr int RIGHT_LOADCELL_SCK_PIN = 3;
constexpr int LEFT_LOADCELL_DOUT_PIN = 5;
constexpr int LEFT_LOADCELL_SCK_PIN = 6;
constexpr int I2C_SDA = 9;
constexpr int I2C_SCL = 10;
constexpr int INTERRUPT_PIN = 0;
constexpr int POWER_PIN = 20;
constexpr int SWITCH_OFF_PIN = 4;
constexpr uint32_t SHUTDOWN_DELAY_MS = 60000; // 1 минута



// FreeRTOS определения
#define SENSOR_STACK_SIZE 4096
#define IMU_STACK_SIZE 4096
#define LOAD_FREQUENCY 10
#define IMU_FREQUENCY 100
#define BLE_FREQUENCY 100
#define BLE_SERIAL_FREQUENCY 10
#define BLE_STACK_SIZE 4096

extern bool log_imu;
extern bool log_load;

// Глобальная переменная для хранения handle задачи
TaskHandle_t serialTaskHandle = NULL;

// Класс для работы с тензодатчиками


// Глобальные объекты
LoadCellHX711 leftCell("LEFT_LOAD", LEFT_LOADCELL_DOUT_PIN, LEFT_LOADCELL_SCK_PIN);
LoadCellHX711 rightCell("RIGHT_LOAD", RIGHT_LOADCELL_DOUT_PIN, RIGHT_LOADCELL_SCK_PIN);
SmartPaddleBLEServer paddle("SmartPaddle"); //  работаем как сервер
IMUSensor_BNO055 imuSensor("IMU_PADDLE_MAIN_BNO055", 0x29, -1, &Serial); 

// Генерация уникального ID весла
uint32_t generatePaddleID() {
    uint32_t chipId = ESP.getEfuseMac();
    return chipId ^ 0xDEADBEEF; // XOR с константой для уникальности
}


class RGBLedInterface: public ILogInterface{
    private:
    int r_pin;
    int g_pin;
    int b_pin;
    public:
    RGBLedInterface(int r_pin, int g_pin, int b_pin):r_pin(r_pin), g_pin(g_pin), b_pin(b_pin){}
    void logQuaternion(const float* q) override{
        float yaw=atan2(2.0f*(q[0]*q[1]+q[2]*q[3]),1.0f-2.0f*(q[1]*q[1]+q[2]*q[2]));
        float pitch=asin(2.0f*(q[0]*q[2]-q[1]*q[3]));
        float roll=atan2(2.0f*(q[0]*q[3]+q[1]*q[2]),1.0f-2.0f*(q[2]*q[2]+q[3]*q[3]));

        if (yaw>M_PI) yaw-=2.0f*M_PI;
        if (pitch>M_PI) pitch-=2.0f*M_PI;
        if (roll>M_PI) roll-=2.0f*M_PI;
        if (yaw<-M_PI) yaw+=2.0f*M_PI;
        if (pitch<-M_PI) pitch+=2.0f*M_PI;
        if (roll<-M_PI) roll+=2.0f*M_PI;
        if (yaw<0) yaw=-yaw;
        if (pitch<0) pitch=-pitch;
        if (roll<0) roll=-roll;

        uint8_t yaw_int=uint8_t(yaw*255.0f/(M_PI));
        uint8_t pitch_int=uint8_t(pitch*255.0f/(M_PI));
        uint8_t roll_int=uint8_t(roll*255.0f/(M_PI));

        analogWrite(r_pin, yaw_int);
        analogWrite(g_pin, pitch_int);
        analogWrite(b_pin, roll_int);

    }
};

void SwitchOff(){
    Serial.printf("Shutdown Paddle. PIN: %d\n", POWER_PIN);
    if (POWER_PIN>=0) digitalWrite(POWER_PIN, LOW);

    
}


class switchOffButton: public ButtonDriver {
    uint32_t lastReleaseTime;
    public:
    switchOffButton(int pin): ButtonDriver(pin), lastReleaseTime(0) {}
    void onLongPress() override {
        if (millis()<5000) {
            return;
        }
        if (millis()-lastReleaseTime<4000) {
            Serial.println("Starting pairing mode");
            paddle.startPairing();
            return;
        }
        Serial.println("Switch off button long press");
        SwitchOff();
        delay(100);
        ESP.restart();
    }
    void onRelease() override { lastReleaseTime=millis();}
    void onPress() override { }
};

switchOffButton OffButton(SWITCH_OFF_PIN);

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

// Обработчик событий для Весла
class PaddleEventHandler: public SP_EventHandler {
    private:
    TimerHandle_t shutdownTimer;

    static void shutdownTimerCallback(TimerHandle_t timer) {
        SwitchOff();
    }

    public:
    PaddleEventHandler() {
        shutdownTimer = xTimerCreate(
            "ShutdownTimer",
            pdMS_TO_TICKS(SHUTDOWN_DELAY_MS),
            pdFALSE,  // one-shot timer
            this,
            shutdownTimerCallback
        );
        xTimerStart(shutdownTimer, 0);
    }

    ~PaddleEventHandler() {
        if (shutdownTimer) {
            xTimerDelete(shutdownTimer, 0);
        }
    }

    void onConnect(SmartPaddle* paddle) override {
        Serial.println("Paddle connected");
        xTimerStop(shutdownTimer, 0);
    }

    void onDisconnect(SmartPaddle* paddle) override {
        Serial.println("Paddle disconnected");
        xTimerStart(shutdownTimer, 0);
    }

    void onShutdown(SmartPaddle* paddle) override {
        SwitchOff();
    }
};

PaddleEventHandler eventHandler;

// Обработка команд
void processCommand(const char* cmd) {

    paddle.getSerial()->sendString(paddle.getSerial()->Processor().createLogMessage(cmd));
    if(strcmp(cmd, CMD_CALIBRATE_LOAD) == 0) {
        paddle.setLogStream(&Serial);
        paddle.calibrateLoads(ALL_BLADES);
    }
    else if(strcmp(cmd, CMD_CALIBRATE_IMU) == 0) {
        Serial.printf("Calibrate IMU command\n");
        paddle.setLogStream(&Serial);
        Serial.printf("Log stream set to Serial\n");
        paddle.calibrateIMU();
    }
    else if(strcmp(cmd, CMD_CALIBRATE_ALL) == 0) {
        paddle.setLogStream(&Serial);
        paddle.calibrateLoads(ALL_BLADES);
        paddle.calibrateIMU();
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
        Serial.printf("Left: factor = %.3f, offset = %d\n", leftCell.getCalibrationData().calibrationFactor, leftCell.getCalibrationData().offset);
        Serial.printf("Right: factor = %.3f, offset = %d\n", rightCell.getCalibrationData().calibrationFactor, rightCell.getCalibrationData().offset);
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
    }
    else if(strcmp(cmd, CMD_LOG_LOAD) == 0)  {
        log_load = true;
    }
    else if(strcmp(cmd, CMD_LOG_STOP) == 0)  {
        log_imu = false;
        log_load = false;
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
            Serial.print(c);
            
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
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, HIGH);
    Serial.begin(115200);
    for (int i=0; i<200; i++) {
        digitalWrite(POWER_PIN, HIGH);  
        delay(10);
    }
    Wire.end();
    Wire.begin(I2C_SDA, I2C_SCL);
    
    Serial.println("\nSmart Paddle Initializing...");
    
    // Инициализация тензодатчиков
    leftCell.begin(HX711_DEFAULT_FREQUENCY);
    rightCell.begin(HX711_DEFAULT_FREQUENCY);

    // Инициализация IMU

    imuSensor.setInterruptPin(INTERRUPT_PIN);
    imuSensor.begin();

    // Инициализация Весла
    
    // Проверка/генерация ID весла
    uint32_t paddleId;
    paddleId = generatePaddleID();
    paddle.setPaddleID(paddleId);
    paddle.setFilterFrequency(IMU_FREQUENCY);
    paddle.setEventHandler(&eventHandler);
    paddle.SetYAxisDirection(Y_AXIS_DIRECTION);
    // Инициализация Весла
    paddle.setIMU(&imuSensor);
    paddle.setLoads(&rightCell, &leftCell);

 //   paddle.setLogInterface(new RGBLedInterface(4, 1, 7));
    
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
    
    setupSerialInterrupt();
    
    paddle.begin("SmartPaddle v. 1.2");
    OffButton.begin();
    
    xTaskCreatePinnedToCore(
        serialCommandTask,
        "SerialCmd",
        4096*2,
        NULL,
        1,
        &serialTaskHandle,
        1
    );

    paddle.startTasks();


    digitalWrite(POWER_PIN, HIGH);   
}


void loop() {
    digitalWrite(POWER_PIN, HIGH); 

    vTaskDelete(NULL);
} 