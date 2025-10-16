#include "SmartPaddle.h"
#include "SmartPaddleServer.h"
#include "ImuBNO08X.h"
#include "LoadCellSetADS1220.h"
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
constexpr int I2C_SDA = IMU_SDA;
constexpr int I2C_SCL = IMU_SCL;
constexpr int INTERRUPT_PIN = IMU_INTA; //0;
constexpr int RESET_PIN = IMU_RST;
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

// Глобальная переменная для хранения handle задачи
TaskHandle_t serialTaskHandle = NULL;

// Класс для работы с тензодатчиками


// Глобальные объекты
LoadCellSetADS1220 loadsCellSet("LoadsADS1220", ONE_BLADE, RIGHT_BLADE);
SmartPaddleBLEServer paddle("SmartPaddle"); //  работаем как сервер
ImuBNO08X imuSensor("IMU_PADDLE_MAIN_BNO08X"); 

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
    bool switchOff;
    public:
    switchOffButton(int pin): ButtonDriver(pin), lastReleaseTime(0), switchOff(false) {}
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
        switchOff=true;
    }
    void onRelease() override { 
        lastReleaseTime=millis(); 
        if (isLongPressed()){
            if (switchOff){
                delay(300);
                SwitchOff();
                delay(1000);
                ESP.restart();
            }
        }
    }
    void onPress() override { }
};

switchOffButton OffButton(SWITCH_OFF_PIN);

// Команды калибровки
#define CMD_CALIBRATE_LOAD "calibrate_load"
#define CMD_CALIBRATE_IMU "calibrate_imu"
#define CMD_CALIBRATE_ALL "calibrate_all"
#define CMD_HELP "help"
#define CMD_STATUS "status"
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

    }
    else if(strcmp(cmd, CMD_STATUS) == 0) {
       Serial.println("LOAD Calibration: ");
  //      Serial.printf("Left: factor = %.3f, offset = %d\n", leftCell.getCalibrationData().calibrationFactor, leftCell.getCalibrationData().offset);
  //      Serial.printf("Right: factor = %.3f, offset = %d\n", rightCell.getCalibrationData().calibrationFactor, rightCell.getCalibrationData().offset);

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
    Wire.setClock(400000);
    
    Serial.println("\nSmart Paddle Initializing...");
    
    // Инициализация тензодатчиков
//    leftCell.begin(HX711_DEFAULT_FREQUENCY);
//    rightCell.begin(HX711_DEFAULT_FREQUENCY);
    loadsCellSet.begin(LOADCELL_CS, LOADCELL_MISO, LOADCELL_MOSI, LOADCELL_SCK, LOADCELL_DRDY);
    loadsCellSet.setFrequency(LOAD_FREQUENCY);
    

    // Инициализация IMU

    imuSensor.setInterruptPin(INTERRUPT_PIN);
    imuSensor.begin(&Wire, 0x4B, INTERRUPT_PIN, RESET_PIN);

    // Инициализация Весла
    
    // Проверка/генерация ID весла
    uint32_t paddleId;
    paddleId = generatePaddleID();
    paddle.setPaddleID(paddleId);
    paddle.setEventHandler(&eventHandler);
    paddle.SetYAxisDirection(Y_AXIS_DIRECTION);
    // Инициализация Весла
    paddle.setIMU(&imuSensor);
//    paddle.setLoads(&rightCell, &leftCell);
    paddle.setLoads(&loadsCellSet);

 //   paddle.setLogInterface(new RGBLedInterface(4, 1, 7));
    
    Serial.printf("Paddle ID: %08X\n", paddleId);
    
    // Проверка калибровки


    if(loadsCellSet.isCalibrationValid(BladeSideType::ALL_BLADES)){
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