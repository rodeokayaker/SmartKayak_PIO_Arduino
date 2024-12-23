#include "SmartPaddle.h"
#include "SmartPaddleServer.h"
//#include "IMUSensor_GY85.h"
#include "IMUSensor_GY87.h"
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


// Определения пинов
constexpr int RIGHT_LOADCELL_DOUT_PIN = 2;
constexpr int RIGHT_LOADCELL_SCK_PIN = 3;
constexpr int LEFT_LOADCELL_DOUT_PIN = 5;
constexpr int LEFT_LOADCELL_SCK_PIN = 6;
constexpr int I2C_SDA = 9;
constexpr int I2C_SCL = 10;
constexpr int INTERRUPT_PIN = -1;
constexpr int POWER_PIN = 1;
constexpr int SWITCH_OFF_PIN = 4;

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
SmartPaddleBLEServer paddle("SmartPaddle",IMU_FREQUENCY); //  работаем как сервер
IMUSensor_GY87 imuSensor("IMU_PADDLE_MAIN"); // Адрес флага калибровки IMU

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


static uint32_t last_load_time = millis();
static float frequency_load = 10;

class switchOffButton: public ButtonDriver {
    public:
    switchOffButton(int pin): ButtonDriver(pin) {}
    void onLongPress() override {
        Serial.println("Switch off button long press");
        digitalWrite(POWER_PIN, LOW);
    }
    void onRelease() override { Serial.println("Switch off button release");}
    void onPress() override { Serial.println("Switch off button press");}
};

switchOffButton OffButton(SWITCH_OFF_PIN);


// Задача чтения тензодатчиков
void loadCellTask(void *pvParameters) {

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/LOAD_FREQUENCY);
    
    while(1) {
        int32_t time_diff = millis()-last_load_time;
        last_load_time = millis();
/*        frequency_load = 100.0/(99.0/frequency_load+time_diff/1000.0);
        Serial.printf("Load cell callback time = %d, frequency = %f\n", time_diff, frequency_load);*/
        if (paddle.connected()) {
            paddle.updateLoads();
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Задача чтения IMU

static uint32_t last_imu_time = millis();
static float frequency_imu = 98;

void imuTask(void *pvParameters) {
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/imuSensor.getFrequency());
    
    while(1) {
        int32_t time_diff = millis()-last_imu_time;
        last_imu_time = millis();
/*        frequency_imu = 100.0/(99.0/frequency_imu+time_diff/1000.0);
        Serial.printf("IMU callbck time = %d, frequency = %f\n", time_diff, frequency_imu);*/

        imuSensor.update();

        if (paddle.connected()) {
            paddle.updateIMU();
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

    paddle.getSerial()->log(cmd);
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
        imuSensor.setLogLevel(2);
    }
    else if(strcmp(cmd, CMD_LOG_LOAD) == 0)  {
        log_load = true;
    }
    else if(strcmp(cmd, CMD_LOG_STOP) == 0)  {
        log_imu = false;
        log_load = false;
        imuSensor.setLogLevel(0);

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

void bleSerialTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/BLE_SERIAL_FREQUENCY);
    
    while(1) {
        if(paddle.getSerial()) 
            paddle.getSerial()->updateJSON();
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup() {
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, HIGH);
    Serial.begin(115200);
    for (int i=0; i<200; i++) {
        digitalWrite(POWER_PIN, HIGH);  
        delay(10);
    }

    Wire.begin(I2C_SDA, I2C_SCL);
    
    
    Serial.println("\nSmart Paddle Initializing...");
    
    // Инициализация тензодатчиков
    leftCell.begin();
    rightCell.begin();

    // Инициализация IMU

    imuSensor.setInterruptPin(INTERRUPT_PIN);
    imuSensor.begin();

    // Инициализация Весла
    
    // Проверка/генерация ID весла
    uint32_t paddleId;
    paddleId = generatePaddleID();
    paddle.setPaddleID(paddleId);
    paddle.setFilterFrequency(IMU_FREQUENCY);
    paddle.setPowerPin(POWER_PIN);
    // Инициализация Весла
    paddle.begin("SmartPaddle v. 1.0");
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

    xTaskCreatePinnedToCore(
        bleSerialTask,
        "BLESerial",
        BLE_STACK_SIZE,
        NULL,
        1,
        NULL,
        1
    );
    

    setupSerialInterrupt();
    

    Serial.println("Smart Paddle Ready!");
    Serial.println("Type 'help' for available commands");
    digitalWrite(POWER_PIN, HIGH);   
    OffButton.begin();
}


void loop() {
    vTaskDelete(NULL);
} 