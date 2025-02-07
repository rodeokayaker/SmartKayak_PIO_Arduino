#include "SmartPaddleClient.h"

#include "Wire.h"
#include "esp_log.h"

#include "MadgwickAHRS.h"
#include "Peripherals.h"
#include "SmartKayak.h"

#include "SPI.h"
#include "SD.h"
#include "AmperikaCRLog.h"
#include <ESP32Servo.h>
#include "IMUSensor_GY87.h"

#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

#define INCLUDE_vTaskDelayUntil 1

// Define pins
#define BUTTON_PIN    18 // ESP32 pin GPIO18, which connected to power mode changing button
#define BUTTON1_PIN   17 // ESP32 pin GPIO19, which connected to button 1
#define MOTOR_PIN     32 // PWM output for motor driver
//#define REVERSE_PIN   19 // output for reverce direction PIN
#define LED_PIN       25 // PWM MOTOR Duty LED 
#define LOW_LED_PIN   23 // Green LED
#define MED_LED_PIN   25 // Yellow LED
#define HIGH_LED_PIN  26 // Red LED
#define BLUE_LED_PIN  27 // Blue LED

#define FSYNC_PIN     33 // FSYNC pin
#define INTR_PIN      35 // Interrupt pin
#define DRDY_PIN      34 // Data ready pin

#define IMU_INTERRUPT_PIN INTR_PIN

// Define PWM output and duty Cycle mode
#define PWM1_Ch    0
#define PWM1_Res   8
#define PWM1_Freq  1000
#define PWM1_HIGH  3.0 
#define PWM1_MED   2.0
#define PWM1_LOW   1.0

#define SD_MISO     12    // GPIO19
#define SD_MOSI     13    // GPIO23  
#define SD_SCK      14    // GPIO18
#define SD_CS       15     // GPIO5 (CS

#define I2C_SDA 21
#define I2C_SCL 22

#define WIRE1_SDA 21
#define WIRE1_SCL 22

#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 20
#define LCD_ROWS 4
#define LCD2_ADDRESS 0x26
#define LCD2_COLUMNS 16
#define LCD2_ROWS 2

// Создаем объекты дисплеев
hd44780_I2Cexp lcd1(LCD_ADDRESS);
hd44780_I2Cexp lcd2(LCD2_ADDRESS);

// define initial power state, mode and force threshold
const int FORCE_THRESHOLD = 20; // минимальный уровень чувствительности тензодатчиков
const int MAX_FORCE = 100; // максимальное усилие на весле
int PWM1_DutyCycle = 0;



// FreeRTOS определения
#define PROCESS_STACK_SIZE 4096
#define PROCESS_FREQUENCY 100  // Частота обработки данных в Гц
#define IMU_FREQUENCY 100
#define LOG_FREQUENCY 100
#define VIZUALIZE_FREQUENCY 30

static bool log_paddle = false;
static bool SDCardReady = false;


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
        currentMode(MOTOR_OFF), 
        greenLED(LOW_LED_PIN), 
        blueLED(MED_LED_PIN), 
        redLED(HIGH_LED_PIN) {}

    void begin(){
        ButtonDriver::begin();
        greenLED.begin();
        blueLED.begin();
        redLED.begin();
        switch(currentMode) {
            case MOTOR_LOW_POWER: greenLED.on(); blueLED.off(); redLED.off(); break;
            case MOTOR_MEDIUM_POWER: greenLED.off(); blueLED.on(); redLED.off(); break;
            case MOTOR_HIGH_POWER: greenLED.off(); blueLED.off(); redLED.on(); break;
            case MOTOR_OFF: greenLED.off(); blueLED.off(); redLED.off(); break;
        }

    }
    void onPress() override{};
    void onRelease() override;
    void onLongPress() override {}
    MotorPowerMode getMode() { return currentMode; }
};


void PowerButton::onRelease() {
    currentMode = (MotorPowerMode)((((int)currentMode+1) % 4));
    switch(currentMode) {
        case 3: greenLED.off(); blueLED.off(); redLED.on(); break;
        case 2: greenLED.off(); blueLED.on(); redLED.off(); break;
        case 1: greenLED.on(); blueLED.off(); redLED.off(); break;
        case 0: greenLED.off(); blueLED.off(); redLED.off(); break;
    }
}



class ChinaMotor: public IMotorDriver {

    int currentForce;
    Servo servo;
    int motor_pin;
    uint32_t motor_idle_start_time;

    public:
    ChinaMotor(int pin): currentForce(0), motor_pin(pin), motor_idle_start_time(0) {}

    void begin() override {
  
        // Initialize and arm the MOTOR
        servo.attach(motor_pin);
        servo.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
        delay(5000); // delay to allow the ESC to recognize the stopped signal.
          
        Serial.printf("ChinaMotor begin\n");
    }

    void setForce(int speed) override {
        if (speed == 0 && currentForce == 0) {
            return;
        }
        if ((currentForce >0 && speed<0) || (currentForce <0 && speed>0)) {
            // if force is changing direction, start idle time
            //Serial.printf("Changing direction, starting idle time\n");
            motor_idle_start_time = millis();
            servo.writeMicroseconds(1500);
            return;
        }
        if (millis() - motor_idle_start_time < 1000) {
            // keep motor idle for 1 second
            servo.writeMicroseconds(1500);
            return;
        }
        currentForce = speed;
//        Serial.printf("ChinaMotor setForce: %d\n", speed);
        int PWM1_DutyCycle = map(speed, -1000, 1000, 1000, 2000);
        if (PWM1_DutyCycle < 1000) {
            PWM1_DutyCycle = 1000;
        }
        if (PWM1_DutyCycle > 2000) {
            PWM1_DutyCycle = 2000;
        }
        servo.writeMicroseconds(PWM1_DutyCycle);
    }

    int getForce() override {
        return currentForce;
    }
};


// Глобальные объекты
//ADXL345 accel;
//ITG3200 gyro;
//MechaQMC5883 qmc;
SmartPaddleBLEClient paddle("Paddle_1");
IMUSensor_GY87 imu("imu_gy87", true, IMU_INTERRUPT_PIN);
SmartKayak kayak;
PowerButton powerButton(BUTTON_PIN);
AmperikaCRLog SDlog(SD_CS, SD_SCK, SD_MISO, SD_MOSI);
String filename = "/log.csv";
bool SD_log=false;
ChinaMotor motor(MOTOR_PIN);

class Button1: public ButtonDriver {
    LEDDriver blueLED;
    BladeSideType side;
    public:
    Button1(int pin): ButtonDriver(pin), blueLED(BLUE_LED_PIN), side(RIGHT_BLADE) {}
    void begin() {
        ButtonDriver::begin();
        blueLED.begin();
        blueLED.off();
    }
    void onRelease() override {
        if (isLongPressed()) {
            return;
        }


        if (SD_log) {
            if (SDCardReady){
                SDlog.closeFile();
            }
            SD_log=false;
            blueLED.off();
        } else {
            filename = "/log_"+String(millis())+".csv";
            if (SDCardReady){
                SDlog.closeFile();
                SDlog.setFilename(filename.c_str());
                SDlog.clearFile();
            }
            SD_log=true;
            blueLED.on();
        }

    }
    void onLongPress() override {
        kayak.calibratePaddle();

        blueLED.Blink(200, 200, 2000, SD_log?LED_MODE::LED_ON:LED_MODE::LED_OFF);

/*        if (SD_log) {
            if (SDCardReady){
                SDlog.closeFile();
            }
            SD_log=false;
            blueLED.off();
        }*/
    }
    void onPress() override {
    }
};

Button1 button1(BUTTON1_PIN);


// Задача обработки данных с весла
void processDataTask(void *pvParameters) {

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/PROCESS_FREQUENCY);
    
    while(1) {
//        paddle.updateLoads();
//        paddle.updateIMU();
        kayak.update();

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

void logTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/LOG_FREQUENCY);

    while(1) {

        if (SD_log&&SDCardReady) {
            SDlog.openFile();
            kayak.logState(&SDlog);
            SDlog.closeFile();
        }


        if (log_paddle) {
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
            
/*            if(paddle.receiveBladeData(blade)) {
                paddleData.blade = blade;
                paddleData.bladeValid = true;
                if(log_paddle) {
                    Serial.printf("Blade - Side: %d Force: %.2f\n",
                                blade.bladeSide, blade.force);
                }
            }*/
            
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void vizualizeSerialTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/VIZUALIZE_FREQUENCY);

    while(1) {
        if (SD_log) {
            kayak.logVizualizeSerial();
//            kayak.logVizualizeMag();
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
#define CMD_IMU_CALIBRATE "calib_imu"
#define CMD_MAG_CALIBRATE "calib_mag"
#define CMD_PADDLE_IMU_CALIBRATE "calib_p_imu"
#define CMD_PADDLE_LOADS_CALIBRATE "calib_p_loads"
#define CMD_PADDLE_LOADS_CALIBRATE_LEFT "calib_p_left"
#define CMD_PADDLE_LOADS_CALIBRATE_RIGHT "calib_p_right"
#define CMD_PADDLE_COMPASS_CALIBRATE "calib_p_mag"
#define CMD_PADDLE_PAIR "paddle_pair"
#define CMD_PADDLE_SHUTDOWN "shutdown"
#define CMD_PADDLE_SEND_SPECS "send_specs"
#define CMD_PADDLE_NEXT_FILE "next_file"
#define CMD_PADDLE_SD_START "sd_start"
#define CMD_PADDLE_SD_STOP "sd_stop"
#define CMD_PADDLE_TARE_LOADS "tare_loads"
#define CMD_PADDLE_CALIBRATE_BLADE_ANGLE_LEFT "calib_blade_l"
#define CMD_PADDLE_CALIBRATE_BLADE_ANGLE_RIGHT "calib_blade_r"


// Обработка команд
void processCommand(const char* cmd) {
    if(strcmp(cmd, CMD_HELP) == 0) {
        Serial.println("Available commands:");
        Serial.println("pair          - Start pairing with paddle");
        Serial.println("status        - Show current paddle status");
        Serial.println("log_start     - Start logging paddle data");
        Serial.println("log_stop      - Stop logging paddle data");
        Serial.println("calib_imu - Start IMU calibration");
        Serial.println("calib_mag - Start Magnetometer calibration");
        Serial.println("calib_p_imu - Start Paddle IMU calibration");
        Serial.println("calib_p_loads - Start Paddle loads calibration");
        Serial.println("calib_p_left - Start Paddle left load calibration");
        Serial.println("calib_p_right - Start Paddle right load calibration");
        Serial.println("calib_p_mag - Start PaddleMagnetometer calibration");
        Serial.println("calib_blade_r - Start Paddle right blade angle calibration");
        Serial.println("calib_blade_l - Start Paddle left blade angle calibration");
        Serial.println("tare_loads - Tare all loads");
        Serial.println("paddle_pair - Start Paddle pairing");
        Serial.println("send_specs - Send paddle specifications");
        Serial.println("shutdown - Shutdown Paddle");
        Serial.println("sd_start - Start SD logging");
        Serial.println("sd_stop - Stop SD logging");
        Serial.println("next_file - SD open next file");
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
        Serial.printf("Paddle ID: %s\n", specs.PaddleID);
        Serial.printf("Blade type: %s\n", 
                     specs.paddleType == TWO_BLADES ? "Double" : "Single");
        imu.PrintChipsInfo();
        imu.PrintCalibrationData();
    }
    else if(strcmp(cmd, CMD_PAIR) == 0) {
        Serial.println("Starting pairing mode...");
        paddle.startPairing();
    }
    else if(strcmp(cmd, CMD_LOG_START) == 0) {
        //log_paddle = true;
//        log_imu = true;
//        log_load = true;
        kayak.onLogLevel(SMARTKAYAK_LOG_FORCE);
        //imu.setLogLevel(1);
        Serial.println("Logging started");
    }
    else if(strcmp(cmd, CMD_LOG_STOP) == 0) {
        //log_paddle = false;
        //log_imu = false;
        //log_load = false;
        kayak.offLogLevel(SMARTKAYAK_LOG_FORCE);
        //imu.setLogLevel(0);
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
    else if(strcmp(cmd, CMD_PADDLE_PAIR) == 0) {
        paddle.getSerial()->sendCommand(SP_BLESerial_Commands::START_PAIR);
    }
    else if(strcmp(cmd, CMD_PADDLE_SHUTDOWN) == 0) {
        paddle.getSerial()->sendCommand(SP_BLESerial_Commands::SHUTDOWN);
    }
    else if(strcmp(cmd, CMD_PADDLE_SD_START) == 0) {
        SDlog.begin(filename.c_str());
        SDlog.clearFile();
        SD_log=true;
    }
    else if(strcmp(cmd, CMD_PADDLE_SD_STOP) == 0) {
        SDlog.closeFile();
        SD_log=false;
    }
    else if(strcmp(cmd, CMD_PADDLE_NEXT_FILE) == 0) {
        filename = "/log_"+String(millis())+".csv";
        SDlog.setFilename(filename.c_str());
        SDlog.clearFile();
    }
    else if(strcmp(cmd, CMD_PADDLE_SEND_SPECS) == 0) {
        if (paddle.getSerial()) {
            paddle.getSerial()->sendCommand(SP_BLESerial_Commands::SEND_SPECS);
        }
    }
    else if(strcmp(cmd, CMD_PADDLE_COMPASS_CALIBRATE) == 0) {
        if (paddle.getSerial()) {
            paddle.getSerial()->sendCommand(SP_BLESerial_Commands::CALIBRATE_COMPASS);
        }
    }
    else if(strcmp(cmd, CMD_MAG_CALIBRATE) == 0) {
        imu.calibrateCompass();
    }
    else if(strcmp(cmd, CMD_PADDLE_CALIBRATE_BLADE_ANGLE_LEFT) == 0) {
        paddle.calibrateBladeAngle(LEFT_BLADE);
    }
    else if(strcmp(cmd, CMD_PADDLE_CALIBRATE_BLADE_ANGLE_RIGHT) == 0) {
        paddle.calibrateBladeAngle(RIGHT_BLADE);
    }
    else if(strcmp(cmd, CMD_PADDLE_TARE_LOADS) == 0) {
        if (paddle.getSerial()) {
            JsonDocument doc;
            JsonObject params=doc.to<JsonObject>();
            params[SP_BLESerial_Commands::BLADE_SIDE_PARAM] = ALL_BLADES;
            paddle.getSerial()->sendCommand(SP_BLESerial_Commands::TARE_LOADS, &params);    
        }
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

class PaddleEventHandler: public SP_Event_Handler {
    private:
    float imu_freq;
    float load_freq;
    uint32_t last_imu_ts;
    uint32_t last_load_ts;
    public:
    PaddleEventHandler(): imu_freq(100), load_freq(10), last_imu_ts(0), last_load_ts(0) {}
    void onUpdateIMU(IMUData& imuData, SmartPaddle* paddle) override {
        uint32_t dt = imuData.timestamp - last_imu_ts;
        if (dt > 0) {
            imu_freq = 0.001 * (1000.0/dt) + 0.999 * imu_freq;
        } else {
            Serial.printf("IMU timestamp error: %d\n", dt);
        }
        last_imu_ts = imuData.timestamp;
//        Serial.printf("IMU Freq:  %.2f\n",imu_freq);
    }
    void onUpdateLoad(loadData& ld, SmartPaddle* paddle) override {
        uint32_t dt = ld.timestamp - last_load_ts;
        if (dt > 0) {
            load_freq = 0.001 * (1000.0/dt) + 0.999 * load_freq;
        } else {
            Serial.printf("Load timestamp error: %d\n", dt);
        }
        last_load_ts = ld.timestamp;
//        Serial.printf("Load Freq:  %.2f\n",                     load_freq);
    }
    void onConnect(SmartPaddle* paddle) override {
        lcd1.clear();
        lcd1.setCursor(0, 0);
        lcd1.print("Smart Kayak");
        lcd1.setCursor(0, 2);
        lcd1.print("Connected");
        lcd2.clear();
        lcd2.setCursor(0, 0);
        lcd2.print("Smart Kayak");
        lcd2.setCursor(0, 1);
        lcd2.print("Connected");
    }

    void onDisconnect(SmartPaddle* paddle) override {
        Serial.printf("DisconnectedON\n");
        lcd1.clear();
        lcd1.setCursor(0, 0);
        lcd1.print("Smart Kayak");
        lcd1.setCursor(0, 2);
        lcd1.print("Waiting 4 connect...");
        lcd2.clear();
        lcd2.setCursor(0, 0);
        lcd2.print("Smart Kayak");
        lcd2.setCursor(0, 1);
        lcd2.print("Disconnected");
    }
};

PaddleEventHandler paddleEventHandler;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    // Инициализация дисплеев
    int status;
    
    // Инициализация первого дисплея
    status = lcd1.begin(LCD_COLUMNS, LCD_ROWS);
    if(status) {
        Serial.print("LCD 1 initialization failed: ");
        Serial.println(status);
    }
    
    // Инициализация второго дисплея
    status = lcd2.begin(LCD2_COLUMNS, LCD2_ROWS, LCD2_ADDRESS);
    if(status) {
        Serial.print("LCD 2 initialization failed: ");
        Serial.println(status);
    }

    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("Smart Kayak");
    lcd2.clear();
    lcd2.setCursor(0, 0);
    lcd2.print("Smart Kayak");
    lcd1.setCursor(0, 2);
    lcd1.print("Loading...");
    
    delay(1000);
    lcd1.setCursor(0, 3);
    lcd1.print("motor");
    motor.begin();

    Wire.begin(I2C_SDA, I2C_SCL);
    filename = "/log_"+String(millis())+".csv";
    SDCardReady = SDlog.begin(filename.c_str());
    SDlog.clearFile();

    lcd1.setCursor(0, 3);
    lcd1.print("paddle");   
    Serial.println("\nKayak Smart System Initializing...");

    // Инициализация BLE клиента
    paddle.setEventHandler(&paddleEventHandler);
    paddle.begin("SmartKayak 1.0");
    imu.setFrequency(IMU_FREQUENCY);
    imu.setInterruptPin(IMU_INTERRUPT_PIN);
    imu.begin();

//    imu.setAutoCalibrateMag(true);
    kayak.setPaddle(&paddle);
    kayak.setTextLCD(&lcd1, &lcd2);
    kayak.setIMU(&imu, IMU_FREQUENCY);
    kayak.setModeSwitch(&powerButton);
    kayak.setMotorDriver(&motor);
    kayak.begin();
    button1.begin();
    powerButton.begin();

    // Создание задач
    
    xTaskCreatePinnedToCore(
        processDataTask,
        "Process",
        PROCESS_STACK_SIZE,
        NULL,
        2,
        NULL,
        1  // Обработка на ядре 2
    );
    
    xTaskCreatePinnedToCore(
        serialCommandTask,
        "SerialCmd",
        4096,
        NULL,
        1,
        NULL,
        0  // Команды на ядре 1
    );
 

    xTaskCreatePinnedToCore(
        logTask,
        "Log",
        4096,
        NULL,
        1,
        NULL,
        0  
    );

    xTaskCreatePinnedToCore(
        imuTask,
        "IMU",
        4096,
        NULL,
        1,
        NULL,
        0  
    );

    xTaskCreatePinnedToCore(
        vizualizeSerialTask,
        "Vizualize",
        4096,
        NULL,
        1,
        NULL,
        0  
    );


    Serial.println("Kayak Smart System Ready!");
    Serial.println("Type 'help' for available commands");
    Serial.print("Smart Kayak > ");
    lcd1.setCursor(0, 2);
    lcd1.print("Waiting for");
    lcd1.setCursor(0, 3);
    lcd1.print("connection...");
}

void loop() {
    vTaskDelete(NULL);
} 