// Smart Veslo by Baga
//Padel with two tenzosensors and BLE Server

// HX711 tenzosensor configuration
#include "HX711.h"
constexpr float CALIBRATION_FACTOR_L = 106.165;
constexpr float CALIBRATION_FACTOR_R = 98.777;
constexpr int RIGHT_LOADCELL_DOUT_PIN = 2;
constexpr int RIGHT_LOADCELL_SCK_PIN = 3;
constexpr int LEFT_LOADCELL_DOUT_PIN = 5;
constexpr int LEFT_LOADCELL_SCK_PIN = 6; 
HX711 scaleR;
HX711 scaleL;

#include <SmartPaddle.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_system.h"



// BLE server configuration
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#define SERVICE_UUID        "4b2de81d-c131-4636-8d56-83b83758f7ca"
#define CHARACTERISTIC_FORCE_L_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_FORCE_R_UUID "cba1d466-344c-4be3-ab3f-189f80dd7518"
#define CHARACTERISTIC_IMU_UUID "d2e5bfeb-d9f8-4b75-a295-d3f4032086ea"
BLECharacteristic *forceL;
BLECharacteristic *forceR;
BLECharacteristic *imuData;

// BLE server variables
BLEServer *pServer = NULL;
BLEService *pService = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;


//IMU configuration
#include "Wire.h"
//#include "I2Cdev.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include "MechaQMC5883.h"

ADXL345 accel;
ITG3200 gyro;
MechaQMC5883 qmc;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int mx, my, mz;
int azimuth;
 
#define I2C_SDA 8
#define I2C_SCL 9


// класс для подключения и отключения устройства
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("Connected");
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      Serial.println("Disconnected");
      deviceConnected = false;
    }
};


// calibration function
void calibrateSensors() {

  Serial.println("Before setting up the RIGHT side scale:");
  Serial.print("read average: \t");
  Serial.println(scaleR.read_average(20));  	// print the average of 20 readings from the ADC
  Serial.print("get value: \t");
  Serial.println(scaleR.get_value(5));		// print the average of 5 readings from the ADC minus the tare duty (not set yet)
  Serial.print("get units: \t");
  Serial.println(scaleR.get_units(5), 1);	// print the average of 5 readings from the ADC minus tare duty (not set) divided by the SCALE parameter (not set yet)

  scaleR.set_scale(CALIBRATION_FACTOR_R);     // this value is obtained by calibrating the scale with known dutys; see the README for details
  scaleR.tare();   // reset the scale to 0
  
  Serial.println("After setting up the RIGHT side scale:");
  Serial.print("read average: \t");
  Serial.println(scaleR.read_average(20));       // print the average of 20 readings from the ADC
  Serial.print("get value: \t");
  Serial.println(scaleR.get_value(5));		// print the average of 5 readings from the ADC minus the tare duty, set with tare()
  Serial.print("get units: \t");
  Serial.println(scaleR.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare duty, divided by the SCALE parameter set with set_scale


  Serial.println("Before setting up the LEFT side scale:");
  Serial.print("read average: \t");
  Serial.println(scaleL.read_average(20));  	// print the average of 20 readings from the ADC
  Serial.print("get value: \t");
  Serial.println(scaleL.get_value(5));		// print the average of 5 readings from the ADC minus the tare duty (not set yet)
  Serial.print("get units: \t");
  Serial.println(scaleL.get_units(5), 1);	// print the average of 5 readings from the ADC minus tare duty (not set) divided by the SCALE parameter (not set yet)

  scaleL.set_scale(CALIBRATION_FACTOR_L);     // this value is obtained by calibrating the scale with known dutys; see the README for details
  scaleL.tare();   // reset the scale to 0
  
  Serial.println("After setting up the LEFT side scale:");
  Serial.print("read average: \t");
  Serial.println(scaleL.read_average(20));       // print the average of 20 readings from the ADC
  Serial.print("get value: \t");
  Serial.println(scaleL.get_value(5));		// print the average of 5 readings from the ADC minus the tare duty, set with tare()
  Serial.print("get units: \t");
  Serial.println(scaleL.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare duty, divided by the SCALE parameter set with set_scale

}


// Calibration factor calculation function function
void calibratinFactor() {
  if (scaleR.is_ready()) {
    scaleR.set_scale();    
    Serial.println("Tare... remove any dutys from the RIGHT.");
    delay(5000);
    scaleR.tare();
    Serial.println("Tare done...");
    Serial.print("Place a known duty on the RIGHT...");
    delay(5000);
    long reading = scaleR.get_units(10);
    Serial.print("Result: ");
    Serial.println(reading);
    Serial.println("calibration factor will be the (Result)/(known duty)");
  }
  else {
    Serial.println("RIGHT HX711 not found.");
  };

  if (scaleL.is_ready()) {
    scaleL.set_scale();    
    Serial.println("Tare... remove any dutys from the LEFT.");
    delay(5000);
    scaleL.tare();
    Serial.println("Tare done...");
    Serial.print("Place a known duty on the LEFT...");
    delay(5000);
    long reading = scaleL.get_units(10);
    Serial.print("Result: ");
    Serial.println(reading);
    Serial.println("calibration factor will be the (Result)/(known duty)");
  }
  else {
    Serial.println("LEFT HX711 not found.");
  };
  delay(1000);
//calibration factor will be the (reading)/(known duty)
}

// Определения для FreeRTOS
#define SENSOR_STACK_SIZE 4096
#define BLE_STACK_SIZE 4096
#define IMU_STACK_SIZE 4096

#define LoadFrequency 100
#define IMUFrequency 40

// Очереди для обмена данными между задачами
static QueueHandle_t sensorQueue = NULL;
static QueueHandle_t imuQueue = NULL;

// Задача чтения данных с тензодатчиков
void loadSensorTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(LoadFrequency); // 100 мс интервал
    
    // Инициализация времени последнего пробуждения
    xLastWakeTime = xTaskGetTickCount();

    while(1) {
        loadData data;
        
        // Чтение данных с тензодатчиков
        data.forceR = (int32_t)scaleR.get_units(1);
        data.forceL = (int32_t)scaleL.get_units(1);
        data.timestamp = millis();
        // Отправка данных в очередь
        xQueueSend(sensorQueue, &data, portMAX_DELAY);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Задача чтения данных с IMU
void imuTask(void *pvParameters) {
    IMUData data;

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100 мс интервал
    
    // Инициализация времени последнего пробуждения
    xLastWakeTime = xTaskGetTickCount();


    while(1) {

        
        // Чтение данных с IMU
        accel.getAcceleration(&data.ax, &data.ay, &data.az);
        gyro.getRotation(&data.gx, &data.gy, &data.gz);
        qmc.read(&data.mx, &data.my, &data.mz);
        data.timestamp = millis();

        Serial.print("IMU Data: ");
        Serial.print(data.ax); Serial.print(" ");
        Serial.print(data.ay); Serial.print(" ");
        Serial.println(data.az);
        Serial.print(data.gx); Serial.print(" ");
        Serial.print(data.gy); Serial.print(" ");
        Serial.println(data.gz);
        Serial.print(data.mx); Serial.print(" ");
        Serial.print(data.my); Serial.print(" ");
        Serial.println(data.mz);

        // Отправка данных в очередь
       // xQueueSend(imuQueue, &data, portMAX_DELAY);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Задача отправки данных по BLE
void bleTask(void *pvParameters) {
    loadData loadSensorData;
    IMUData imuDataStruct;
    
    while(1) {
        if(deviceConnected) {
            // Получение данных из очередей
            if(xQueueReceive(sensorQueue, &loadSensorData, 0) == pdTRUE) {
                forceL->setValue((uint8_t*)&loadSensorData.forceL, sizeof(int32_t));
                forceR->setValue((uint8_t*)&loadSensorData.forceR, sizeof(int32_t));
                forceL->notify();
                forceR->notify();
                
                // Вывод в консоль
                Serial.print("LEFT:\t");
                Serial.print(loadSensorData.forceL);
                Serial.print("\t| RIGHT:\t");
                Serial.println(loadSensorData.forceR);
            }

            if(xQueueReceive(imuQueue, &imuDataStruct, 0) == pdTRUE) {
                imuData->setValue((uint8_t*)&imuDataStruct, sizeof(IMUData));
                imuData->notify();
                
                // Вывод в консоль
                Serial.print("IMU Data: ");
                Serial.print(imuDataStruct.ax); Serial.print(" ");
                Serial.print(imuDataStruct.ay); Serial.print(" ");
                Serial.println(imuDataStruct.az);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(300)); // Задержка 300мс
    }
}

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }

  Serial.println("Smart Padel by Baga");
  Serial.println("HX711 tenzosensor with BLE Server");

// Padel Initialization and Calibration the tenzosensors
  Serial.println("Initializing the padel");
  scaleR.begin(RIGHT_LOADCELL_DOUT_PIN, RIGHT_LOADCELL_SCK_PIN);
  scaleL.begin(LEFT_LOADCELL_DOUT_PIN, LEFT_LOADCELL_SCK_PIN);
  calibrateSensors();

// initializing IMU 9DOF
    Serial.println("Initializing I2C devices...");
    accel.initialize(); 
    qmc.init();
    gyro.initialize();

    Serial.println("Testing device connections...");
    Serial.println(gyro.testConnection() ? "ITG3200 connection successful" : "ITG3200 connection failed");
    Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");
// BLE Server initialization
  BLEDevice::init("SmartVeslo");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  pService = pServer->createService(SERVICE_UUID);
  forceL = pService->createCharacteristic(
    CHARACTERISTIC_FORCE_L_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  forceL->setValue("0");

  forceR = pService->createCharacteristic(
    CHARACTERISTIC_FORCE_R_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  forceR->setValue("0");
  
  // Создание новой характеристики для IMU
  imuData = pService->createCharacteristic(
    CHARACTERISTIC_IMU_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  imuData->setValue("0");
  
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your kayak!");
  Serial.println("Waiting for a connection...");

  // Создание очередей
  sensorQueue = xQueueCreate(10, sizeof(loadData));
  imuQueue = xQueueCreate(10, sizeof(IMUData));
  
  // Создание задач
  xTaskCreatePinnedToCore(
      loadSensorTask,          // Функция задачи
      "LoadSensorTask",        // Имя задачи
      SENSOR_STACK_SIZE,   // Размер стека
      NULL,               // Параметры
      1,                  // Приоритет
      NULL,              // Handle задачи
      0                  // Ядро CPU
  );
  
  xTaskCreatePinnedToCore(
      imuTask,
      "IMUTask",
      IMU_STACK_SIZE,
      NULL,
      1,
      NULL,
      0
  );
  
  xTaskCreatePinnedToCore(
      bleTask,
      "BLETask",
      BLE_STACK_SIZE,
      NULL,
      2,                  // Более высокий приоритет для BLE
      NULL,
      1                   // Второе ядро
  );
}

void loop() {
    // В loop() ничего не делаем, так как вся логика теперь в задачах
    vTaskDelete(NULL);
}

