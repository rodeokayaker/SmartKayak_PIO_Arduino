#include <Arduino.h>
#include "ImuBNO08X.h"

#define SDA 21
#define SCL 22
#define INTERRUPT_PIN 5
#define RESET_PIN -1

// Создаем объект IMU
ImuBNO08X imu("bno08x_test", &Serial);
IMUData LastIMUData;

// Счётчики для вычисления частоты
volatile uint32_t orientationCount = 0;
volatile uint32_t imuDataCount = 0;
uint32_t lastStatsTime = 0;

// Последний кватернион ориентации
volatile float lastQ0 = 0, lastQ1 = 0, lastQ2 = 0, lastQ3 = 0;

// Callback для данных ориентации (ROTATION_VECTOR)
void onOrientationReceived(const OrientationData& o) {
    orientationCount++;
    lastQ0 = o.q0;
    lastQ1 = o.q1;
    lastQ2 = o.q2;
    lastQ3 = o.q3;
}

// Callback для данных IMU (GAME_ROTATION_VECTOR)
void onIMUDataReceived(const IMUData& d) {
    imuDataCount++;
    LastIMUData = d;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== BNO08X Test ===");

    Wire.begin(SDA,SCL);
    Wire.setClock(400000);

    // I2C scanner
    Serial.println("I2C scan start...");
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.printf(" - Device at 0x%02X\n", addr);
            found++;
        } else if (err == 4) {
            Serial.printf(" - 0x%02X: Unknown error\n", addr);
        }
    }
    if (found == 0) Serial.println("No I2C devices found");
    else Serial.printf("I2C scan complete: %u device(s) found\n", found);


    // Инициализация IMU
    // Параметры: i2c, адрес, INT pin, RST pin
    // Для ESP32-C3 mini: укажите реальные пины INT и RST, или -1 если не подключены
    imu.setOrientationFrequency(100);  // 100 Hz для ориентации
    imu.setIMUFrequency(10);          // 100 Hz для IMU данных

    int result = imu.begin(&Wire, 0x4B, INTERRUPT_PIN, RESET_PIN);  // INT и RST не подключены



    // Устанавливаем колбэки
    imu.onOrientation(onOrientationReceived);
    imu.onIMUData(onIMUDataReceived);
    
    if (result) {
        Serial.println("BNO08X initialized successfully");
    } else {
        Serial.println("BNO08X initialization FAILED");
        while(1) delay(1000);
    }

    // Запускаем сервисы (создаёт FreeRTOS задачу на ESP32)
    imu.startServices();
    Serial.println("Services started");
    
    lastStatsTime = millis();
}

void loop() {
    // Каждую секунду выводим статистику
    if (millis() - lastStatsTime >= 1000) {
        uint32_t elapsed = millis() - lastStatsTime;
        
        // Вычисляем частоты
        float oriFreq = (orientationCount * 1000.0f) / elapsed;
        float imuFreq = (imuDataCount * 1000.0f) / elapsed;
        
        // Выводим статистику
        Serial.println("======================");
        Serial.print("Orientation freq: ");
        Serial.print(oriFreq, 2);
        Serial.println(" Hz");
        
        Serial.print("IMU Data freq: ");
        Serial.print(imuFreq, 2);
        Serial.println(" Hz");
        
        Serial.print("Quaternion: w=");
        Serial.print(lastQ0, 4);
        Serial.print(" x=");
        Serial.print(lastQ1, 4);
        Serial.print(" y=");
        Serial.print(lastQ2, 4);
        Serial.print(" z=");
        Serial.println(lastQ3, 4);

        Serial.printf("IMUData\n");
        Serial.printf("ax=%f, ay=%f, az=%f\n",
            LastIMUData.ax, LastIMUData.ay, LastIMUData.az);
        Serial.printf("gx=%f, gy=%f, gz=%f\n",
            LastIMUData.gx, LastIMUData.gy, LastIMUData.gz);
        Serial.printf("mx=%f, my=%f, mz=%f\n",
            LastIMUData.mx, LastIMUData.my, LastIMUData.mz);
        Serial.printf("mag_x=%d, mag_y=%d, mag_z=%d\n",
            LastIMUData.mag_x, LastIMUData.mag_y, LastIMUData.mag_z);
        Serial.printf("q0=%f, q1=%f, q2=%f, q3=%f\n",
            LastIMUData.q0, LastIMUData.q1, LastIMUData.q2, LastIMUData.q3);
        Serial.printf("timestamp=%lu\n", LastIMUData.timestamp);
        
        // Сброс счётчиков
        orientationCount = 0;
        imuDataCount = 0;
        lastStatsTime = millis();
    }
    
    vTaskDelay(100);
}

