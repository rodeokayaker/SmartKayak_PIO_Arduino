/**
 * @file ICM20948_Example.ino
 * @brief Пример использования IMU датчика ICM-20948
 * 
 * Этот пример демонстрирует:
 * - Инициализацию ICM-20948
 * - Калибровку датчика
 * - Чтение данных с прерываниями
 * - Работу с DMP
 * - Автокалибровку и сохранение данных
 * 
 * Требования:
 * - SparkFun ICM-20948 библиотека
 * - ESP32 или Arduino с I2C поддержкой
 */

#include <Wire.h>
#include "IMUSensor_ICM20948.h"

// Пины для I2C и прерывания
#define SDA_PIN 21
#define SCL_PIN 22
#define INT_PIN 2

// Создание объекта ICM-20948
IMUSensor_ICM20948 imu("icm20948_example", ICM20948_I2C_ADDRESS, INT_PIN, &Serial);

// Флаг прерывания
volatile bool dataReady = false;

// Обработчик прерывания
void IRAM_ATTR dataReadyISR() {
    dataReady = true;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("ICM-20948 Example Starting...");
    
    // Инициализация I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000); // 400kHz
    
    // Инициализация ICM-20948
    if (!imu.begin(100, 20)) { // 100Hz IMU, 20Hz магнитометр
        Serial.println("Failed to initialize ICM-20948!");
        while (1) delay(1000);
    }
    
    Serial.println("ICM-20948 initialized successfully!");
    
    // Настройка прерывания
    if (imu.interruptPIN() >= 0) {
        pinMode(imu.interruptPIN(), INPUT);
        attachInterrupt(digitalPinToInterrupt(imu.interruptPIN()), dataReadyISR, RISING);
        Serial.printf("Interrupt configured on pin %d\n", imu.interruptPIN());
    }
    
    // Проверка существующей калибровки
    if (!imu.isCalibrationValid()) {
        Serial.println("No valid calibration found. Starting calibration...");
        Serial.println("Follow the instructions for calibration:");
        
        // Запуск калибровки
        imu.calibrate();
        
        Serial.println("Calibration complete!");
    } else {
        Serial.println("Using saved calibration data");
    }
    
    // Печать статуса калибровки
    imu.printCalibrationStatus();
    
    // Включение автокалибровки
    imu.enableAutoCalibration(true);
    
    Serial.println("Starting data acquisition...");
    Serial.println("Format: Timestamp,Q0,Q1,Q2,Q3,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,Temp");
}

void loop() {
    // Чтение данных при прерывании или каждые 10ms
    if (dataReady || (millis() % 10 == 0)) {
        dataReady = false;
        
        // Чтение данных IMU
        IMUData data = imu.readData();
        
        // Обновление ориентации
        OrientationData orientation = imu.updateOrientation();
        
        // Вывод данных в Serial
        Serial.printf("%lu,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f\n",
                      data.timestamp,
                      orientation.q0, orientation.q1, orientation.q2, orientation.q3,
                      data.ax, data.ay, data.az,
                      data.gx, data.gy, data.gz,
                      data.mx, data.my, data.mz,
                      imu.getTemperature());
        
        // Печать статуса калибровки каждые 10 секунд
        static uint32_t lastCalibCheck = 0;
        if (millis() - lastCalibCheck > 10000) {
            lastCalibCheck = millis();
            
            bool sys, gyro, accel, mag;
            imu.getCalibrationStatus(&sys, &gyro, &accel, &mag);
            
            Serial.printf("Calibration Status - Sys:%d Gyro:%d Accel:%d Mag:%d\n", 
                         sys, gyro, accel, mag);
            
            if (imu.isFullyCalibrated() && !sys) {
                Serial.println("Saving calibration data...");
                imu.saveCalibrationData();
            }
        }
        
        // Обработка команд через Serial
        if (Serial.available()) {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();
            
            if (cmd == "calib") {
                Serial.println("Starting manual calibration...");
                imu.calibrate();
                Serial.println("Calibration complete!");
            }
            else if (cmd == "calib_mag") {
                Serial.println("Starting magnetometer calibration...");
                imu.calibrateCompass();
                Serial.println("Magnetometer calibration complete!");
            }
            else if (cmd == "reset") {
                Serial.println("Resetting calibration...");
                imu.resetCalibration();
                Serial.println("Calibration reset. Restart to recalibrate.");
            }
            else if (cmd == "status") {
                imu.printCalibrationStatus();
                Serial.printf("DMP Enabled: %s\n", imu.DMPEnabled() ? "Yes" : "No");
                Serial.printf("DMP Valid: %s\n", imu.DMPValid() ? "Yes" : "No");
                Serial.printf("Auto Calibration: %s\n", imu.isAutoCalibrationEnabled() ? "Yes" : "No");
                Serial.printf("Temperature: %.2f°C\n", imu.getTemperature());
            }
            else if (cmd == "help") {
                Serial.println("Available commands:");
                Serial.println("  calib - Start full calibration");
                Serial.println("  calib_mag - Start magnetometer calibration");
                Serial.println("  reset - Reset calibration data");
                Serial.println("  status - Show status information");
                Serial.println("  help - Show this help");
            }
        }
    }
} 