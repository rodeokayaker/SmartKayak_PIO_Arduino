/**
 * @file IMUSensor_BNO055.cpp
 * @brief Реализация библиотеки для работы с IMU модулем BNO055
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#include "IMUSensor_BNO055.h"
#include <Preferences.h>

IMUSensor_BNO055::IMUSensor_BNO055(const char* prefsName, uint8_t i2cAddr, 
                                   int32_t sensorID, Stream* logStream)
    : bno(sensorID, i2cAddr),
      prefsName(prefsName),
      logStream(logStream ? logStream : &Serial),
      imuFrequency(BNO055_IMU_DEFAULT_FREQUENCY),
      magFrequency(BNO055_MAG_DEFAULT_FREQUENCY),
      interruptPin(-1),
      sensorReady(false),
      i2cAddress(i2cAddr),
      system_cal(0), gyro_cal(0), accel_cal(0), mag_cal(0),
      lastSavedCalibrationTime(0)
{
    // Инициализация структур данных
    memset(&currentData, 0, sizeof(IMUData));
    currentData.q0=1;
    memset(&currentOrientation, 0, sizeof(OrientationData));
    currentOrientation.q0=1;
    memset(&calibData, 0, sizeof(IMUCalibData));
}

bool IMUSensor_BNO055::begin(uint16_t imuFreq, uint16_t magFreq) {
    imuFrequency = imuFreq;
    magFrequency = magFreq;
    calibrationSaved = false; // Инициализируем флаг
    
    logStream->println("Initializing BNO055...");
    
    // Инициализация BNO055
    if (!bno.begin()) {
        logStream->println("No BNO055 detected. Check wiring or I2C address!");
        return false;
    }
    logStream->println("BNO055 initialized successfully");
    
    delay(1000);
    
    // Включаем внешний кристалл для лучшей точности
    bno.setExtCrystalUse(true);
    
    // Попытка загрузить сохраненные калибровочные данные
    if (readCalibrationData()) {
        logStream->println("Loading saved calibration data...");
        setSensorCalibration();
        calibrationSaved = true; // Калибровка уже сохранена
    } else {
        logStream->println("No saved calibration data found");
        // Установка значений по умолчанию для совместимости
        for(int i = 0; i < 3; i++) {
            calibData.accelScale[i] = 1.0f;  // м/с²
            calibData.gyroScale[i] = 1.0f;   // рад/с
            calibData.magScale[i] = 1.0f;    // µT
            calibData.accelOffset[i] = 0;
            calibData.gyroOffset[i] = 0;
            calibData.magOffset[i] = 0;
            calibData.magSI[i] = 0.0f;
        }    
        calibrationSaved = false; // Сброс флага сохранения
    }
    
    // Установка режима работы
    bno.setMode(OPERATION_MODE_NDOF);
    delay(100);
    
    // Проверка версии сенсора
    sensor_t sensor;
    bno.getSensor(&sensor);
    logStream->println("BNO055 initialized successfully");
    logStream->printf("Sensor: %s\n", sensor.name);
    logStream->printf("Driver Ver: %d\n", sensor.version);
    logStream->printf("Unique ID: %d\n", sensor.sensor_id);
    logStream->printf("Max Value: %f\n", sensor.max_value);
    logStream->printf("Min Value: %f\n", sensor.min_value);
    logStream->printf("Resolution: %f\n", sensor.resolution);
    
    sensorReady = true;
    
    
    return true;
}

void IMUSensor_BNO055::calibrate() {
    // BNO055 имеет автоматическую калибровку
    logStream->println("BNO055 has automatic calibration. No manual calibration needed.");
    logStream->println("Move the sensor in figure-8 motion for magnetometer calibration.");
    logStream->println("Keep device stable for accelerometer and gyroscope calibration.");
    
    // Ждем улучшения калибровки
    uint32_t startTime = millis();
    while (millis() - startTime < 30000) { // 30 секунд
        getCalibrationStatus(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
        
        if (system_cal >= 2 && gyro_cal >= 2 && accel_cal >= 2 && mag_cal >= 2) {
            logStream->println("Calibration completed successfully!");
            break;
        }
        
        if ((millis() - startTime) % 2000 == 0) {
            printCalibrationStatus();
        }
        delay(100);
    }
}

void IMUSensor_BNO055::calibrateCompass() {
    // BNO055 имеет автоматическую калибровку компаса
    logStream->println("BNO055 compass calibration is automatic.");
    logStream->println("Move the sensor in figure-8 motion for best results.");
}

IMUData IMUSensor_BNO055::readData() {
    if (!sensorReady) {
        return currentData;
    }
    
    // Получение кватерниона
    imu::Quaternion quat = bno.getQuat();
    currentData.q0 = quat.w();
    currentData.q1 = quat.x();
    currentData.q2 = quat.y();
    currentData.q3 = quat.z();

//    Serial.printf("q: %f, %f, %f, %f\n", currentData.q0, currentData.q1, currentData.q2, currentData.q3);
    
    // Получение СЫРОГО ускорения (с гравитацией)
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    currentData.ax = accel.x();
    currentData.ay = accel.y();
    currentData.az = accel.z();
    
    // Получение угловой скорости (уже в рад/с для BNO055)
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    currentData.gx = gyro.x() * DEG_TO_RAD; // Преобразуем в рад/с
    currentData.gy = gyro.y() * DEG_TO_RAD;
    currentData.gz = gyro.z() * DEG_TO_RAD;
    
    // Получение магнитного поля (сырые данные)
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    currentData.mx = mag.x();
    currentData.my = mag.y();
    currentData.mz = mag.z();
    
    // Сырые данные магнитометра (для совместимости)
    currentData.mag_x = (int16_t)mag.x();
    currentData.mag_y = (int16_t)mag.y();
    currentData.mag_z = (int16_t)mag.z();
    
    currentData.timestamp = millis();
    
    // Проверка и сохранение калибровки при полной калибровке
    checkAndSaveCalibration();
    
    return currentData;
}

IMUData IMUSensor_BNO055::getData() {
    return currentData;
}

OrientationData IMUSensor_BNO055::updateOrientation() {
    // BNO055 уже предоставляет обработанную ориентацию
    currentOrientation.q0 = currentData.q0;
    currentOrientation.q1 = currentData.q1;
    currentOrientation.q2 = currentData.q2;
    currentOrientation.q3 = currentData.q3;
    currentOrientation.timestamp = millis();
    
    return currentOrientation;
}

OrientationData IMUSensor_BNO055::getOrientation() {
    return currentOrientation;
}

bool IMUSensor_BNO055::isCalibrationValid() {
    if (!sensorReady) return false;
    
    getCalibrationStatus(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
    return (system_cal >= 2 && gyro_cal >= 2 && accel_cal >= 2 && mag_cal >= 2);
}

void IMUSensor_BNO055::magnetometerUpdate() {
    // BNO055 обновляет магнитометр автоматически
    // Этот метод оставлен для совместимости с интерфейсом
}

void IMUSensor_BNO055::setCalibrationData(const IMUCalibData data, bool save) {
    // BNO055 не поддерживает внешнюю установку калибровочных данных
    // Метод оставлен для совместимости с интерфейсом
    calibData = data;
    
    if (save) {
        logStream->println("Warning: BNO055 calibration data cannot be manually set");
    }
}

IMUCalibData IMUSensor_BNO055::getCalibrationData() {
    // Возвращаем текущие значения для совместимости
    return calibData;
}

void IMUSensor_BNO055::getCalibrationStatus(uint8_t* sys, uint8_t* gyro, 
                                           uint8_t* accel, uint8_t* mag) {
    if (!sensorReady) {
        *sys = *gyro = *accel = *mag = 0;
        return;
    }
    
    bno.getCalibration(sys, gyro, accel, mag);
    system_cal = *sys;
    gyro_cal = *gyro;
    accel_cal = *accel;
    mag_cal = *mag;
}

bool IMUSensor_BNO055::isFullyCalibrated() {
    getCalibrationStatus(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
    return (system_cal == 3 && gyro_cal == 3 && accel_cal == 3 && mag_cal == 3);
}

void IMUSensor_BNO055::printCalibrationStatus() {
    getCalibrationStatus(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
    
    logStream->printf("Calibration Status: Sys=%d Gyro=%d Accel=%d Mag=%d\n",
                     system_cal, gyro_cal, accel_cal, mag_cal);
    
    if (system_cal < 3) logStream->println("- System calibration needed");
    if (gyro_cal < 3) logStream->println("- Gyroscope calibration needed (keep device stable)");
    if (accel_cal < 3) logStream->println("- Accelerometer calibration needed (place in different orientations)");
    if (mag_cal < 3) logStream->println("- Magnetometer calibration needed (move in figure-8 pattern)");
}

float IMUSensor_BNO055::getTemperature() {
    if (!sensorReady) return 0.0f;
    return bno.getTemp();
}

void IMUSensor_BNO055::checkAndSaveCalibration() {
    if (calibrationSaved&&(BNO055_CALIBRATION_SAVE_INTERVAL<0||(millis()-lastSavedCalibrationTime<BNO055_CALIBRATION_SAVE_INTERVAL)))
    { 
        return; // Уже сохранена
    }
//    logStream->println("Checking calibration");
  // Получение статуса калибровки
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  lastSavedCalibrationTime = millis();

  // Вывод статуса калибровки
  Serial.print("Статус калибровки: Сис=");
  Serial.print(system, DEC);
  Serial.print(" Гиро=");
  Serial.print(gyro, DEC);
  Serial.print(" Аксел=");
  Serial.print(accel, DEC);
  Serial.print(" Маг=");
  Serial.println(mag, DEC);

    if (isFullyCalibrated()) {
        logStream->println("Calibration is fully calibrated");
        saveCalibrationData();
        calibrationSaved = true;
    }
}

bool IMUSensor_BNO055::readCalibrationData() {
    Preferences prefs;
    if (!prefs.begin(prefsName.c_str(), true)) {
        return false;
    }
    
    // Проверяем наличие сохраненных данных
    size_t dataSize = prefs.getBytesLength("bno_calib");
    if (dataSize != sizeof(adafruit_bno055_offsets_t)) {
        prefs.end();
        return false;
    }
    
    // Читаем калибровочные данные BNO055
    size_t readSize = prefs.getBytes("bno_calib", &sensorOffsets, sizeof(adafruit_bno055_offsets_t));
    bool validFlag = prefs.getBool("calib_valid", false);
    
    prefs.end();
    
    bool success = (readSize == sizeof(adafruit_bno055_offsets_t) && validFlag);
    
    if (success) {
        // Заполняем стандартные calibData для совместимости
        calibData.accelOffset[0] = sensorOffsets.accel_offset_x;
        calibData.accelOffset[1] = sensorOffsets.accel_offset_y;
        calibData.accelOffset[2] = sensorOffsets.accel_offset_z;
        
        calibData.gyroOffset[0] = sensorOffsets.gyro_offset_x;
        calibData.gyroOffset[1] = sensorOffsets.gyro_offset_y;
        calibData.gyroOffset[2] = sensorOffsets.gyro_offset_z;
        
        calibData.magOffset[0] = sensorOffsets.mag_offset_x;
        calibData.magOffset[1] = sensorOffsets.mag_offset_y;
        calibData.magOffset[2] = sensorOffsets.mag_offset_z;
        
        // Устанавливаем масштабы как 1.0 (BNO055 уже калиброван)
        for(int i = 0; i < 3; i++) {
            calibData.accelScale[i] = sensorOffsets.accel_radius;
            calibData.gyroScale[i] = 1.0f;
            calibData.magScale[i] = sensorOffsets.mag_radius;
            calibData.magSI[i] = 0.0f; // Для BNO055 не используется
        }
        
    } 
    
    return success;
}

void IMUSensor_BNO055::saveCalibrationData() {

    // Получаем текущие калибровочные данные с сенсора
    bool getSuccess = bno.getSensorOffsets(sensorOffsets);
    if (!getSuccess) {
        return;
    }
    
    // Обновляем стандартные calibData при сохранении
    calibData.accelOffset[0] = sensorOffsets.accel_offset_x;
    calibData.accelOffset[1] = sensorOffsets.accel_offset_y;
    calibData.accelOffset[2] = sensorOffsets.accel_offset_z;
    
    calibData.gyroOffset[0] = sensorOffsets.gyro_offset_x;
    calibData.gyroOffset[1] = sensorOffsets.gyro_offset_y;
    calibData.gyroOffset[2] = sensorOffsets.gyro_offset_z;
    
    calibData.magOffset[0] = sensorOffsets.mag_offset_x;
    calibData.magOffset[1] = sensorOffsets.mag_offset_y;
    calibData.magOffset[2] = sensorOffsets.mag_offset_z;
    
    
    Preferences prefs;
    if (!prefs.begin(prefsName.c_str(), false)) {
        return;
    }
    
    // Сохраняем калибровочные данные BNO055
    size_t written = prefs.putBytes("bno_calib", &sensorOffsets, sizeof(adafruit_bno055_offsets_t));
    prefs.putBool("calib_valid", true);
    
    prefs.end();
    
}

void IMUSensor_BNO055::getSensorCalibration() {

    // Получаем калибровочные данные с BNO055
    bool success = bno.getSensorOffsets(sensorOffsets);
 }

void IMUSensor_BNO055::setSensorCalibration() {

    // Переводим сенсор в режим конфигурации для установки калибровки
    bno.setMode(OPERATION_MODE_CONFIG);
    delay(25);
    
    // Устанавливаем калибровочные данные в BNO055
    bno.setSensorOffsets(sensorOffsets);
    bno.setMode(OPERATION_MODE_NDOF);
    delay(100);
}

void IMUSensor_BNO055::resetCalibration() {
    // Сброс калибровочных данных
    memset(&sensorOffsets, 0, sizeof(adafruit_bno055_offsets_t));
    calibrationSaved = false;
    
    // Удаляем сохраненные данные
    Preferences prefs;
    if (prefs.begin(prefsName.c_str(), false)) {
        prefs.remove("bno_calib");
        prefs.putBool("calib_valid", false);
        prefs.end();
    }
    
    logStream->println("BNO055 calibration data reset");
    
    // Перезапускаем сенсор для применения сброса
    if (sensorReady) {
        bno.setMode(OPERATION_MODE_CONFIG);
        delay(25);
        bno.setMode(OPERATION_MODE_NDOF);
        delay(100);
    }
}

IMUSensor_BNO055::~IMUSensor_BNO055() {
    // Деструктор
}
