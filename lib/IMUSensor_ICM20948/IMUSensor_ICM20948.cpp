/**
 * @file IMUSensor_ICM20948.cpp
 * @brief Реализация библиотеки для работы с IMU модулем ICM-20948
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#include "IMUSensor_ICM20948.h"
#include <Preferences.h>

IMUSensor_ICM20948::IMUSensor_ICM20948(const char* prefsName, uint8_t i2cAddr, 
                                       int interruptPin, Stream* logStream)
    : prefsName(prefsName),
      logStream(logStream ? logStream : &Serial),
      imuFrequency(ICM20948_IMU_DEFAULT_FREQUENCY),
      magFrequency(ICM20948_MAG_DEFAULT_FREQUENCY),
      interruptPin(interruptPin),
      sensorReady(false),
      i2cAddress(i2cAddr),
      calibrationSaved(false),
      lastSavedCalibrationTime(0),
      dmpEnabled(false),
      dmpValid(false),
      gyroCalibrated(false),
      accelCalibrated(false),
      magCalibrated(false),
      systemCalibrated(false),
      calibrationInProgress(false),
      calibrationStartTime(0)
{
    // Инициализация структур данных
    memset(&currentData, 0, sizeof(IMUData));
    currentData.q0 = 1;
    memset(&currentOrientation, 0, sizeof(OrientationData));
    currentOrientation.q0 = 1;
    memset(&calibData, 0, sizeof(IMUCalibData));
    memset(&icmCalibData, 0, sizeof(ICM20948CalibData));
}

bool IMUSensor_ICM20948::begin(uint16_t imuFreq, uint16_t magFreq) {
    imuFrequency = imuFreq;
    magFrequency = magFreq;
    calibrationSaved = false;
    
    logStream->println("Initializing ICM-20948...");
    
    // Инициализация ICM-20948
    icm.begin(Wire, i2cAddress);
    
    if (icm.status != ICM_20948_Stat_Ok) {
        logStream->println("No ICM-20948 detected. Check wiring or I2C address!");
        return false;
    }
    
    logStream->println("ICM-20948 initialized successfully");
    
    delay(100);
    
    // Установка диапазонов
    ICM_20948_fss_t myFSS;
    myFSS.a = gpm4;   // ±4g для акселерометра
    myFSS.g = dps500; // ±500°/s для гироскопа
    
    icm.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
    delay(1); // Задержка для корректной установки настроек
    
    // Установка частоты дискретизации и фильтров
    ICM_20948_dlpcfg_t myDLPcfg;
    myDLPcfg.a = acc_d111bw4_n136bw;   // Фильтр для акселерометра
    myDLPcfg.g = gyr_d119bw5_n154bw3;  // Фильтр для гироскопа
    
    icm.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
    delay(1);
    
    // Включение магнитометра
    icm.startupMagnetometer();
    
    // Попытка загрузить сохраненные калибровочные данные
    if (readCalibrationData()) {
        logStream->println("Loading saved calibration data...");
        applySensorCalibration();
        calibrationSaved = true;
    } else {
        logStream->println("No saved calibration data found");
        // Установка значений по умолчанию
        for(int i = 0; i < 3; i++) {
            calibData.accelScale[i] = 9.81f / 8192.0f;  // для ±4g
            calibData.gyroScale[i] = (DEG_TO_RAD / 65.5f); // для ±500°/s
            calibData.magScale[i] = 1.0f;
            calibData.accelOffset[i] = 0;
            calibData.gyroOffset[i] = 0;
            calibData.magOffset[i] = 0;
            calibData.magSI[i] = 0.0f;
        }
        calibrationSaved = false;
    }
    
    // Инициализация DMP
    if (initializeDMP()) {
        logStream->println("DMP initialized successfully");
        dmpEnabled = true;
    } else {
        logStream->println("DMP initialization failed, using raw data");
        dmpEnabled = false;
    }
    
    // Включение автокалибровки
    enableAutoCalibration(true);
    
    // Настройка прерываний если указан пин
    if (interruptPin >= 0) {
        // Конфигурация прерываний (как в Example3_Interrupts)
        icm.cfgIntActiveLow(true);  // Active low для совместимости с pull-up резистором
        icm.cfgIntOpenDrain(false); // Push-pull 
        icm.cfgIntLatch(true);      // Защелкивание прерывания до очистки
        
        if (dmpEnabled) {
            // Включаем прерывание DMP
            icm.intEnableDMP(true);
        } else {
            // Включаем прерывание готовности данных
            icm.intEnableRawDataReady(true);
        }
        
        logStream->printf("Interrupts configured on pin %d\n", interruptPin);
    }
    
    sensorReady = true;
    
    logStream->printf("ICM-20948 ready. Frequency: %d Hz, Mag: %d Hz\n", 
                     imuFrequency, magFrequency);
    
    return true;
}

bool IMUSensor_ICM20948::initializeDMP() {
    // Инициализация DMP (как в Example6_DMP_Quat9_Orientation)
    bool success = true;
    
    // Инициализация DMP
    success &= (icm.initializeDMP() == ICM_20948_Stat_Ok);
    
    // Включение DMP сенсоров
    success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
    success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
    success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
    success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);
    
    // Установка частоты DMP (максимальная)
    success &= (icm.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok);
    
    // Включение FIFO
    success &= (icm.enableFIFO() == ICM_20948_Stat_Ok);
    
    // Включение DMP
    success &= (icm.enableDMP() == ICM_20948_Stat_Ok);
    
    // Сброс DMP
    success &= (icm.resetDMP() == ICM_20948_Stat_Ok);
    
    // Сброс FIFO
    success &= (icm.resetFIFO() == ICM_20948_Stat_Ok);
    
    return success;
}

void IMUSensor_ICM20948::calibrate() {
    logStream->println("Starting ICM-20948 DMP calibration...");
    logStream->println("Keep device still for 30 seconds, then move in all orientations for 2 minutes");
    
    // Сброс предыдущих биасов
    resetCalibration();
    
    // DMP будет автоматически калибровать биасы во время работы
    // Мы просто ждем, пока DMP накопит достаточно данных
    calibrationStartTime = millis();
    calibrationInProgress = true;
    
    logStream->println("DMP calibration started. This will take about 2 minutes...");
    logStream->println("Move the device in figure-8 patterns and all orientations");
}

void IMUSensor_ICM20948::calibrateCompass() {
    logStream->println("ICM-20948 magnetometer calibration started");
    logStream->println("Rotate device in figure-8 patterns for 30 seconds");
    
    // DMP автоматически калибрует магнитометр
    // Мы просто уведомляем пользователя
    delay(30000); // 30 секунд для калибровки
    
    // Проверяем биасы магнитометра
    updateCalibrationStatus();
    
    if (magCalibrated) {
        logStream->println("Magnetometer calibration completed successfully");
    } else {
        logStream->println("Continue moving device for better magnetometer calibration");
    }
}

IMUData IMUSensor_ICM20948::readData() {
    if (!sensorReady) {
    //    Serial.println("IMU not ready");
        return currentData;

    }
    
    dmpValid = false;
    
    if (dmpEnabled) {
        updateFromDMP();
    } else {
        readRawSensorData();
    }
    
    scaleAndCalibrateData();
    
    currentData.timestamp = millis();
    
    // Очистка прерываний (как в Example3_Interrupts)
    if (interruptPin >= 0) {
        icm.clearInterrupts();
    }
    
    // Проверка и сохранение калибровки
    checkAndSaveCalibration();
    
    return currentData;
}

void IMUSensor_ICM20948::updateFromDMP() {
    icm_20948_DMP_data_t data;
    icm.readDMPdataFromFIFO(&data);
    
    if (icm.status == ICM_20948_Stat_Ok && (data.header & DMP_header_bitmap_Quat9) > 0) {
        // Кватернион из DMP
        double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
        double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
        double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
        double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
        
        currentData.q0 = q0;
        currentData.q1 = q1;
        currentData.q2 = q2;
        currentData.q3 = q3;
        
        dmpValid = true;
    }
    
    // Сырые данные акселерометра
    if ((data.header & DMP_header_bitmap_Accel) > 0) {
        currentData.ax = data.Raw_Accel.Data.X;
        currentData.ay = data.Raw_Accel.Data.Y;
        currentData.az = data.Raw_Accel.Data.Z;
    }
    
    // Сырые данные гироскопа
    if ((data.header & DMP_header_bitmap_Gyro) > 0) {
        currentData.gx = data.Raw_Gyro.Data.X;
        currentData.gy = data.Raw_Gyro.Data.Y;
        currentData.gz = data.Raw_Gyro.Data.Z;
    }
    
    // Данные магнитометра
    if ((data.header & DMP_header_bitmap_Compass) > 0) {
        currentData.mx = data.Compass.Data.X;
        currentData.my = data.Compass.Data.Y;
        currentData.mz = data.Compass.Data.Z;
        
        currentData.mag_x = (int16_t)data.Compass.Data.X;
        currentData.mag_y = (int16_t)data.Compass.Data.Y;
        currentData.mag_z = (int16_t)data.Compass.Data.Z;
    }
}

void IMUSensor_ICM20948::readRawSensorData() {
    icm.getAGMT();
    if (icm.status == ICM_20948_Stat_Ok) {
        // Сырые данные
        currentData.ax = icm.accX();
        currentData.ay = icm.accY();
        currentData.az = icm.accZ();
        
        currentData.gx = icm.gyrX();
        currentData.gy = icm.gyrY();
        currentData.gz = icm.gyrZ();
        
        currentData.mx = icm.magX();
        currentData.my = icm.magY();
        currentData.mz = icm.magZ();
        
        currentData.mag_x = (int16_t)icm.magX();
        currentData.mag_y = (int16_t)icm.magY();
        currentData.mag_z = (int16_t)icm.magZ();
    }
}

void IMUSensor_ICM20948::scaleAndCalibrateData() {
    // Применение калибровки и масштабирования
    if (!dmpEnabled || !dmpValid) {
        // Калибровка акселерометра
        currentData.ax = (currentData.ax - calibData.accelOffset[0]) * calibData.accelScale[0];
        currentData.ay = (currentData.ay - calibData.accelOffset[1]) * calibData.accelScale[1];
        currentData.az = (currentData.az - calibData.accelOffset[2]) * calibData.accelScale[2];
        
        // Калибровка гироскопа
        currentData.gx = (currentData.gx - calibData.gyroOffset[0]) * calibData.gyroScale[0];
        currentData.gy = (currentData.gy - calibData.gyroOffset[1]) * calibData.gyroScale[1];
        currentData.gz = (currentData.gz - calibData.gyroOffset[2]) * calibData.gyroScale[2];
    }
    
    // Калибровка магнитометра
    currentData.mx = (currentData.mx - calibData.magOffset[0]) * calibData.magScale[0];
    currentData.my = (currentData.my - calibData.magOffset[1]) * calibData.magScale[1];
    currentData.mz = (currentData.mz - calibData.magOffset[2]) * calibData.magScale[2];
}

IMUData IMUSensor_ICM20948::getData() {
    return currentData;
}

OrientationData IMUSensor_ICM20948::updateOrientation() {
    // ICM-20948 уже предоставляет кватернион через DMP
    currentOrientation.q0 = currentData.q0;
    currentOrientation.q1 = currentData.q1;
    currentOrientation.q2 = currentData.q2;
    currentOrientation.q3 = currentData.q3;
    currentOrientation.timestamp = millis();
    
    return currentOrientation;
}

OrientationData IMUSensor_ICM20948::getOrientation() {
    return currentOrientation;
}

bool IMUSensor_ICM20948::isCalibrationValid() {
    return sensorReady && (gyroCalibrated || accelCalibrated || magCalibrated);
}

void IMUSensor_ICM20948::magnetometerUpdate() {
    // ICM-20948 обновляет магнитометр автоматически
    // Этот метод оставлен для совместимости с интерфейсом
    performAutoCalibration();
}

void IMUSensor_ICM20948::setCalibrationData(const IMUCalibData data, bool save) {
    calibData = data;
    
    // Копирование в ICM специфическую структуру
    for (int i = 0; i < 3; i++) {
        icmCalibData.gyroBias[i] = data.gyroOffset[i];
        icmCalibData.accelBias[i] = data.accelOffset[i];
        icmCalibData.magBias[i] = data.magOffset[i];
        icmCalibData.magScale[i] = data.magScale[i];
    }
    
    applySensorCalibration();
    
    if (save) {
        saveCalibrationData();
        calibrationSaved = true;
    }
}

IMUCalibData IMUSensor_ICM20948::getCalibrationData() {
    return calibData;
}

void IMUSensor_ICM20948::getCalibrationStatus(bool* sys, bool* gyro, bool* accel, bool* mag) {
    // Проверяем реальные биасы DMP (как в Example11)
    updateCalibrationStatus();
    
    *sys = systemCalibrated;
    *gyro = gyroCalibrated;
    *accel = accelCalibrated;
    *mag = magCalibrated;
}

bool IMUSensor_ICM20948::isFullyCalibrated() {
    updateCalibrationStatus();
    return gyroCalibrated && accelCalibrated && magCalibrated;
}

void IMUSensor_ICM20948::printCalibrationStatus() {
    logStream->printf("Calibration Status: Sys=%d Gyro=%d Accel=%d Mag=%d\n",
                     systemCalibrated, gyroCalibrated, accelCalibrated, magCalibrated);
    
    if (!systemCalibrated) logStream->println("- System calibration needed");
    if (!gyroCalibrated) logStream->println("- Gyroscope calibration needed (keep device stable)");
    if (!accelCalibrated) logStream->println("- Accelerometer calibration needed (place in different orientations)");
    if (!magCalibrated) logStream->println("- Magnetometer calibration needed (move in figure-8 pattern)");
}

float IMUSensor_ICM20948::getTemperature() {
    if (!sensorReady) return 0.0f;
    
    icm.getAGMT();
    if (icm.status == ICM_20948_Stat_Ok) {
        return icm.temp();
    }
    return 0.0f;
}

int8_t IMUSensor_ICM20948::getIntStatus() {
    if (!sensorReady) return 0;
    
    // Проверяем готовность данных (как в примерах SparkFun)
    if (icm.dataReady()) {
        return 1; // Данные готовы
    }
    
    return 0; // Данные не готовы
}

void IMUSensor_ICM20948::updateCalibrationStatus() {
    if (!sensorReady || !dmpEnabled) {
        gyroCalibrated = false;
        accelCalibrated = false;
        magCalibrated = false;
        systemCalibrated = false;
        return;
    }
    
    // Проверяем биасы DMP (как в Example11_DMP_Bias_Save_Restore_ESP32)
    int32_t biasGyroX, biasGyroY, biasGyroZ;
    int32_t biasAccelX, biasAccelY, biasAccelZ;
    int32_t biasCPassX, biasCPassY, biasCPassZ;
    
    bool success = true;
    success &= (icm.getBiasGyroX(&biasGyroX) == ICM_20948_Stat_Ok);
    success &= (icm.getBiasGyroY(&biasGyroY) == ICM_20948_Stat_Ok);
    success &= (icm.getBiasGyroZ(&biasGyroZ) == ICM_20948_Stat_Ok);
    success &= (icm.getBiasAccelX(&biasAccelX) == ICM_20948_Stat_Ok);
    success &= (icm.getBiasAccelY(&biasAccelY) == ICM_20948_Stat_Ok);
    success &= (icm.getBiasAccelZ(&biasAccelZ) == ICM_20948_Stat_Ok);
    success &= (icm.getBiasCPassX(&biasCPassX) == ICM_20948_Stat_Ok);
    success &= (icm.getBiasCPassY(&biasCPassY) == ICM_20948_Stat_Ok);
    success &= (icm.getBiasCPassZ(&biasCPassZ) == ICM_20948_Stat_Ok);
    
    if (success) {
        // Проверяем разумные значения биасов
        gyroCalibrated = (abs(biasGyroX) < 100000 && abs(biasGyroY) < 100000 && abs(biasGyroZ) < 100000);
        accelCalibrated = (abs(biasAccelX) < 1000000 && abs(biasAccelY) < 1000000 && abs(biasAccelZ) < 1000000);
        magCalibrated = (abs(biasCPassX) < 10000 && abs(biasCPassY) < 10000 && abs(biasCPassZ) < 10000);
        
        // Сохраняем биасы в стандартную структуру для совместимости
        calibData.gyroOffset[0] = biasGyroX;
        calibData.gyroOffset[1] = biasGyroY;
        calibData.gyroOffset[2] = biasGyroZ;
        calibData.accelOffset[0] = biasAccelX;
        calibData.accelOffset[1] = biasAccelY;
        calibData.accelOffset[2] = biasAccelZ;
        calibData.magOffset[0] = biasCPassX;
        calibData.magOffset[1] = biasCPassY;
        calibData.magOffset[2] = biasCPassZ;
    } else {
        gyroCalibrated = false;
        accelCalibrated = false;
        magCalibrated = false;
    }
    
    systemCalibrated = gyroCalibrated && accelCalibrated && magCalibrated;
    
    // Проверяем процесс калибровки
    if (calibrationInProgress) {
        uint32_t elapsed = millis() - calibrationStartTime;
        if (elapsed > 120000) { // 2 минуты
            calibrationInProgress = false;
            if (systemCalibrated) {
                logStream->println("DMP calibration completed successfully!");
                saveCalibrationData();
                calibrationSaved = true;
            } else {
                logStream->println("DMP calibration incomplete. Continue moving device...");
            }
        }
    }
}

void IMUSensor_ICM20948::checkAndSaveCalibration() {
    updateCalibrationStatus();
    
    if (calibrationSaved && 
        (ICM20948_CALIBRATION_SAVE_INTERVAL < 0 || 
         (millis() - lastSavedCalibrationTime < ICM20948_CALIBRATION_SAVE_INTERVAL))) {
        return; // Уже сохранена или еще рано
    }
    
    lastSavedCalibrationTime = millis();
    
    if (isFullyCalibrated()) {
        saveCalibrationData();
        calibrationSaved = true;
    }
}

void IMUSensor_ICM20948::performAutoCalibration() {
    // Простая автокалибровка для небольших дрифтов
    static int calibCount = 0;
    static float gyroSum[3] = {0, 0, 0};
    
    if (!gyroCalibrated) return;
    
    calibCount++;
    
    // Каждые 1000 измерений обновляем смещение гироскопа
    if (calibCount >= 1000) {
        for (int i = 0; i < 3; i++) {
            float avgDrift = gyroSum[i] / 1000.0f;
            
            // Обновляем только если дрифт небольшой
            if (abs(avgDrift) < 0.1f) {
                calibData.gyroOffset[i] += (int16_t)(avgDrift * 1000);
                icmCalibData.gyroBias[i] = calibData.gyroOffset[i];
            }
            
            gyroSum[i] = 0;
        }
        calibCount = 0;
    } else {
        // Накапливаем данные для усреднения
        gyroSum[0] += currentData.gx;
        gyroSum[1] += currentData.gy;
        gyroSum[2] += currentData.gz;
    }
}

void IMUSensor_ICM20948::applySensorCalibration() {
    // Применение калибровочных данных к датчику
    // ICM-20948 не поддерживает прямую установку смещений через регистры
    // Калибровка применяется программно в scaleAndCalibrateData()
}

void IMUSensor_ICM20948::enableAutoCalibration(bool enable) {
    // Включение/отключение автокалибровки
    // Для ICM-20948 это означает включение/отключение performAutoCalibration()
    gyroCalibrated = enable;
}

bool IMUSensor_ICM20948::isAutoCalibrationEnabled() {
    return gyroCalibrated;
}

bool IMUSensor_ICM20948::readCalibrationData() {
    Preferences prefs;
    if (!prefs.begin(prefsName.c_str(), true)) {
        return false;
    }
    
    // Читаем биасы DMP как в Example11
    int32_t biasGyroX = prefs.getLong("biasGyroX", 0);
    int32_t biasGyroY = prefs.getLong("biasGyroY", 0);
    int32_t biasGyroZ = prefs.getLong("biasGyroZ", 0);
    int32_t biasAccelX = prefs.getLong("biasAccelX", 0);
    int32_t biasAccelY = prefs.getLong("biasAccelY", 0);
    int32_t biasAccelZ = prefs.getLong("biasAccelZ", 0);
    int32_t biasCPassX = prefs.getLong("biasCPassX", 0);
    int32_t biasCPassY = prefs.getLong("biasCPassY", 0);
    int32_t biasCPassZ = prefs.getLong("biasCPassZ", 0);
    bool validFlag = prefs.getBool("biases_valid", false);
    
    prefs.end();
    
    if (validFlag && sensorReady && dmpEnabled) {
        // Применяем биасы к DMP
        bool success = true;
        success &= (icm.setBiasGyroX(biasGyroX) == ICM_20948_Stat_Ok);
        success &= (icm.setBiasGyroY(biasGyroY) == ICM_20948_Stat_Ok);
        success &= (icm.setBiasGyroZ(biasGyroZ) == ICM_20948_Stat_Ok);
        success &= (icm.setBiasAccelX(biasAccelX) == ICM_20948_Stat_Ok);
        success &= (icm.setBiasAccelY(biasAccelY) == ICM_20948_Stat_Ok);
        success &= (icm.setBiasAccelZ(biasAccelZ) == ICM_20948_Stat_Ok);
        success &= (icm.setBiasCPassX(biasCPassX) == ICM_20948_Stat_Ok);
        success &= (icm.setBiasCPassY(biasCPassY) == ICM_20948_Stat_Ok);
        success &= (icm.setBiasCPassZ(biasCPassZ) == ICM_20948_Stat_Ok);
        
        if (success) {
            // Обновляем статус калибровки
            updateCalibrationStatus();
            logStream->println("DMP biases restored from memory");
            return true;
        }
    }
    
    return false;
}

void IMUSensor_ICM20948::saveCalibrationData() {
    if (!sensorReady || !dmpEnabled) {
        return;
    }
    
    // Получаем текущие биасы DMP (как в Example11)
    int32_t biasGyroX, biasGyroY, biasGyroZ;
    int32_t biasAccelX, biasAccelY, biasAccelZ;
    int32_t biasCPassX, biasCPassY, biasCPassZ;
    
    bool success = true;
    success &= (icm.getBiasGyroX(&biasGyroX) == ICM_20948_Stat_Ok);
    success &= (icm.getBiasGyroY(&biasGyroY) == ICM_20948_Stat_Ok);
    success &= (icm.getBiasGyroZ(&biasGyroZ) == ICM_20948_Stat_Ok);
    success &= (icm.getBiasAccelX(&biasAccelX) == ICM_20948_Stat_Ok);
    success &= (icm.getBiasAccelY(&biasAccelY) == ICM_20948_Stat_Ok);
    success &= (icm.getBiasAccelZ(&biasAccelZ) == ICM_20948_Stat_Ok);
    success &= (icm.getBiasCPassX(&biasCPassX) == ICM_20948_Stat_Ok);
    success &= (icm.getBiasCPassY(&biasCPassY) == ICM_20948_Stat_Ok);
    success &= (icm.getBiasCPassZ(&biasCPassZ) == ICM_20948_Stat_Ok);
    
    if (success) {
        Preferences prefs;
        if (prefs.begin(prefsName.c_str(), false)) {
            // Сохраняем биасы DMP
            prefs.putLong("biasGyroX", biasGyroX);
            prefs.putLong("biasGyroY", biasGyroY);
            prefs.putLong("biasGyroZ", biasGyroZ);
            prefs.putLong("biasAccelX", biasAccelX);
            prefs.putLong("biasAccelY", biasAccelY);
            prefs.putLong("biasAccelZ", biasAccelZ);
            prefs.putLong("biasCPassX", biasCPassX);
            prefs.putLong("biasCPassY", biasCPassY);
            prefs.putLong("biasCPassZ", biasCPassZ);
            prefs.putBool("biases_valid", true);
            
            prefs.end();
            
            logStream->println("DMP biases saved successfully");
            logStream->printf("Gyro: [%ld, %ld, %ld]\n", biasGyroX, biasGyroY, biasGyroZ);
            logStream->printf("Accel: [%ld, %ld, %ld]\n", biasAccelX, biasAccelY, biasAccelZ);
            logStream->printf("Compass: [%ld, %ld, %ld]\n", biasCPassX, biasCPassY, biasCPassZ);
        }
    }
}

void IMUSensor_ICM20948::resetCalibration() {
    // Сброс калибровочных данных
    memset(&calibData, 0, sizeof(IMUCalibData));
    memset(&icmCalibData, 0, sizeof(ICM20948CalibData));
    
    // Установка значений по умолчанию
    for(int i = 0; i < 3; i++) {
        calibData.accelScale[i] = 9.81f / 8192.0f;  // для ±4g
        calibData.gyroScale[i] = (DEG_TO_RAD / 65.5f); // для ±500°/s
        calibData.magScale[i] = 1.0f;
    }
    
    calibrationSaved = false;
    gyroCalibrated = false;
    accelCalibrated = false;
    magCalibrated = false;
    systemCalibrated = false;
    
    // Удаляем сохраненные данные
    Preferences prefs;
    if (prefs.begin(prefsName.c_str(), false)) {
        prefs.remove("icm_calib");
        prefs.putBool("calib_valid", false);
        prefs.end();
    }
    
    logStream->println("ICM-20948 calibration data reset");
}

IMUSensor_ICM20948::~IMUSensor_ICM20948() {
    // Деструктор
} 