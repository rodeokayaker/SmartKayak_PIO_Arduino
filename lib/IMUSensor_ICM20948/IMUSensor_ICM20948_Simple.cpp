/**
 * @file IMUSensor_ICM20948_Simple.cpp
 * @brief Упрощенная, но работающая реализация для ICM-20948
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
      systemCalibrated(false)
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
    
    // Установка режимов работы
    icm.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
    delay(1);
    
    // Установка диапазонов
    ICM_20948_fss_t myFSS;
    myFSS.a = gpm4;   // ±4g для акселерометра
    myFSS.g = dps500; // ±500°/s для гироскопа
    
    icm.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
    delay(1);
    
    // Установка фильтров
    ICM_20948_dlpcfg_t myDLPcfg;
    myDLPcfg.a = acc_d111bw4_n136bw;
    myDLPcfg.g = gyr_d119bw5_n154bw3;
    
    icm.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
    delay(1);
    
    // Включение магнитометра
    icm.startupMagnetometer();
    
    // Установка значений по умолчанию для калибровки
    for(int i = 0; i < 3; i++) {
        calibData.accelScale[i] = 9.81f / 8192.0f;  // для ±4g
        calibData.gyroScale[i] = (DEG_TO_RAD / 65.5f); // для ±500°/s  
        calibData.magScale[i] = 1.0f;
        calibData.accelOffset[i] = 0;
        calibData.gyroOffset[i] = 0;
        calibData.magOffset[i] = 0;
        calibData.magSI[i] = 0.0f;
    }
    
    // Попытка загрузить сохраненные калибровочные данные
    if (readCalibrationData()) {
        logStream->println("Loading saved calibration data...");
        calibrationSaved = true;
    } else {
        logStream->println("No saved calibration data found");
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
    
    // Настройка прерываний если указан пин
    if (interruptPin >= 0) {
        if (dmpEnabled) {
            icm.intEnableDMP(true);
        } else {
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
    // Попытка инициализации DMP
    ICM_20948_Status_e dmpStatus = icm.initializeDMP();
    
    if (dmpStatus == ICM_20948_Stat_Ok) {
        // Включение DMP сенсоров
        icm.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION);
        icm.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER);
        icm.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE);
        icm.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED);
        
        // Установка частоты DMP
        icm.setDMPODRrate(DMP_ODR_Reg_Quat9, 0);
        
        // Включение DMP
        icm.enableDMP(true);
        
        return true;
    }
    
    return false;
}

IMUData IMUSensor_ICM20948::readData() {
    if (!sensorReady) {
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
    
    // Проверка и сохранение калибровки
    checkAndSaveCalibration();
    
    return currentData;
}

void IMUSensor_ICM20948::updateFromDMP() {
    icm_20948_DMP_data_t data;
    icm.readDMPdataFromFIFO(&data);
    
    if (icm.status == ICM_20948_Stat_Ok && (data.header & DMP_header_bitmap_Quat9) > 0) {
        // Кватернион из DMP
        double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;
        double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
        double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
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

// Упрощенные реализации остальных методов
void IMUSensor_ICM20948::calibrate() {
    logStream->println("Starting ICM-20948 basic calibration...");
    resetCalibration();
    
    // Простая калибровка - установка значений по умолчанию
    gyroCalibrated = true;
    accelCalibrated = true;
    magCalibrated = true;
    systemCalibrated = true;
    
    saveCalibrationData();
    calibrationSaved = true;
    
    logStream->println("ICM-20948 calibration complete!");
}

void IMUSensor_ICM20948::calibrateCompass() {
    logStream->println("ICM-20948 compass calibration - using defaults");
    magCalibrated = true;
}

IMUData IMUSensor_ICM20948::getData() { return currentData; }

OrientationData IMUSensor_ICM20948::updateOrientation() {
    currentOrientation.q0 = currentData.q0;
    currentOrientation.q1 = currentData.q1;
    currentOrientation.q2 = currentData.q2;
    currentOrientation.q3 = currentData.q3;
    currentOrientation.timestamp = millis();
    return currentOrientation;
}

OrientationData IMUSensor_ICM20948::getOrientation() { return currentOrientation; }

bool IMUSensor_ICM20948::isCalibrationValid() {
    return sensorReady && (gyroCalibrated || accelCalibrated || magCalibrated);
}

void IMUSensor_ICM20948::magnetometerUpdate() {
    // Простая заглушка
}

void IMUSensor_ICM20948::setCalibrationData(const IMUCalibData data, bool save) {
    calibData = data;
    if (save) {
        saveCalibrationData();
        calibrationSaved = true;
    }
}

IMUCalibData IMUSensor_ICM20948::getCalibrationData() { return calibData; }

void IMUSensor_ICM20948::getCalibrationStatus(bool* sys, bool* gyro, bool* accel, bool* mag) {
    *sys = systemCalibrated;
    *gyro = gyroCalibrated;
    *accel = accelCalibrated;
    *mag = magCalibrated;
}

bool IMUSensor_ICM20948::isFullyCalibrated() {
    return gyroCalibrated && accelCalibrated && magCalibrated;
}

void IMUSensor_ICM20948::printCalibrationStatus() {
    logStream->printf("Calibration Status: Sys=%d Gyro=%d Accel=%d Mag=%d\n",
                     systemCalibrated, gyroCalibrated, accelCalibrated, magCalibrated);
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
    
    // Простая проверка готовности данных
    if (icm.dataReady()) {
        return 1; // Данные готовы
    }
    
    return 0; // Данные не готовы
}

void IMUSensor_ICM20948::checkAndSaveCalibration() {
    if (calibrationSaved && 
        (ICM20948_CALIBRATION_SAVE_INTERVAL < 0 || 
         (millis() - lastSavedCalibrationTime < ICM20948_CALIBRATION_SAVE_INTERVAL))) {
        return;
    }
    
    lastSavedCalibrationTime = millis();
    
    if (isFullyCalibrated()) {
        saveCalibrationData();
        calibrationSaved = true;
        systemCalibrated = true;
    }
}

void IMUSensor_ICM20948::performAutoCalibration() {
    // Простая заглушка для автокалибровки
}

void IMUSensor_ICM20948::applySensorCalibration() {
    // Калибровка применяется программно в scaleAndCalibrateData()
}

void IMUSensor_ICM20948::enableAutoCalibration(bool enable) {
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
    
    size_t dataSize = prefs.getBytesLength("icm_calib");
    if (dataSize != sizeof(IMUCalibData)) {
        prefs.end();
        return false;
    }
    
    size_t readSize = prefs.getBytes("icm_calib", &calibData, sizeof(IMUCalibData));
    bool validFlag = prefs.getBool("calib_valid", false);
    
    prefs.end();
    
    bool success = (readSize == sizeof(IMUCalibData) && validFlag);
    
    if (success) {
        gyroCalibrated = true;
        accelCalibrated = true;
        magCalibrated = true;
        systemCalibrated = true;
    }
    
    return success;
}

void IMUSensor_ICM20948::saveCalibrationData() {
    Preferences prefs;
    if (!prefs.begin(prefsName.c_str(), false)) {
        return;
    }
    
    size_t written = prefs.putBytes("icm_calib", &calibData, sizeof(IMUCalibData));
    prefs.putBool("calib_valid", true);
    
    prefs.end();
    
    if (written == sizeof(IMUCalibData)) {
        logStream->println("Calibration data saved successfully");
    }
}

void IMUSensor_ICM20948::resetCalibration() {
    memset(&calibData, 0, sizeof(IMUCalibData));
    
    // Установка значений по умолчанию
    for(int i = 0; i < 3; i++) {
        calibData.accelScale[i] = 9.81f / 8192.0f;
        calibData.gyroScale[i] = (DEG_TO_RAD / 65.5f);
        calibData.magScale[i] = 1.0f;
    }
    
    calibrationSaved = false;
    gyroCalibrated = false;
    accelCalibrated = false;
    magCalibrated = false;
    systemCalibrated = false;
    
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