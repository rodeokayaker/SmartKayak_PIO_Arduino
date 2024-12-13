/**
 * @file IMUSensor_GY85.cpp
 * @brief Реализация библиотеки для работы с IMU модулем GY-85
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#include "IMUSensor_GY85.h"
#include <Preferences.h>
// Конструктор с инициализацией всех полей
IMUSensor_GY85::IMUSensor_GY85(ADXL345& a, ITG3200& g, MechaQMC5883& m, Stream* logStream) 
    : accel(a), gyro(g), mag(m), calibValid(false), log_imu(0), logStream(logStream) {
    if(!logStream) {
        logStream = &Serial;
    }
}

IMUCalibData& IMUSensor_GY85::getCalibrationData() {
    return calibData;
}

void IMUSensor_GY85::resetCalibration() {
    calibValid = false;
    Preferences prefs;
    prefs.begin("imu", false);
    prefs.clear();
    prefs.end();
}

void IMUSensor_GY85::setDefaultCalibration() {
    calibValid = false;
    calibData.magOffset[0] = 0;
    calibData.magOffset[1] = 0;
    calibData.magOffset[2] = 0;
    calibData.magScale[0] = 1.0f;
    calibData.magScale[1] = 1.0f;
    calibData.magScale[2] = 1.0f;
    calibData.gyroOffset[0] = 0;
    calibData.gyroOffset[1] = 0;
    calibData.gyroOffset[2] = 0;
    calibData.gyroScale[0] = 0.069565f;
    calibData.gyroScale[1] = 0.069565f;
    calibData.gyroScale[2] = 0.069565f;
    calibData.accelOffset[0] = 0;
    calibData.accelOffset[1] = 0;
    calibData.accelOffset[2] = 0;
    calibData.accelScale[0] = 0.039f;
    calibData.accelScale[1] = 0.039f;
    calibData.accelScale[2] = 0.039f;
}

void IMUSensor_GY85::saveCalibrationData() {
    Preferences prefs;
    prefs.begin("imu", false);
    prefs.putBytes("calibData", (byte*)&calibData, sizeof(IMUCalibData));
    prefs.end();
}

bool IMUSensor_GY85::readCalibrationData() {
    Preferences prefs;
    prefs.begin("imu", false);
    if(prefs.isKey("calibData")) {
        prefs.getBytes("calibData", (byte*)&calibData, sizeof(IMUCalibData));
        calibValid = true;
        prefs.end();
        return true;
    } else {
        setDefaultCalibration();
        calibValid = false;
        prefs.end();
        return false;
    }
}

bool IMUSensor_GY85::begin() {
    if(!accel.testConnection() || !gyro.testConnection()) {
        logStream->println("IMU initialization failed!");
        return false;
    }
    
    accel.initialize();
    gyro.initialize();
    mag.init();
    
    // Установка базовых параметров
    accel.setRange(ADXL345_RANGE_2G);
    gyro.setFullScaleRange(ITG3200_FULLSCALE_2000);
    gyro.setDLPFBandwidth(ITG3200_DLPF_BW_98);

    readCalibrationData();        
    return true;
}

void IMUSensor_GY85::calibrate() {
    logStream->println("Starting IMU calibration...");
    logStream->println("Keep device still for initial calibration");

    setDefaultCalibration();
    
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    const int samples = 1000;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    
    for(int i = 0; i < samples; i++) {
        int16_t ax, ay, az;
        accel.getAcceleration(&ax, &ay, &az);
        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;

        int16_t gx, gy, gz;
        gyro.getRotation(&gx, &gy, &gz);
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;

        delay(10);
    }
    
    // Вычисление и установка смещений акселерометра
    int8_t accel_offset[3];
    accel_offset[0] = -(ax_sum / samples) / 4;
    accel_offset[1] = -(ay_sum / samples) / 4;
    accel_offset[2] = -(az_sum / samples) / 4;
    
    calibData.gyroOffset[0] = -gx_sum / samples;
    calibData.gyroOffset[1] = -gy_sum / samples;
    calibData.gyroOffset[2] = -(gz_sum / samples);
    
    logStream->println("Rotate device in all directions for magnetometer calibration");
    int16_t mx_min = 32767, my_min = 32767, mz_min = 32767;
    int16_t mx_max = -32768, my_max = -32768, mz_max = -32768;
    
    uint32_t startTime = millis();
    while(millis() - startTime < 20000) {
        int mx, my, mz;
        mag.read(&mx, &my, &mz);
        
        mx_min = std::min(mx_min, (int16_t)mx);
        my_min = std::min(my_min, (int16_t)my);
        mz_min = std::min(mz_min, (int16_t)mz);
        
        mx_max = std::max(mx_max, (int16_t)mx);
        my_max = std::max(my_max, (int16_t)my);
        mz_max = std::max(mz_max, (int16_t)mz);
        
        delay(10);
    }
    
    // Вычисление параметров калибровки магнитометра
    calibData.magOffset[0] = -(mx_max + mx_min) / 2;
    calibData.magOffset[1] = -(my_max + my_min) / 2;
    calibData.magOffset[2] = -(mz_max + mz_min) / 2;
    
    calibData.magScale[0] = 1000.0f / (float)(mx_max - mx_min);
    calibData.magScale[1] = 1000.0f / (float)(my_max - my_min);
    calibData.magScale[2] = 1000.0f / (float)(mz_max - mz_min);
    
    saveCalibrationData();
    calibValid = true;
    logStream->println("Calibration complete!");
}

void IMUSensor_GY85::getData(IMUData& data) {
    #if 0
    data.ax = (accel.getAccelerationX() + calibData.accelOffset[0]) * calibData.accelScale[0];
    data.ay = (accel.getAccelerationY() + calibData.accelOffset[1]) * calibData.accelScale[1];
    data.az = (accel.getAccelerationZ() + calibData.accelOffset[2]) * calibData.accelScale[2];
    data.gx = (gyro.getRotationX() + calibData.gyroOffset[0]) * calibData.gyroScale[0];
    data.gy = (gyro.getRotationY() + calibData.gyroOffset[1]) * calibData.gyroScale[1];
    data.gz = (gyro.getRotationZ() + calibData.gyroOffset[2]) * calibData.gyroScale[2];
    #else
    data.ax = accel.getAccelerationX();
    data.ay = accel.getAccelerationY();
    data.az = accel.getAccelerationZ();

    data.gx = (gyro.getRotationX() + calibData.gyroOffset[0]) * calibData.gyroScale[0];
    data.gy = (gyro.getRotationY() + calibData.gyroOffset[1]) * calibData.gyroScale[1];
    data.gz = (gyro.getRotationZ() + calibData.gyroOffset[2]) * calibData.gyroScale[2];

    #endif

    int mx, my, mz;
    mag.read(&mx, &my, &mz);
    data.mx = ((float)mx + calibData.magOffset[0]) * calibData.magScale[0];
    data.my = ((float)my + calibData.magOffset[1]) * calibData.magScale[1];
    data.mz = ((float)mz + calibData.magOffset[2]) * calibData.magScale[2];

    if (log_imu) {
        logStream->print("Accel: ");
        logStream->print(data.ax);
        logStream->print(", ");
        logStream->print(data.ay);
        logStream->print(", ");
        logStream->println(data.az);
        logStream->print("Gyro: ");
        logStream->print(data.gx);
        logStream->print(", ");
        logStream->print(data.gy);
        logStream->print(", ");
        logStream->println(data.gz);
        logStream->print("Mag: ");
        logStream->print(data.mx);
        logStream->print(", ");
        logStream->print(data.my);
        logStream->print(", ");
        logStream->println(data.mz);
    }
    
    data.timestamp = millis();
}

IMUData IMUSensor_GY85::getData() {
    getData(currentData);
    return currentData;
}

bool IMUSensor_GY85::isCalibrationValid() {
    return calibValid;
}
