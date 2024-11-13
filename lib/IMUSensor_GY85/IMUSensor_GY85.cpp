/**
 * @file IMUSensor_GY85.cpp
 * @brief Реализация библиотеки для работы с IMU модулем GY-85
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#include "IMUSensor_GY85.h"

// Флаг валидной калибровки в EEPROM
constexpr byte VALID_IMU_CALIB_FLAG = 0x42;

// Вспомогательные функции для работы с EEPROM
void EEPROM_writeBytes(int addr, const void* data, size_t length) {
    for(size_t i = 0; i < length; i++) {
        EEPROM.write(addr + i, ((uint8_t*)data)[i]);
    }
    EEPROM.commit();
}

void EEPROM_readBytes(int addr, void* data, size_t length) {
    for(size_t i = 0; i < length; i++) {
        ((uint8_t*)data)[i] = EEPROM.read(addr + i);
    }
}

// Конструктор с инициализацией всех полей
IMUSensor::IMUSensor(ADXL345& a, ITG3200& g, MechaQMC5883& m, 
                     int imuAddr, int imuValidFlagAddr) 
    : accel(a), gyro(g), mag(m), calibValid(false),
      imuCalibAddr(imuAddr), imuValidFlagAddr(imuValidFlagAddr), log_imu(0) {
}

IMUCalibData& IMUSensor::getCalibrationData() {
    return calibData;
}

void IMUSensor::resetCalibration() {
    calibValid = false;
    EEPROM.write(imuValidFlagAddr, 0);
}

void IMUSensor::setDefaultCalibration() {
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

void IMUSensor::saveCalibrationData() {
    EEPROM.write(imuValidFlagAddr, VALID_IMU_CALIB_FLAG);
    EEPROM_writeBytes(imuCalibAddr, &calibData, sizeof(calibData));
    EEPROM.commit();
}

bool IMUSensor::readCalibrationData() {
    if(EEPROM.read(imuValidFlagAddr) == VALID_IMU_CALIB_FLAG) {
        EEPROM_readBytes(imuCalibAddr, &calibData, sizeof(calibData));
        calibValid = true;
        return true;
    } else {
        setDefaultCalibration();
        calibValid = false;
        return false;
    }
}

bool IMUSensor::begin() {
    if(!accel.testConnection() || !gyro.testConnection()) {
        Serial.println("IMU initialization failed!");
        return false;
    }
    
    accel.initialize();
    gyro.initialize();
    mag.init();
    
    // Установка базовых параметров
    accel.setRange(ADXL345_RANGE_2G);
    gyro.setFullScaleRange(ITG3200_FULLSCALE_2000);
    gyro.setDLPFBandwidth(ITG3200_DLPF_BW_42);

    readCalibrationData();        
    return true;
}

void IMUSensor::calibrate() {
    Serial.println("Starting IMU calibration...");
    Serial.println("Keep device still for initial calibration");

    setDefaultCalibration();
    
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    const int samples = 100;
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
    
    calibData.gyroOffset[0] = -(float)gx_sum / (float)samples;
    calibData.gyroOffset[1] = -(float)gy_sum / (float)samples;
    calibData.gyroOffset[2] = -(float)gz_sum / (float)samples;
    
    Serial.println("Rotate device in all directions for magnetometer calibration");
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
    calibData.magOffset[0] = -(float)(mx_max + mx_min) / 2;
    calibData.magOffset[1] = -(float)(my_max + my_min) / 2;
    calibData.magOffset[2] = -(float)(mz_max + mz_min) / 2;
    
    calibData.magScale[0] = 1000.0f / (float)(mx_max - mx_min);
    calibData.magScale[1] = 1000.0f / (float)(my_max - my_min);
    calibData.magScale[2] = 1000.0f / (float)(mz_max - mz_min);
    
    saveCalibrationData();
    calibValid = true;
    Serial.println("Calibration complete!");
}

void IMUSensor::getData(IMUData& data) {
    data.ax = (accel.getAccelerationX() + calibData.accelOffset[0]) * calibData.accelScale[0];
    data.ay = (accel.getAccelerationY() + calibData.accelOffset[1]) * calibData.accelScale[1];
    data.az = (accel.getAccelerationZ() + calibData.accelOffset[2]) * calibData.accelScale[2];
    data.gx = (gyro.getRotationX() + calibData.gyroOffset[0]) * calibData.gyroScale[0];
    data.gy = (gyro.getRotationY() + calibData.gyroOffset[1]) * calibData.gyroScale[1];
    data.gz = (gyro.getRotationZ() + calibData.gyroOffset[2]) * calibData.gyroScale[2];
    int mx, my, mz;
    mag.read(&mx, &my, &mz);
    data.mx = ((float)mx + calibData.magOffset[0]) * calibData.magScale[0];
    data.my = ((float)my + calibData.magOffset[1]) * calibData.magScale[1];
    data.mz = ((float)mz + calibData.magOffset[2]) * calibData.magScale[2];

    if (log_imu) {
        Serial.print("Accel: ");
        Serial.print(data.ax);
        Serial.print(", ");
        Serial.print(data.ay);
        Serial.print(", ");
        Serial.println(data.az);
        Serial.print("Gyro: ");
        Serial.print(data.gx);
        Serial.print(", ");
        Serial.print(data.gy);
        Serial.print(", ");
        Serial.println(data.gz);
        Serial.print("Mag: ");
        Serial.print(data.mx);
        Serial.print(", ");
        Serial.print(data.my);
        Serial.print(", ");
        Serial.println(data.mz);
    }
    
    data.timestamp = millis();
}

IMUData IMUSensor::getData() {
    getData(currentData);
    return currentData;
}

bool IMUSensor::isCalibrationValid() {
    return calibValid;
}
