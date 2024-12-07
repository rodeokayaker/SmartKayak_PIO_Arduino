/**
 * @file IMUSensor_GY87.cpp
 * @brief Реализация библиотеки для работы с IMU модулем GY-87
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#include "IMUSensor_GY87.h"

constexpr byte VALID_IMU_CALIB_FLAG = 0x42;

IMUSensor::IMUSensor(int imuAddr, int imuValidFlagAddr)
    : imuCalibAddr(imuAddr),
      imuValidFlagAddr(imuValidFlagAddr),
      calibValid(false),
      log_imu(0),
      dmpReady(false) {
}

void IMUSensor::setDefaultCalibration() {
    // MPU6050 defaults
    calibData.accelOffset[0] = -1003;
    calibData.accelOffset[1] = -465;
    calibData.accelOffset[2] = 1353;
    calibData.gyroOffset[0] = 90;
    calibData.gyroOffset[1] = 33;
    calibData.gyroOffset[2] = 37;
    
    // QMC5883L defaults
    calibData.magOffset[0] = 50.00;
    calibData.magOffset[1] = 1423.00;
    calibData.magOffset[2] = -265.00;
    calibData.magScale[0] = 0.87;
    calibData.magScale[1] = 1.06;
    calibData.magScale[2] = 1.11;
}

bool IMUSensor::begin() {
    // Инициализация барометра
    baro.begin(0);
    baro.measurement(3); // Высокая точность измерений
    
    // Инициализация MPU6050
    mpu.initialize();
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    
    // Инициализация DMP
    devStatus = mpu.dmpInitialize();
    
    if (devStatus == 0) {
        // Загрузка калибровочных данных
        if(readCalibrationData()) {
            mpu.setXAccelOffset(calibData.accelOffset[0]);
            mpu.setYAccelOffset(calibData.accelOffset[1]);
            mpu.setZAccelOffset(calibData.accelOffset[2]);
            mpu.setXGyroOffset(calibData.gyroOffset[0]);
            mpu.setYGyroOffset(calibData.gyroOffset[1]);
            mpu.setZGyroOffset(calibData.gyroOffset[2]);
        } else {
            setDefaultCalibration();
        }
        
        mpu.setDMPEnabled(true);
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.println("DMP Initialization failed");
        return false;
    }
    
    // Инициализация компаса
    compass.init();
    compass.setCalibrationOffsets(calibData.magOffset[0], 
                                 calibData.magOffset[1], 
                                 calibData.magOffset[2]);
    compass.setCalibrationScales(calibData.magScale[0], 
                                calibData.magScale[1], 
                                calibData.magScale[2]);
    
    return true;
}

void IMUSensor::getData(IMUData& data) {
    if (!dmpReady) return;
    
    // Чтение DMP
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        // Чтение сырых данных
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        // Преобразование в физические величины
        data.ax = convertRawAcceleration(ax);
        data.ay = convertRawAcceleration(ay);
        data.az = convertRawAcceleration(az);
        data.gx = convertRawGyro(gx);
        data.gy = convertRawGyro(gy);
        data.gz = convertRawGyro(gz);
        
        // Чтение магнитометра
        compass.read();
        data.mx = convertRawCompass(compass.getX());
        data.my = convertRawCompass(compass.getY());
        data.mz = convertRawCompass(compass.getZ());
        
        // Чтение барометра
        if(baro.read(2)) {
            pressure = baro.pressure;
            temperature = baro.temperature;
            altitude = baro.altitude;
        }
        
        // Сохранение кватерниона
        data.q0 = q.w;
        data.q1 = q.x;
        data.q2 = q.y;
        data.q3 = q.z;
        
        if(log_imu > 0) {
            Serial.printf("YPR: %.2f %.2f %.2f\n", 
                        ypr[0] * 180/M_PI, 
                        ypr[1] * 180/M_PI, 
                        ypr[2] * 180/M_PI);
        }
    }
    
    data.timestamp = millis();
}

void IMUSensor::calibrate() {
    Serial.println("Starting MPU6050 calibration...");
    Serial.println("Keep device still and level!");
    delay(3000);

    // Калибровка MPU6050 с использованием PID
    mpu.CalibrateAccel(6);  // Точная калибровка акселерометра (6 итераций)
    mpu.CalibrateGyro(6);   // Точная калибровка гироскопа (6 итераций)
    
    // Сохранение калибровочных данных MPU6050
    calibData.accelOffset[0] = mpu.getXAccelOffset();
    calibData.accelOffset[1] = mpu.getYAccelOffset();
    calibData.accelOffset[2] = mpu.getZAccelOffset();
    calibData.gyroOffset[0] = mpu.getXGyroOffset();
    calibData.gyroOffset[1] = mpu.getYGyroOffset();
    calibData.gyroOffset[2] = mpu.getZGyroOffset();

    if(log_imu > 0) {
        Serial.println("\nMPU6050 calibration results:");
        Serial.println("Accel offsets:");
        Serial.printf("X: %d, Y: %d, Z: %d\n", 
            calibData.accelOffset[0],
            calibData.accelOffset[1],
            calibData.accelOffset[2]);
        Serial.println("Gyro offsets:");
        Serial.printf("X: %d, Y: %d, Z: %d\n",
            calibData.gyroOffset[0],
            calibData.gyroOffset[1],
            calibData.gyroOffset[2]);
    }

    // Калибровка магнитометра
    Serial.println("\nStarting magnetometer calibration...");
    Serial.println("Rotate device in all directions for 20 seconds");
    
    int16_t mx_min = 32767, my_min = 32767, mz_min = 32767;
    int16_t mx_max = -32768, my_max = -32768, mz_max = -32768;
    
    uint32_t startTime = millis();
    while(millis() - startTime < 20000) {
        int x, y, z;
        compass.read(&x, &y, &z);
        
        mx_min = min(mx_min, (int16_t)x);
        my_min = min(my_min, (int16_t)y);
        mz_min = min(mz_min, (int16_t)z);
        
        mx_max = max(mx_max, (int16_t)x);
        my_max = max(my_max, (int16_t)y);
        mz_max = max(mz_max, (int16_t)z);
        
        if(log_imu > 1) {
            Serial.printf("Mag raw - X: %d, Y: %d, Z: %d\n", x, y, z);
        }
        delay(100);
    }
    
    // Вычисление параметров калибровки магнитометра
    calibData.magOffset[0] = (mx_max + mx_min) / 2.0f;
    calibData.magOffset[1] = (my_max + my_min) / 2.0f;
    calibData.magOffset[2] = (mz_max + mz_min) / 2.0f;
    
    calibData.magScale[0] = 1.0f / (mx_max - mx_min);
    calibData.magScale[1] = 1.0f / (my_max - my_min);
    calibData.magScale[2] = 1.0f / (mz_max - mz_min);

    if(log_imu > 0) {
        Serial.println("\nMagnetometer calibration results:");
        Serial.println("Offsets:");
        Serial.printf("X: %.2f, Y: %.2f, Z: %.2f\n",
            calibData.magOffset[0],
            calibData.magOffset[1],
            calibData.magOffset[2]);
        Serial.println("Scales:");
        Serial.printf("X: %.3f, Y: %.3f, Z: %.3f\n",
            calibData.magScale[0],
            calibData.magScale[1],
            calibData.magScale[2]);
    }

    // Сохранение всех калибровочных данных в EEPROM
    saveCalibrationData();
    calibValid = true;
    
    Serial.println("Calibration complete!");
}

void IMUSensor::saveCalibrationData() {
    EEPROM.begin(512);
    EEPROM.write(imuValidFlagAddr, VALID_IMU_CALIB_FLAG);
    EEPROM.put(imuCalibAddr, calibData);
    EEPROM.commit();
    EEPROM.end();
}

bool IMUSensor::readCalibrationData() {
    EEPROM.begin(512);
    if(EEPROM.read(imuValidFlagAddr) == VALID_IMU_CALIB_FLAG) {
        EEPROM.get(imuCalibAddr, calibData);
        EEPROM.end();
        
        // Применение загруженных калибровочных данных
        mpu.setXAccelOffset(calibData.accelOffset[0]);
        mpu.setYAccelOffset(calibData.accelOffset[1]);
        mpu.setZAccelOffset(calibData.accelOffset[2]);
        mpu.setXGyroOffset(calibData.gyroOffset[0]);
        mpu.setYGyroOffset(calibData.gyroOffset[1]);
        mpu.setZGyroOffset(calibData.gyroOffset[2]);
        
        compass.setCalibrationOffsets(
            calibData.magOffset[0],
            calibData.magOffset[1],
            calibData.magOffset[2]
        );
        compass.setCalibrationScales(
            calibData.magScale[0],
            calibData.magScale[1],
            calibData.magScale[2]
        );
        
        return true;
    }
    EEPROM.end();
    return false;
}

float IMUSensor::convertRawAcceleration(int16_t aRaw) {
    return aRaw / 16384.0f;  // для диапазона ±2g
}

float IMUSensor::convertRawGyro(int16_t gRaw) {
    return gRaw / 16.4f;     // для диапазона ±2000°/s
}

float IMUSensor::convertRawCompass(int mag) {
    return mag / 3000.0f;    // нормализация
}

void IMUSensor::getYawPitchRoll(float* ypr_out) {
    ypr_out[0] = ypr[0];
    ypr_out[1] = ypr[1];
    ypr_out[2] = ypr[2];
}

void IMUSensor::getQuaternion(float* quat) {
    quat[0] = q.w;
    quat[1] = q.x;
    quat[2] = q.y;
    quat[3] = q.z;
}

// ... (остальные методы аналогичны IMUSensor_GY85)
