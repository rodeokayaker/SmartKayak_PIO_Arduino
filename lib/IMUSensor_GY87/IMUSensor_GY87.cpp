/**
 * @file IMUSensor_GY87.cpp
 * @brief Реализация библиотеки для работы с IMU модулем GY-87
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#include "IMUSensor_GY87.h"
#include <Preferences.h>
#include "CalibrationUtils.h"

#define ADAPTIVE_GAIN 0.001f
#define AUTO_CALIB_SAVE_INTERVAL 60000 // one time in 10 minutes if we update data 100HZ


IMUSensor_GY87::IMUSensor_GY87( const char* prefsName, bool use_dmp, uint8_t interPin, Stream* logStream)
    : prefsName(prefsName),
      calibValid(false),
      log_imu(0),
      dmpReady(false),
      dmpValid(false),
      imuFrequency(GY87_IMU_DEFAULT_FREQUENCY),
      logStream(logStream ? logStream : &Serial),
      autoCalibrateMag(false),
      autoCalibCount(0),
      magCalibSampleCount(0),
      magCalibAvgError(0),
      config(),
      fusionFilter(config),
      useDMP(use_dmp),
      interruptPin(interPin) {

    config.magnetic_declination = 0.127f; // ~7.3 градуса для Москвы
    fusionFilter.setConfig(config);
}

void IMUSensor_GY87::setDefaultCalibration() {
    calibValid = false;
    
    // MPU6050 масштабирующие коэффициенты для выбранных режимов
    float accel_scale = 1.0f / 8192.0f;
    float gyro_scale = 1.0f / 65.5f;
    uint8_t accel_range = mpu.getFullScaleAccelRange();
    switch (accel_range) {
        case MPU6050_ACCEL_FS_2:  // ±2g
            accel_scale = 9.81f / 16384.0f;
            break;
        case MPU6050_ACCEL_FS_4:  // ±4g
            accel_scale = 9.81f / 8192.0f;
            break;
        case MPU6050_ACCEL_FS_8:  // ±8g
            accel_scale = 9.81f / 4096.0f;
            break;
        case MPU6050_ACCEL_FS_16: // ±16g
            accel_scale = 9.81f / 2048.0f;
            break;
        default:
            accel_scale = 0;  // ошибка
    }    

    uint8_t gyro_range = mpu.getFullScaleGyroRange();
    switch (gyro_range) {
        case MPU6050_GYRO_FS_250:  // ±250°/s
            gyro_scale = 1.0f / 131.0f;           // 131 LSB/(°/s)
            break;
        case MPU6050_GYRO_FS_500:  // ±500°/s
            gyro_scale = 1.0f / 65.5f;            // 65.5 LSB/(°/s)
            break;
        case MPU6050_GYRO_FS_1000: // ±1000°/s
            gyro_scale = 1.0f / 32.8f;            // 32.8 LSB/(°/s)
            break;
        case MPU6050_GYRO_FS_2000: // ±2000°/s
            gyro_scale = 1.0f / 16.4f;            // 16.4 LSB/(°/s)
            break;
        default:
            gyro_scale = 0.0;             // ошибка
    }

    for(int i = 0; i < 3; i++) {
        calibData.accelOffset[i] = 0;
        calibData.accelScale[i] = accel_scale;  
        
        calibData.gyroOffset[i] = 0;
        calibData.gyroScale[i] = gyro_scale;     
        
        calibData.magOffset[i] = 0;
        calibData.magScale[i] = 1.0f;
    }
    // Применяем значения по умолчанию к устройствам
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);

    if(log_imu > 0) {
        logStream->println("Default calibration values set");
        logStream->printf("Accel scale: 1/%d LSB/g\n", 8192);
        logStream->printf("Gyro scale: 1/%0.1f LSB/°/s\n", 65.5f);
    }
}

bool IMUSensor_GY87::begin(uint16_t imuFreq, uint16_t magFreq) {

    imuFrequency = imuFreq;
    magFrequency = magFreq;

    madgwick.begin(imuFrequency); 

//  Если нет дисплея на Wire, то надо увеличить скорость I2C
//  Раскоментировать эту строку

#ifdef WIRECLOCK400
    Wire.setClock(400000); // 400kHz I2C clock.
#endif

    // Инициализация MPU6050
    mpu.reset();
    delay(100);
    mpu.initialize();
    // Включаем bypass mode для прямого доступа к магнитометру
    
    Wire.beginTransmission(0x68);
    Wire.write(0x6A);
    Wire.write(0x00);
    Wire.endTransmission();
    //Disable Sleep Mode
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();  

    // Инициализация магнитометра
    mag.init();
    mag.setMode(Mode_Continuous, ODR_100Hz, RNG_2G, OSR_512);
    
    if(log_imu > 0) {
        logStream->println("QMC5883L initialized");
        logStream->println("Mode: Continuous");
        logStream->println("ODR: 200Hz");
        logStream->println("Range: 2G");
        logStream->println("OSR: 512");
    }

    // Проверка подключения
    if (!mpu.testConnection()) {
        if(log_imu > 0) {
            logStream->println("MPU6050 connection failed");
        }
        return false;
    }
    
    // Установка режимов работы
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);    // ±4g
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);   // ±500°/s
        mpu.setRate(4);
        mpu.setDLPFMode(MPU6050_DLPF_BW_98);              // фильтр 98Hz
        mpu.setSleepEnabled(false);
    
    if(log_imu > 0) {
        logStream->println("MPU6050 initialized successfully");
        logStream->printf("Accel range: ±4g\n");
        logStream->printf("Gyro range: ±500°/s\n");
        logStream->printf("DLPF bandwidth: 98Hz\n");
    }

    // Инициализация барометра
    baro.begin(0);
    baro.measurement(3); // Высокая точность измерений
    
    // Инициализация DMP
    if(useDMP) {

        devStatus = mpu.dmpInitialize();
        
        if (devStatus == 0) {
            mpu.resetFIFO();         // Clear FIFO buffer
            mpu.setDMPEnabled(true);
            mpu.setIntEnabled(0x00);
            delay(10);
            mpu.setIntEnabled(0x02);  // Enable DMP and FIFO overflow interrupts

            dmpReady = true;
            packetSize = mpu.dmpGetFIFOPacketSize();
        } else {
            Serial.print("DMP Initialization failed. Device status: ");
            Serial.println(devStatus);
            if (!readCalibrationData()) {
                setDefaultCalibration();
            }
    //        return false;
        }
    }

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
            


    // это какой-то странный код для того чтобы было видно QMC5883L вместе с MPU6050
  //Bypass Mode

    if(interruptPin >= 0) {
        Serial.printf("MPU interrupt status: %d\n", mpu.getIntStatus());
    }

    Wire.beginTransmission(0x68);
    Wire.write(0x37);  // Регистр INT_PIN_CFG
    Wire.write(0x02);  // Включаем bypass mode
    Wire.endTransmission();

    // Инициализация компаса
    mag.init();

    
    return true;
}

void IMUSensor_GY87::calibrate() {
    logStream->printf("Starting IMU '%s' calibration...\n", prefsName.c_str());
    
    // Сначала устанавливаем значения по умолчанию
    setDefaultCalibration();
    
    // Этап 1: Калибровка MPU6050 (акселерометр и гироскоп)
    logStream->println("Keep the device still for MPU6050 calibration");
    delay(2000);  // Даем время пользователю подготовиться
    
    logStream->println("Calibrating gyroscope and accelerometer...");

    // Используем встроенную функцию автокалибровки MPU6050
    mpu.CalibrateAccel(6);  // 6 итераций для точности
    mpu.CalibrateGyro(6);   // 6 итераций для точности

    // Проверка стабильности
    int16_t readings1[6], readings2[6];
    getSmoothedReadings(readings1);
    getSmoothedReadings(readings2);
    
    if(!isStable(readings1, readings2, 5)) {
        if(log_imu > 0) {
            logStream->println("Calibration failed - unstable readings");
        }
 //       return false;
    }    
    
    // Сохраняем полученные смещения
    calibData.accelOffset[0] = mpu.getXAccelOffset();
    calibData.accelOffset[1] = mpu.getYAccelOffset();
    calibData.accelOffset[2] = mpu.getZAccelOffset();
    
    calibData.gyroOffset[0] = mpu.getXGyroOffset();
    calibData.gyroOffset[1] = mpu.getYGyroOffset();
    calibData.gyroOffset[2] = mpu.getZGyroOffset();
    
    // Устанавливаем масштабирующие коэффициенты для акселерометра и гироскопа
    // в соответствии с выбранными диапазонами
    for(int i = 0; i < 3; i++) {
        calibData.accelScale[i] = 1.0f / 8192.0f;  // для диапазона ±4g
        calibData.gyroScale[i] = 1.0f / 65.5f;     // для диапазона ±500°/s
    }
    
    logStream->println("MPU6050 calibration complete");
    logStream->println("Offsets:");
    logStream->printf("Accel: X=%d Y=%d Z=%d\n", 
        calibData.accelOffset[0], 
        calibData.accelOffset[1], 
        calibData.accelOffset[2]);
    logStream->printf("Gyro: X=%d Y=%d Z=%d\n", 
        calibData.gyroOffset[0], 
        calibData.gyroOffset[1], 
        calibData.gyroOffset[2]);
    
    // Этап 2: Калибровка магнитометра QMC5883L
    initialCalibrateMagnetometer(false, true);
    
     
    // Применяем калибровку к DMP
    mpu.setXAccelOffset(calibData.accelOffset[0]);
    mpu.setYAccelOffset(calibData.accelOffset[1]);
    mpu.setZAccelOffset(calibData.accelOffset[2]);
    mpu.setXGyroOffset(calibData.gyroOffset[0]);
    mpu.setYGyroOffset(calibData.gyroOffset[1]);
    mpu.setZGyroOffset(calibData.gyroOffset[2]);

        
    // Сохраняем калибровочные данные в энергонезависимую память
    saveCalibrationData();
    calibValid = true;
    
    logStream->println("\nCalibration complete!");
    logStream->println("Calibration data has been saved");
}



void IMUSensor_GY87::initialCalibrateMagnetometer(bool ransac, bool geometric) {
    const int samples = 2000;
    float* x = nullptr;
    float* y = nullptr;
    float* z = nullptr;

    if (geometric||ransac) {
        x = new float[samples];
        y = new float[samples];
        z = new float[samples];
    }
    
    // 1. Сбор данных и калибровка по минимуму и максимуму

    int16_t minX = 32767, maxX = -32767;
    int16_t minY = 32767, maxY = -32767;
    int16_t minZ = 32767, maxZ = -32767;

    logStream->println("\nRotate device in all directions for magnetometer calibration");
    logStream->println("This will take 20 seconds...");

    for(int i = 0; i < samples; i++) {
        int mx, my, mz;
        mag.read(&mx, &my, &mz);
        if (geometric||ransac) {
            x[i] = mx;
            y[i] = my;
            z[i] = mz;
        }
        if(mx < minX) minX = mx;
        if(mx > maxX) maxX = mx;
        if(my < minY) minY = my;
        if(my > maxY) maxY = my;
        if(mz < minZ) minZ = mz;
        if(mz > maxZ) maxZ = mz;
        delay(10);
    }
    
    
    calibData.magOffset[0] = (maxX + minX) / 2;
    calibData.magOffset[1] = (maxY + minY) / 2;
    calibData.magOffset[2] = (maxZ + minZ) / 2;
        
    calibData.magScale[0] = 2.0f / (maxX - minX);
    calibData.magScale[1] = 2.0f / (maxY - minY);
    calibData.magScale[2] = 2.0f / (maxZ - minZ);

    if (!geometric&&!ransac)
    {

        logStream->println("Magnetometer calibration completed");
        logStream->printf("Offset: [%f, %f, %f]\n", 
                  calibData.magOffset[0], 
                  calibData.magOffset[1], 
                  calibData.magOffset[2]);
        logStream->printf("Scale: [%f, %f, %f]\n", 
                  calibData.magScale[0], 
                  calibData.magScale[1], 
                  calibData.magScale[2]);
        return;
    }


    
    bool* inliers = nullptr;
    float centerX=(maxX+minX)/2, centerY=(maxY+minY)/2, centerZ=(maxZ+minZ)/2;
    float radX=(maxX-minX)/2, radY=(maxY-minY)/2, radZ=(maxZ-minZ)/2;

    logStream->printf("Initial center: %f %f %f\n", centerX, centerY, centerZ);
    logStream->printf("Initial radius: %f %f %f\n", radX, radY, radZ);

    // 4. RANSAC фильтрация и оценка параметров
    if (ransac) {
        logStream->printf("RANSAC filtering...\n");
        // Нормализация данных
        for(int i = 0; i < samples; i++) {
            x[i] = (x[i] - centerX) / radX;
            y[i] = (y[i] - centerY) / radY;
            z[i] = (z[i] - centerZ) / radZ;
        }

        inliers = new bool[samples];    
    
        const float ransacThreshold = 0.15f;
        /**max(max(radX, radY), radZ)*/
        const int maxIterations = 500;
    

    
        CalibrationUtils::ransacEllipsoidFilter(x, y, z, inliers, samples, ransacThreshold, maxIterations,
                             &centerX, &centerY, &centerZ, &radX, &radY, &radZ);

        calibData.magOffset[0] = centerX/calibData.magScale[0] + calibData.magOffset[0];
        calibData.magOffset[1] = centerY/calibData.magScale[1] + calibData.magOffset[1];
        calibData.magOffset[2] = centerZ/calibData.magScale[2] + calibData.magOffset[2];
    
        calibData.magScale[0] *= 1.0f/radX;
        calibData.magScale[1] *= 1.0f/radY;
        calibData.magScale[2] *= 1.0f/radZ;

        logStream->printf("RANSAC center: %f %f %f\n", calibData.magOffset[0], calibData.magOffset[1], calibData.magOffset[2]);
        logStream->printf("RANSAC radius: %f %f %f\n", 1/calibData.magScale[0], 1/calibData.magScale[1], 1/calibData.magScale[2]);
    } else{
        centerX=(maxX+minX)/2;
        centerY=(maxY+minY)/2;
        centerZ=(maxZ+minZ)/2;
        radX=(maxX-minX)/2;
        radY=(maxY-minY)/2;
        radZ=(maxZ-minZ)/2;
    }
    
    if (geometric) {
        // Нормализация данных
        for(int i = 0; i < samples; i++) {
            x[i] = (x[i] - centerX) / radX;
            y[i] = (y[i] - centerY) / radY;
            z[i] = (z[i] - centerZ) / radZ;
        }
        centerX = 0;
        centerY = 0;
        centerZ = 0;
        radX = 1;
        radY = 1;
        radZ = 1;

        logStream->printf("Geometric calibration...\n");
        CalibrationUtils::geometricCalibration(x, y, z, samples, centerX, centerY, centerZ, radX, radY, radZ, 1500, 0.002, nullptr/*inliers*/);

        calibData.magOffset[0] = centerX/calibData.magScale[0] + calibData.magOffset[0];
        calibData.magOffset[1] = centerY/calibData.magScale[1] + calibData.magOffset[1];
        calibData.magOffset[2] = centerZ/calibData.magScale[2] + calibData.magOffset[2];
    
        calibData.magScale[0] *= 1.0f/radX;
        calibData.magScale[1] *= 1.0f/radY;
        calibData.magScale[2] *= 1.0f/radZ;        

        logStream->printf("Geometric center: %f %f %f\n", calibData.magOffset[0], calibData.magOffset[1], calibData.magOffset[2]);
        logStream->printf("Geometric radius: %f %f %f\n", 1/calibData.magScale[0], 1/calibData.magScale[1], 1/calibData.magScale[2]);
    }
    
    delete[] x;
    delete[] y;
    delete[] z;
    if (inliers) delete[] inliers;

    
    logStream->println("Magnetometer calibration completed");
    logStream->printf("Offset: [%f, %f, %f]\n", 
                  calibData.magOffset[0], 
                  calibData.magOffset[1], 
                  calibData.magOffset[2]);
    logStream->printf("Scale: [%f, %f, %f]\n", 
                  calibData.magScale[0], 
                  calibData.magScale[1], 
                  calibData.magScale[2]);
}

void IMUSensor_GY87::calibrateCompass() {
    initialCalibrateMagnetometer(false, true);
    saveCalibrationData();
}

void IMUSensor_GY87::adaptiveCalibrateMagnetometer() {
    // Берем сырые данные магнитометра
    float mx_raw = currentData.mag_x;
    float my_raw = currentData.mag_y;
    float mz_raw = currentData.mag_z;
    
    // Применяем текущую калибровку
    float mx = (mx_raw - calibData.magOffset[0]) * calibData.magScale[0];
    float my = (my_raw - calibData.magOffset[1]) * calibData.magScale[1];
    float mz = (mz_raw - calibData.magOffset[2]) * calibData.magScale[2];
    
    // Вычисляем текущую норму вектора
    float mNorm = sqrt(mx*mx + my*my + mz*mz);
    if(mNorm < 1e-6f) return;

    // Ожидаемая норма вектора (идеальная сфера имеет радиус 1)
    const float TARGET_NORM = 1.0f;
    
    // Вычисляем ошибку нормы
    float normError = mNorm - TARGET_NORM;
    
    // Нормализованные компоненты для определения направления коррекции
    float nx = mx / mNorm;
    float ny = my / mNorm;
    float nz = mz / mNorm;

    // Уменьшаем коэффициенты коррекции
    const float BASE_GAIN = 0.05f;  // Уменьшен на порядок
    const float MAX_ERROR = 0.1f;     // Уменьшен до 10%
    float adaptiveGain = BASE_GAIN * (fabs(normError) > MAX_ERROR ? MAX_ERROR : fabs(normError));

    // Коэффициенты для разных типов коррекции
    const float OFFSET_RATIO = 0.7f;
    const float SCALE_RATIO = 0.3f;   // Уменьшен для стабильности

    // Обновление смещения с учетом масштаба
    calibData.magOffset[0] += nx * normError * adaptiveGain * OFFSET_RATIO / calibData.magScale[0];
    calibData.magOffset[1] += ny * normError * adaptiveGain * OFFSET_RATIO / calibData.magScale[1];
    calibData.magOffset[2] += nz * normError * adaptiveGain * OFFSET_RATIO / calibData.magScale[2];

    // Обновление масштаба (более осторожное)
    float scaleCorrection = normError * adaptiveGain * SCALE_RATIO;
    calibData.magScale[0] *= 1.0f - nx * scaleCorrection;
    calibData.magScale[1] *= 1.0f - ny * scaleCorrection;
    calibData.magScale[2] *= 1.0f - nz * scaleCorrection;

/*    // Более строгие ограничения на масштабные коэффициенты
    const float MIN_SCALE = 0.8f;
    const float MAX_SCALE = 1.2f;
    for(int i = 0; i < 3; i++) {
        if(calibData.magScale[i] < MIN_SCALE) calibData.magScale[i] = MIN_SCALE;
        if(calibData.magScale[i] > MAX_SCALE) calibData.magScale[i] = MAX_SCALE;
    }
*/
    // Накапливаем статистику для оценки качества калибровки
    
    magCalibAvgError = (magCalibAvgError * magCalibSampleCount + fabs(normError)) / (magCalibSampleCount + 1);
    if(magCalibSampleCount < MAG_CALIB_STATS_WINDOW) {
        magCalibSampleCount++;
    } else {
        if(log_imu > 1) {
            logStream->printf("Mag norm: %.3f, Error: %.3f\n", mNorm, normError);
            logStream->printf("Scales: %.3f, %.3f, %.3f\n", 
                calibData.magScale[0], calibData.magScale[1], calibData.magScale[2]);
            logStream->printf("Offsets: %.3f, %.3f, %.3f\n", 
                calibData.magOffset[0], calibData.magOffset[1], calibData.magOffset[2]);
        }
        magCalibAvgError = 0;
        magCalibSampleCount = 0;
    }
}

void IMUSensor_GY87::adaptiveCalibrateMagnetometer(float* q) {
    // Извлекаем направление магнитного поля из кватерниона
    float bx = 2.0f * (q[1]*q[3] - q[0]*q[2]);
    float by = 2.0f * (q[0]*q[1] + q[2]*q[3]);
    float bz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    
    // Нормализуем ожидаемое направление
    float bNorm = sqrt(bx*bx + by*by + bz*bz);
    if(bNorm < 1e-6f) return;
    bx /= bNorm;
    by /= bNorm;
    bz /= bNorm;

    // Текущие измерения (до применения калибровки)
    float mx_raw = currentData.mag_x;
    float my_raw = currentData.mag_y;
    float mz_raw = currentData.mag_z;
    
    // Применяем текущую калибровку
    float mx = (mx_raw - calibData.magOffset[0]) * calibData.magScale[0];
    float my = (my_raw - calibData.magOffset[1]) * calibData.magScale[1];
    float mz = (mz_raw - calibData.magOffset[2]) * calibData.magScale[2];
    
    // Нормализуем текущие измерения
    float mNorm = sqrt(mx*mx + my*my + mz*mz);
    if(mNorm < 1e-6f) return;
    
    float mx_norm = mx / mNorm;
    float my_norm = my / mNorm;
    float mz_norm = mz / mNorm;

    // Вычисляем ошибку как векторное произведение нормализованных векторов
    float errX = my_norm*bz - mz_norm*by;
    float errY = mz_norm*bx - mx_norm*bz;
    float errZ = mx_norm*by - my_norm*bx;
    
    // Вычисляем величину ошибки
    float errMagnitude = sqrt(errX*errX + errY*errY + errZ*errZ);
    
    // Уменьшаем коэффициенты коррекции
    const float BASE_GAIN = 0.01f;  // Уменьшен на порядок
    const float MAX_ERROR = 0.1f;     // Уменьшен до 10%
    float adaptiveGain = BASE_GAIN * (errMagnitude > MAX_ERROR ? MAX_ERROR : errMagnitude);

    const float OFFSET_RATIO = 0.7f;
    const float SCALE_RATIO = 0.2f;   // Уменьшен для большей стабильности

    // Обновление смещения (используем исходные данные)
    calibData.magOffset[0] += errX * adaptiveGain * OFFSET_RATIO * mNorm;
    calibData.magOffset[1] += errY * adaptiveGain * OFFSET_RATIO * mNorm;
    calibData.magOffset[2] += errZ * adaptiveGain * OFFSET_RATIO * mNorm;

    // Обновление масштаба (более осторожное)
    float scaleCorrection = adaptiveGain * SCALE_RATIO;
    calibData.magScale[0] *= 1.0f + errX * scaleCorrection;
    calibData.magScale[1] *= 1.0f + errY * scaleCorrection;
    calibData.magScale[2] *= 1.0f + errZ * scaleCorrection;

    // Добавляем статистику для отладки

    if(log_imu > 1 && magCalibSampleCount++ % 100 == 0) {
        logStream->printf("Mag norm: %.3f, Error: %.3f\n", mNorm, errMagnitude);
        logStream->printf("Scales: %.3f, %.3f, %.3f\n", 
            calibData.magScale[0], calibData.magScale[1], calibData.magScale[2]);
        logStream->printf("Offsets: %.3f, %.3f, %.3f\n", 
            calibData.magOffset[0], calibData.magOffset[1], calibData.magOffset[2]);
    }
}


void IMUSensor_GY87::saveCalibrationData() {
    Preferences prefs;
    prefs.begin(prefsName.c_str(), false);
    
    // Сохраняем структуру калибровочных данных целиком
    prefs.putBytes("calibData", &calibData, sizeof(IMUCalibData));
    
    if(log_imu > 0) {
        logStream->println("Calibration data saved to flash memory");
    }
    
    prefs.end();
}

bool IMUSensor_GY87::readCalibrationData() {
    Preferences prefs;
    prefs.begin(prefsName.c_str(), true);
    
    bool success = false;
    
    // Проверяем наличие калибровочных данных
    if(prefs.isKey("calibData")) {
        // Читаем структуру калибровочных данных
        size_t readBytes = prefs.getBytes("calibData", &calibData, sizeof(IMUCalibData));
        
        if(readBytes == sizeof(IMUCalibData)) {
            // Применяем прочитанные значения к MPU6050
            mpu.setXAccelOffset(calibData.accelOffset[0]);
            mpu.setYAccelOffset(calibData.accelOffset[1]);
            mpu.setZAccelOffset(calibData.accelOffset[2]);
            mpu.setXGyroOffset(calibData.gyroOffset[0]);
            mpu.setYGyroOffset(calibData.gyroOffset[1]);
            mpu.setZGyroOffset(calibData.gyroOffset[2]);
            
            calibValid = true;
            success = true;

            // MPU6050 масштабирующие коэффициенты для выбранных режимов
            float accel_scale = 1.0f / 8192.0f;
            float gyro_scale = 1.0f / 65.5f;
            uint8_t accel_range = mpu.getFullScaleAccelRange();
            switch (accel_range) {
                case MPU6050_ACCEL_FS_2:  // ±2g
                    accel_scale = 9.81f / 16384.0f;
                    break;
                case MPU6050_ACCEL_FS_4:  // ±4g
                    accel_scale = 9.81f / 8192.0f;
                    break;
                case MPU6050_ACCEL_FS_8:  // ±8g
                    accel_scale = 9.81f / 4096.0f;
                    break;
                case MPU6050_ACCEL_FS_16: // ±16g
                    accel_scale = 9.81f / 2048.0f;
                    break;
                default:
                    accel_scale = 0;  // ошибка
            }    

            uint8_t gyro_range = mpu.getFullScaleGyroRange();
            switch (gyro_range) {
                case MPU6050_GYRO_FS_250:  // ±250°/s
                    gyro_scale = 1.0f / 131.0f;           // 131 LSB/(°/s)
                    break;
                case MPU6050_GYRO_FS_500:  // ±500°/s
                    gyro_scale = 1.0f / 65.5f;            // 65.5 LSB/(°/s)
                    break;
                case MPU6050_GYRO_FS_1000: // ±1000°/s
                    gyro_scale = 1.0f / 32.8f;            // 32.8 LSB/(°/s)
                    break;
                case MPU6050_GYRO_FS_2000: // ±2000°/s
                    gyro_scale = 1.0f / 16.4f;            // 16.4 LSB/(°/s)
                    break;
                default:
                    gyro_scale = 0.0;             // ошибка
            }

            for(int i = 0; i < 3; i++) {
                calibData.accelScale[i] = accel_scale;   
                calibData.gyroScale[i] = gyro_scale;     
            }

            
            if(log_imu > 0) {
                logStream->println("Calibration data loaded from flash memory");
                logStream->println("Current calibration values:");
                logStream->printf("Accel offsets: X=%d Y=%d Z=%d\n",
                    calibData.accelOffset[0],
                    calibData.accelOffset[1],
                    calibData.accelOffset[2]);
                logStream->printf("Gyro offsets: X=%d Y=%d Z=%d\n",
                    calibData.gyroOffset[0],
                    calibData.gyroOffset[1],
                    calibData.gyroOffset[2]);
                logStream->printf("Mag offsets: X=%.2f Y=%.2f Z=%.2f\n",
                    calibData.magOffset[0],
                    calibData.magOffset[1],
                    calibData.magOffset[2]);
                logStream->printf("Mag scales: X=%.2f Y=%.2f Z=%.2f\n",
                    calibData.magScale[0],
                    calibData.magScale[1],
                    calibData.magScale[2]);
            }
        }
    }
    
    if(!success) {
        if(log_imu > 0) {
            logStream->println("No valid calibration data found, using defaults");
        }
        setDefaultCalibration();
    }
    prefs.end();

    return success;
}

void IMUSensor_GY87::resetCalibration() {
    // Сбрасываем калибровку в памяти
    setDefaultCalibration();
    
    // Удаляем сохраненные данные
    Preferences prefs;
    prefs.begin(prefsName.c_str(), false);
    prefs.remove("calibData");
    prefs.end();
    
    if(log_imu > 0) {
        logStream->println("Calibration data reset to defaults");
    }
}

void IMUSensor_GY87::getQuaternion(float* quat) {
    quat[0] = q.w;
    quat[1] = q.x;
    quat[2] = q.y;
    quat[3] = q.z;
}

IMUSensor_GY87::~IMUSensor_GY87() {
}

IMUData IMUSensor_GY87::readData() {
    dmpValid = false;
    int16_t acc[3], gyro[3];
    if (dmpReady && useDMP) {
        uint16_t fifoCount = mpu.getFIFOCount();
        uint8_t mpuIntStatus = mpu.getIntStatus();
    // Проверяем переполнение FIFO
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        Serial.println("FIFO overflow!");
        mpu.resetFIFO();

    }
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            // Сохранение кватерниона
            currentData.q0 = q.w;
            currentData.q1 = q.x;
            currentData.q2 = q.y;
            currentData.q3 = q.z;  
            mpu.dmpGetAccel(acc, fifoBuffer);
            mpu.dmpGetGyro(gyro, fifoBuffer);
            dmpValid = true;
//            Serial.printf("Got DMP data\n");
        } else{
            mpu.getMotion6(&acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2]);
/*            Serial.printf("DMP is not ready\n");
    if (mpuIntStatus & 0x01) Serial.println("Data Ready INT");
    if (mpuIntStatus & 0x02) Serial.println("DMP INT");
    if (mpuIntStatus & 0x04) Serial.println("PLL Ready INT");
    if (mpuIntStatus & 0x08) Serial.println("I2C Master INT");
    if (mpuIntStatus & 0x10) Serial.println("FIFO Overflow INT");
    if (mpuIntStatus & 0x20) Serial.println("Zero Motion INT");
    if (mpuIntStatus & 0x40) Serial.println("Motion INT");
    if (mpuIntStatus & 0x80) Serial.println("Free Fall INT");        */    
        }
    } else {
        mpu.getMotion6(&acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2]);
//        Serial.printf("DMP is not used\n");
    }
            
        // Преобразование в физические величины

    currentData.ax = acc[0]*calibData.accelScale[0];
    currentData.ay = acc[1]*calibData.accelScale[1];
    currentData.az = acc[2]*calibData.accelScale[2];
    currentData.gx = gyro[0]*calibData.gyroScale[0];
    currentData.gy = gyro[1]*calibData.gyroScale[1];
    currentData.gz = gyro[2]*calibData.gyroScale[2];
/*        Serial.printf("Accel length: %f, gyro length: %f\n", 
            sqrt(currentData.ax*currentData.ax+ currentData.ay*currentData.ay+ currentData.az*currentData.az),
            sqrt(currentData.gx*currentData.gx+ currentData.gy*currentData.gy+ currentData.gz*currentData.gz));
*/            
        // Чтение магнитометра

    // Чтение и преобразование данных магнитометра
    int mx, my, mz;
    mag.read(&mx, &my, &mz);
    currentData.mag_x = mx;
    currentData.mag_y = my;
    currentData.mag_z = mz;
    
    // Применяем калибровку к данным магнитометра
    currentData.mx = (mx - calibData.magOffset[0]) * calibData.magScale[0];
    currentData.my = (my - calibData.magOffset[1]) * calibData.magScale[1];
    currentData.mz = (mz - calibData.magOffset[2]) * calibData.magScale[2];
    

    if(log_imu > 2) {
        logStream->printf("Raw mag: %d %d %d\n", mx, my, mz);
        logStream->printf("Calibrated mag: %.2f %.2f %.2f\n", 
            currentData.mx, currentData.my, currentData.mz);
    }

    if (log_imu == 1) {
        float magNorm = sqrt(currentData.mx*currentData.mx + currentData.my*currentData.my + currentData.mz*currentData.mz);
        logStream->printf("Mag norm: %.2f\n", magNorm);
    }
            
    /*    // Чтение барометра
        if(baro.read(2)) {
            currentData.pressure = baro.pressure;
            currentData.temperature = baro.temperature;
            currentData.altitude = baro.altitude;
        }*/
            
    currentData.timestamp = millis();

    if(log_imu > 2) {  // Расширенное логирование
        logStream->printf("Accel: %.3f %.3f %.3f\n", 
            currentData.ax, currentData.ay, currentData.az);
        logStream->printf("Gyro: %.3f %.3f %.3f\n", 
            currentData.gx, currentData.gy, currentData.gz);
        logStream->printf("Mag: %.3f %.3f %.3f\n", 
            currentData.mx, currentData.my, currentData.mz);
        logStream->printf("Q: %.3f %.3f %.3f %.3f\n", 
            q.w, q.x, q.y, q.z);
    }

    
    return currentData;
}

OrientationData IMUSensor_GY87::getOrientation() {
    return currentOrientation;
}

OrientationData IMUSensor_GY87::updateOrientation() {
    //    old Implementation of Madgwick filter
    madgwick.updateIMU(
        currentData.gx, currentData.gy, currentData.gz,
        currentData.ax, currentData.ay, currentData.az/*,
        currentData.mx, currentData.my, currentData.mz*/
    );
    
    madgwick.getQuaternion(&currentOrientation.q0);

 /*   SP_Math::Quaternion dmp_q(currentData.q0, currentData.q1, currentData.q2, currentData.q3);
    SP_Math::Vector mag(currentData.mx, currentData.my, currentData.mz);
    SP_Math::Vector gyro(currentData.gx, currentData.gy, currentData.gz);
        
    SP_Math::Quaternion final_orientation = fusionFilter.update(dmp_q, mag, gyro);
    
    // Можно получить информацию о качестве данных
    float variance = fusionFilter.getMagVariance();
    float trust = fusionFilter.getMagTrust();
    
    // Обновляем данные
    currentOrientation.q0 = final_orientation[0];
    currentOrientation.q1 = final_orientation[1];
    currentOrientation.q2 = final_orientation[2];
    currentOrientation.q3 = final_orientation[3];*/

    currentOrientation.timestamp = millis();
/*    if (!dmpReady) {
        // Если DMP не готов, возвращаем нулевую ориентацию
        currentOrientation = {0, 0, 0, 0, millis()};
        return currentOrientation;
    }*/

    // Возвращаем последнюю вычисленную ориентацию
    return currentOrientation;
}


void IMUSensor_GY87::setFrequency(uint16_t frequency) {
    imuFrequency = frequency;

    // Настройка частоты DMP
    // DMP работает на фиксированной частоте 200Hz, поэтому мы настраиваем 
    // только частоту DLPF для сглаживания данных

    uint8_t dlpf;
/*    if (frequency <= 5) dlpf = MPU6050_DLPF_BW_5;
    else if (frequency <= 10) dlpf = MPU6050_DLPF_BW_10;
    else if (frequency <= 20) dlpf = MPU6050_DLPF_BW_20;
    else if (frequency <= 42) dlpf = MPU6050_DLPF_BW_42;
    else if (frequency <= 98) dlpf = MPU6050_DLPF_BW_98;
    else dlpf = MPU6050_DLPF_BW_188;

    mpu.setDLPFMode(dlpf);*/

    madgwick.begin(frequency);

    if(log_imu > 0) {
        logStream->printf("IMU frequency set to %dHz\n", frequency);
        logStream->printf("DLPF bandwidth adjusted accordingly\n");
    }
}

IMUData IMUSensor_GY87::getData() {
    return currentData;
}

bool IMUSensor_GY87::isCalibrationValid() {
    return calibValid && dmpReady;
}

bool IMUSensor_GY87::isStable(int16_t* readings1, int16_t* readings2, int tolerance) {
    for(int i = 0; i < 6; i++) {
        if(abs(readings1[i] - readings2[i]) > tolerance) {
            return false;
        }
    }
    return true;
}

void IMUSensor_GY87::getSmoothedReadings(int16_t* readings, int samples) {
    long sums[6] = {0,0,0,0,0,0};
    
    for(int i = 0; i < samples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        sums[0] += ax; sums[1] += ay; sums[2] += az;
        sums[3] += gx; sums[4] += gy; sums[5] += gz;
        
        delayMicroseconds(3150); // держим частоту около 200 Hz
    }
    
    for(int i = 0; i < 6; i++) {
        readings[i] = (sums[i] + samples/2) / samples;
    }
}


void IMUSensor_GY87::setCalibrationData(const IMUCalibData data, bool save) {
    calibData = data;
    mpu.setXAccelOffset(calibData.accelOffset[0]);
    mpu.setYAccelOffset(calibData.accelOffset[1]);
    mpu.setZAccelOffset(calibData.accelOffset[2]);
    mpu.setXGyroOffset(calibData.gyroOffset[0]);
    mpu.setYGyroOffset(calibData.gyroOffset[1]);
    mpu.setZGyroOffset(calibData.gyroOffset[2]);
                    
    calibValid = true;    
    if(save) {
        saveCalibrationData();
    }
}

IMUCalibData IMUSensor_GY87::getCalibrationData() {
    return calibData;
}

void IMUSensor_GY87::PrintCalibrationData(Stream* stream) {
    stream->printf("Accel scale: %f %f %f\n", 
        calibData.accelScale[0], calibData.accelScale[1], calibData.accelScale[2]);
    stream->printf("Accel offset: %d %d %d\n", 
        calibData.accelOffset[0], calibData.accelOffset[1], calibData.accelOffset[2]);
    stream->printf("Gyro scale: %f %f %f\n", 
        calibData.gyroScale[0], calibData.gyroScale[1], calibData.gyroScale[2]);
    stream->printf("Gyro offset: %d %d %d\n", 
        calibData.gyroOffset[0], calibData.gyroOffset[1], calibData.gyroOffset[2]);
    stream->printf("Mag scale: %f %f %f\n", 
        calibData.magScale[0], calibData.magScale[1], calibData.magScale[2]);
    stream->printf("Mag offset: %f %f %f\n", 
        calibData.magOffset[0], calibData.magOffset[1], calibData.magOffset[2]);
}

void IMUSensor_GY87::PrintChipsInfo(Stream* stream) {

    stream->printf("IMU: GY87\n");
//    stream->printf("MPU6050: %s\n", mpu.getDeviceID());
    uint8_t accel_range = mpu.getFullScaleAccelRange();
    switch (accel_range) {
        case MPU6050_ACCEL_FS_2:  // ±2g
            stream->printf("Accel range: +/-2g\n");
            break;
        case MPU6050_ACCEL_FS_4:  // ±4g
            stream->printf("Accel range: +/-4g\n");
            break;
        case MPU6050_ACCEL_FS_8:  // ±8g
            stream->printf("Accel range: +/-8g\n");
            break;
        case MPU6050_ACCEL_FS_16: // ±16g
            stream->printf("Accel range: +/-16g\n");
            break;
        default:
            stream->printf("Accel range: unknown\n");
    }    

    uint8_t gyro_range = mpu.getFullScaleGyroRange();
    switch (gyro_range) {
        case MPU6050_GYRO_FS_250:  // ±250°/s
            stream->printf("Gyro range: +/-250°/s\n");
            break;
        case MPU6050_GYRO_FS_500:  // ±500°/s
            stream->printf("Gyro range: +/-500°/s\n");
            break;
        case MPU6050_GYRO_FS_1000: // ±1000°/s
            stream->printf("Gyro range: +/-1000°/s\n");
            break;
        case MPU6050_GYRO_FS_2000: // ±2000°/s
            stream->printf("Gyro range: +/-2000°/s\n");
            break;
        default:
            stream->printf("Gyro range: unknown\n");
    }

    if (mpu.getDMPEnabled()) {
        stream->printf("DMP enabled, packet size: %d\n", mpu.dmpGetFIFOPacketSize());
    } else {
        stream->printf("DMP disabled\n");
    }


}

void IMUSensor_GY87::magnetometerUpdate() {
    if (autoCalibrateMag) {
        // Адаптивная калибровка магнитометра по данным фильтра Madgwick
//        AHRSQuaternion q = madgwick.getQuaternion();
//        adaptiveCalibrateMagnetometer((float*)&q);
        adaptiveCalibrateMagnetometer();
        autoCalibCount++;

        // Периодически сохраняем калибровку в память
//        if(autoCalibCount % AUTO_CALIB_SAVE_INTERVAL == 0) {
//            saveCalibrationData();
//        }
    }
    return;
}


