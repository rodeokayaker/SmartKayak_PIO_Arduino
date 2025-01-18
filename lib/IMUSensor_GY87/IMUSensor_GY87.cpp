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


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
uint8_t mpuIntStatus = 0;
void dmpDataReady() {
    mpuInterrupt = true;
}


IMUSensor_GY87::IMUSensor_GY87(const char* prefsName, Stream* logStream)
    : prefsName(prefsName),
      calibValid(false),
      log_imu(0),
      dmpReady(false),
      imuFrequency(GY87_IMU_DEFAULT_FREQUENCY),
      logStream(logStream ? logStream : &Serial),
      interruptPin(-1),
      autoCalibrateMag(false),
      autoCalibCount(0) {
}

void IMUSensor_GY87::setDefaultCalibration() {
    calibValid = false;
    
    // MPU6050 масштабирующие коэффициенты для выбранных режимов
    for(int i = 0; i < 3; i++) {
        calibData.accelOffset[i] = 0;
        calibData.accelScale[i] = 1.0f / 8192.0f;   // для диапазона ±4g
        
        calibData.gyroOffset[i] = 0;
        calibData.gyroScale[i] = 1.0f / 65.5f;      // для диапазона ±500°/s
        
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

bool IMUSensor_GY87::begin() {

    madgwick.begin(imuFrequency); 
    Wire.setClock(400000); // 400kHz I2C clock.
    // Инициализация MPU6050
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
    mag.setMode(Mode_Continuous, ODR_200Hz, RNG_2G, OSR_512);
    
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
    mpu.setDLPFMode(MPU6050_DLPF_BW_98);              // фильтр 98Hz
    
    if(log_imu > 0) {
        logStream->println("MPU6050 initialized successfully");
        logStream->printf("Accel range: ±4g\n");
        logStream->printf("Gyro range: ±500°/s\n");
        logStream->printf("DLPF bandwidth: 42Hz\n");
    }

    // Инициализация барометра
    baro.begin(0);
    baro.measurement(3); // Высокая точность измерений
    
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
        Serial.print("DMP Initialization failed. Device status: ");
        Serial.println(devStatus);
//        return false;
    }
    // это какой-то странный код для того чтобы было видно QMC5883L вместе с MPU6050
  //Bypass Mode

    if(interruptPin >= 0) {
        attachInterrupt(digitalPinToInterrupt(interruptPin), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        mpuInterrupt = false;
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
    initialCalibrateMagnetometer(true, false);
    
     
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
    
        const float ransacThreshold = 0.2f*max(max(radX, radY), radZ);
        const int maxIterations = 100;
    

    
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
        CalibrationUtils::geometricCalibration(x, y, z, samples, centerX, centerY, centerZ, radX, radY, radZ, 100, 0.001, inliers);

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

void IMUSensor_GY87::adaptiveCalibrateMagnetometer(float* q) {
    // Извлечение оценки направления магнитного поля из кватерниона ориентации
    float bx = 2.0f * (q[1]*q[3] - q[0]*q[2]);
    float by = 2.0f * (q[0]*q[1] + q[2]*q[3]);
    float bz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    
    float bNorm = sqrt(bx*bx + by*by + bz*bz);
    bx /= bNorm;
    by /= bNorm;
    bz /= bNorm;

    float magNorm = sqrt(currentData.mx*currentData.mx + currentData.my*currentData.my + currentData.mz*currentData.mz);
    
    // Вычисление ошибки между оценкой и текущими измерениями магнитометра
    float magError;
    magError = bx - currentData.mx;
    calibData.magOffset[0] -= magError * ADAPTIVE_GAIN/calibData.magScale[0];
    if (currentData.mx > 0.001 || currentData.mx < -0.001) {
        calibData.magScale[0] *= 1.0f + magError*ADAPTIVE_GAIN/currentData.mx;
    }
    
    magError = by - currentData.my;
    calibData.magOffset[1] -= magError * ADAPTIVE_GAIN/calibData.magScale[1];
    if (currentData.my > 0.001 || currentData.my < -0.001) {
        calibData.magScale[1] *= 1.0f + magError*ADAPTIVE_GAIN/currentData.my;
    }
    
    magError = bz - currentData.mz;
    calibData.magOffset[2] -= magError * ADAPTIVE_GAIN/calibData.magScale[2];
    if (currentData.mz > 0.001 || currentData.mz < -0.001) {
        calibData.magScale[2] *= 1.0f + magError*ADAPTIVE_GAIN/currentData.mz;
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

void IMUSensor_GY87::update() {

    if ((interruptPin >= 0 && mpuInterrupt) || (interruptPin < 0)) {
        mpuInterrupt = false;
        if (dmpReady) {
            if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                // Сохранение кватерниона
                currentData.q0 = q.w;
                currentData.q1 = q.x;
                currentData.q2 = q.y;
                currentData.q3 = q.z;            
            } else{
                currentData.q0 = 0;
                currentData.q1 = 0;
                currentData.q2 = 0;
                currentData.q3 = 0;
            }
        }

        // Чтение сырых данных
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            
        // Преобразование в физические величины
        currentData.ax = ax*calibData.accelScale[0];
        currentData.ay = ay*calibData.accelScale[1];
        currentData.az = az*calibData.accelScale[2];
        currentData.gx = gx*calibData.gyroScale[0];
        currentData.gy = gy*calibData.gyroScale[1];
        currentData.gz = gz*calibData.gyroScale[2];
            
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
    }
    madgwick.update(
        currentData.gx, currentData.gy, currentData.gz,
        currentData.ax, currentData.ay, currentData.az,
        currentData.mx, currentData.my, currentData.mz
    );

    if (autoCalibrateMag) {
        // Адаптивная калибровка магнитометра по данным фильтра Madgwick
        AHRSQuaternion q = madgwick.getQuaternion();
        adaptiveCalibrateMagnetometer((float*)&q);
  
        autoCalibCount++;

        // Периодически сохраняем калибровку в память
        if(autoCalibCount % AUTO_CALIB_SAVE_INTERVAL == 0) {
            saveCalibrationData();
        }
    }
    

}

OrientationData IMUSensor_GY87::getOrientation() {
    madgwick.getQuaternion(&currentOrientation.q0);
    currentOrientation.timestamp = millis();
/*    if (!dmpReady) {
        // Если DMP не готов, возвращаем нулевую ориентацию
        currentOrientation = {0, 0, 0, 0, millis()};
        return currentOrientation;
    }*/

    // Возвращаем последнюю вычисленную ориентацию
    return currentOrientation;
}

int16_t IMUSensor_GY87::getFrequency() {
    return imuFrequency;
}

void IMUSensor_GY87::setFrequency(int16_t frequency) {
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

void IMUSensor_GY87::getData(IMUData& data) {
    data = currentData;
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
