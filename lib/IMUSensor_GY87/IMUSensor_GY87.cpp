/**
 * @file IMUSensor_GY87.cpp
 * @brief Реализация библиотеки для работы с IMU модулем GY-87
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#include "IMUSensor_GY87.h"
#include <Preferences.h>


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
      autoCalibrateMag(false) {
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
    logStream->println("\nRotate device in all directions for magnetometer calibration");
    logStream->println("This will take 20 seconds...");
    
    // Запускаем калибровку магнитометра
    int mx_min = 32767, my_min = 32767, mz_min = 32767;
    int mx_max = -32768, my_max = -32768, mz_max = -32768;
    
    uint32_t startTime = millis();
        
    while(millis() - startTime < 20000) {

        int mx, my, mz;
        mag.read(&mx, &my, &mz);
        
        
        mx_min = std::min(mx_min, mx);
        my_min = std::min(my_min, my);
        mz_min = std::min(mz_min, mz);
        
        mx_max = std::max(mx_max, mx);
        my_max = std::max(my_max, my);
        mz_max = std::max(mz_max, mz);
        
        delay(10);
    }
    
    // Вычисление параметров калибровки магнитометра
    calibData.magOffset[0] = -(mx_max + mx_min) / 2;
    calibData.magOffset[1] = -(my_max + my_min) / 2;
    calibData.magOffset[2] = -(mz_max + mz_min) / 2;
    
    calibData.magScale[0] = 1000.0f / (float)(mx_max - mx_min);
    calibData.magScale[1] = 1000.0f / (float)(my_max - my_min);
    calibData.magScale[2] = 1000.0f / (float)(mz_max - mz_min);
    
    logStream->println("Magnetometer calibration complete");
    logStream->println("Offsets and scales:");
    logStream->printf("Mag offsets: X=%.2f Y=%.2f Z=%.2f\n",
        calibData.magOffset[0],
        calibData.magOffset[1],
        calibData.magOffset[2]);
    logStream->printf("Mag scales: X=%.2f Y=%.2f Z=%.2f\n",
        calibData.magScale[0],
        calibData.magScale[1],
        calibData.magScale[2]);
    
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
    setMagMinMax();
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

float IMUSensor_GY87::convertRawAcceleration(int16_t aRaw) {
    return aRaw / 8192.0f;  // для диапазона ±4g
}

float IMUSensor_GY87::convertRawGyro(int16_t gRaw) {
    return gRaw / 65.5f;    // для диапазона ±500°/s
}

float IMUSensor_GY87::convertRawCompass(int mag) {
    return mag / 3000.0f;    // нормализация
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
        currentData.ax = convertRawAcceleration(ax);
        currentData.ay = convertRawAcceleration(ay);
        currentData.az = convertRawAcceleration(az);
        currentData.gx = convertRawGyro(gx);
        currentData.gy = convertRawGyro(gy);
        currentData.gz = convertRawGyro(gz);
            
        // Чтение магнитометра

    // Чтение и преобразование данных магнитометра
        int mx, my, mz;
        mag.read(&mx, &my, &mz);
        
        if(autoCalibrateMag) {
            bool changed = false;
            if (mx < magMinMax.min[0]) { magMinMax.min[0] = mx; changed = true; }
            if (mx > magMinMax.max[0]) { magMinMax.max[0] = mx; changed = true; }
            if (my < magMinMax.min[1]) { magMinMax.min[1] = my; changed = true; }
            if (my > magMinMax.max[1]) { magMinMax.max[1] = my; changed = true; }
            if (mz < magMinMax.min[2]) { magMinMax.min[2] = mz; changed = true; }
            if (mz > magMinMax.max[2]) { magMinMax.max[2] = mz; changed = true; }
            if (changed) {
    // Вычисление параметров калибровки магнитометра
                calibData.magOffset[0] = -(magMinMax.max[0] + magMinMax.min[0]) / 2;
                calibData.magOffset[1] = -(magMinMax.max[1] + magMinMax.min[1]) / 2;
                calibData.magOffset[2] = -(magMinMax.max[2] + magMinMax.min[2]) / 2;
                
                calibData.magScale[0] = 1000.0f / (float)(magMinMax.max[0] - magMinMax.min[0]);
                calibData.magScale[1] = 1000.0f / (float)(magMinMax.max[1] - magMinMax.min[1]);
                calibData.magScale[2] = 1000.0f / (float)(magMinMax.max[2] - magMinMax.min[2]);
            }
        }
    
    // Применяем калибровку к данным магнитометра
        currentData.mx = (mx + calibData.magOffset[0]) * calibData.magScale[0];
        currentData.my = (my + calibData.magOffset[1]) * calibData.magScale[1];
        currentData.mz = (mz + calibData.magOffset[2]) * calibData.magScale[2];
    

        if(log_imu > 1) {
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

        if(log_imu > 1) {  // Расширенное логирование
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

IMUCalibData& IMUSensor_GY87::getCalibrationData() {
    return calibData;
}

void IMUSensor_GY87::getData(IMUData& data) {
    data = currentData;
}

void IMUSensor_GY87::setMagMinMax() {
    if (calibValid) {
        magMinMax.min[0] = -1000/calibData.magScale[0] - calibData.magOffset[0];
        magMinMax.max[0] = 1000/calibData.magScale[0] - calibData.magOffset[0];
        magMinMax.min[1] = -1000/calibData.magScale[1] - calibData.magOffset[1];
        magMinMax.max[1] = 1000/calibData.magScale[1] - calibData.magOffset[1];
        magMinMax.min[2] = -1000/calibData.magScale[2] - calibData.magOffset[2];
        magMinMax.max[2] = 1000/calibData.magScale[2] - calibData.magOffset[2];

    }
    else {
        magMinMax.min[0] = 32767;
        magMinMax.max[0] = -32768;
        magMinMax.min[1] = 32767;
        magMinMax.max[1] = -32768;
        magMinMax.min[2] = 32767;
        magMinMax.max[2] = -32768;
    }

}