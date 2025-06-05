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
#include "OrientationEstimator.h"
#include "Calibration.h"

#define ADAPTIVE_GAIN 0.001f
#define AUTO_CALIB_SAVE_INTERVAL 60000 // one time in 10 minutes if we update data 100HZ


IMUSensor_GY87::IMUSensor_GY87( const char* prefsName, bool use_dmp, uint8_t interPin, Stream* logStream)
    : prefsName(prefsName),
      calibValid(false),
      dmpReady(false),
      dmpValid(false),
      imuFrequency(GY87_IMU_DEFAULT_FREQUENCY),
      logStream(logStream ? logStream : &Serial),
      useDMP(use_dmp),
      interruptPin(interPin),
      EKF_started(false),
      magFrequency(GY87_MAG_DEFAULT_FREQUENCY)
{

      orientationEstimator = new OrientationEstimator(magFrequency);
      calibrator = new Calibration();
      
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
            gyro_scale = 1.0f;             // ошибка
    }

    for(int i = 0; i < 3; i++) {
        calibData.accelOffset[i] = 0;
        calibData.accelScale[i] = accel_scale;  
        
        calibData.gyroOffset[i] = 0;
        calibData.gyroScale[i] = gyro_scale;     
        
        calibData.magOffset[i] = 0;
        calibData.magScale[i] = 1.0f;

        calibData.magSI[i] = 0.0f;
    }
    // Применяем значения по умолчанию к устройствам
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);

    EKF_started = false;
    orientationEstimator->reset();
    calibrator->setCalibrationMatrix(calibData.magOffset, calibData.magScale, calibData.magSI);


}

bool IMUSensor_GY87::begin(uint16_t imuFreq, uint16_t magFreq) {

    magFrequency = magFreq;
    orientationEstimator->setFrequency(magFrequency);
    EKF_started = false;
    orientationEstimator->reset();

#ifdef WIRECLOCK400
    Wire.setClock(400000); // 400kHz I2C clock.
#endif

    // Инициализация MPU6050
    mpu.reset();
    delay(100);
    mpu.initialize();

    // Проверка подключения
    if (!mpu.testConnection()) {

        return false;
    }
    
    // Установка режимов работы
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);    // ±4g
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);   // ±500°/s
        mpu.setRate(4);
        mpu.setDLPFMode(MPU6050_DLPF_BW_98);              // фильтр 98Hz
        mpu.setSleepEnabled(false);
    
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

    //        return false;
        }
    }
    setDefaultCalibration();

    // Загрузка калибровочных данных
    if(readCalibrationData()) {
        mpu.setXAccelOffset(calibData.accelOffset[0]);
        mpu.setYAccelOffset(calibData.accelOffset[1]);
        mpu.setZAccelOffset(calibData.accelOffset[2]);
        mpu.setXGyroOffset(calibData.gyroOffset[0]);
        mpu.setYGyroOffset(calibData.gyroOffset[1]);
        mpu.setZGyroOffset(calibData.gyroOffset[2]);
    }
            



    if(interruptPin >= 0) {
        Serial.printf("MPU interrupt status: %d\n", mpu.getIntStatus());
    }

        // Включаем bypass mode для прямого доступа к магнитометру
     Wire.beginTransmission(0x68);
    Wire.write(0x37);  // Регистр INT_PIN_CFG
    Wire.write(0x02);  // Включаем bypass mode
    Wire.endTransmission();

/*       
    Wire.beginTransmission(0x68);
    Wire.write(0x6A);
    Wire.write(0x00);
    Wire.endTransmission();
    //Disable Sleep Mode
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();  
    */

    // Инициализация магнитометра
    mag.init();
    mag.setMode(Mode_Continuous, ODR_100Hz, RNG_2G, OSR_512);
    


        // Инициализация барометра
    baro.begin(0);
    baro.measurement(3); // Высокая точность измерений



    calibrator->setCalibrationMatrix(calibData.magOffset, calibData.magScale, calibData.magSI);

    
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
        logStream->println("Calibration failed - unstable readings");
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
    EKF_started = false;
    orientationEstimator->reset();
    calibrator->setCalibrationMatrix(calibData.magOffset, calibData.magScale, calibData.magSI);
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
    calibrator->setCalibrationMatrix(calibData.magOffset, calibData.magScale, calibData.magSI);
    orientationEstimator->reset();
    EKF_started = false;
}

void IMUSensor_GY87::saveCalibrationData() {
    Preferences prefs;
    prefs.begin(prefsName.c_str(), false);
    
    // Сохраняем структуру калибровочных данных целиком
    prefs.putBytes("calibData", &calibData, sizeof(IMUCalibData));
    
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
        }
    }
    
    if(!success) {
        setDefaultCalibration();
    }
    prefs.end();

    orientationEstimator->reset();
    EKF_started = false;
    calibrator->setCalibrationMatrix(calibData.magOffset, calibData.magScale, calibData.magSI);

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

    orientationEstimator->reset();
    EKF_started = false;
    calibrator->setCalibrationMatrix(calibData.magOffset, calibData.magScale, calibData.magSI);
    logStream->println("Calibration data reset to defaults");
}

void IMUSensor_GY87::getQuaternion(float* quat) {
    quat[0] = q.w;
    quat[1] = q.x;
    quat[2] = q.y;
    quat[3] = q.z;
}

IMUSensor_GY87::~IMUSensor_GY87() {
    delete orientationEstimator;
    delete calibrator;
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
        } else{
            mpu.getMotion6(&acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2]);
        }
    } else {
        mpu.getMotion6(&acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2]);
    }
            
        // Преобразование в физические величины

    currentData.ax = acc[0]*calibData.accelScale[0];
    currentData.ay = acc[1]*calibData.accelScale[1];
    currentData.az = acc[2]*calibData.accelScale[2];
    currentData.gx = gyro[0]*calibData.gyroScale[0];
    currentData.gy = gyro[1]*calibData.gyroScale[1];
    currentData.gz = gyro[2]*calibData.gyroScale[2];
  
    // Чтение магнитометра

    // Чтение и преобразование данных магнитометра
    int mx, my, mz;
    
    mag.read(&mx, &my, &mz);
    currentData.mag_x = mx;
    currentData.mag_y = my;
    currentData.mag_z = mz;
    
    calculateCalibratedMagnetometer(&mx, &my, &mz, &currentData.mx, &currentData.my, &currentData.mz);
         
    currentData.timestamp = millis();
    
    return currentData;
}

OrientationData IMUSensor_GY87::getOrientation() {
    return currentOrientation;
}

OrientationData IMUSensor_GY87::updateOrientation() 
{
    Eigen::Quaternionf orientation = orientationEstimator->applyCorrection(Eigen::Quaternionf(currentData.q0, currentData.q1, currentData.q2, currentData.q3));
    currentOrientation.q0 = orientation.w();
    currentOrientation.q1 = orientation.x();
    currentOrientation.q2 = orientation.y();
    currentOrientation.q3 = orientation.z();
    currentOrientation.timestamp = millis();

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
    orientationEstimator->reset();
    EKF_started = false;
    calibrator->setCalibrationMatrix(calibData.magOffset, calibData.magScale, calibData.magSI);

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

    if (!dmpValid) {
        return;
    }

    Eigen::Quaternionf dmp = Eigen::Quaternionf(currentData.q0, currentData.q1, currentData.q2, currentData.q3);

    orientationEstimator->update(dmp, Eigen::Vector3f(currentData.mx, currentData.my, currentData.mz));

    if(EKF_started) {
        calibrator->updateSmallEKF(Eigen::Vector3f(currentData.mag_x, currentData.mag_y, currentData.mag_z), dmp);
    } else {
        calibrator->startSmallEKF(Eigen::Vector3f(currentData.mag_x, currentData.mag_y, currentData.mag_z), dmp);
        EKF_started = true;
        mag_call_count = 0;
    }

    if ((mag_call_count++) % 100 == 0) {
        calibrator->updateOffsetFromSmallEKF();
        calibData.magOffset[0] = calibrator->getOffset()[0];
        calibData.magOffset[1] = calibrator->getOffset()[1];
        calibData.magOffset[2] = calibrator->getOffset()[2];
    }

    return;
}

void IMUSensor_GY87::calculateCalibratedMagnetometer(int* mx, int* my, int* mz, float* mx_cal, float* my_cal, float* mz_cal) 
{
    // Вычитаем смещения
    float x = *mx - calibData.magOffset[0];
    float y = *my - calibData.magOffset[1];
    float z = *mz - calibData.magOffset[2];

    // Применяем матрицу калибровки
    // Диагональные элементы из scale
    float scale_x = calibData.magScale[0];
    float scale_y = calibData.magScale[1];
    float scale_z = calibData.magScale[2];

    // Недиагональные элементы из SI (коэффициенты (0,1), (0,2), (1,2))
    float xy = calibData.magSI[0];  // (0,1)
    float xz = calibData.magSI[1];  // (0,2)
    float yz = calibData.magSI[2];  // (1,2)

    // Применяем матричное преобразование
    *mx_cal = scale_x * x + xy * y + xz * z;
    *my_cal = xy * x + scale_y * y + yz * z;
    *mz_cal = xz * x + yz * y + scale_z * z;
}


