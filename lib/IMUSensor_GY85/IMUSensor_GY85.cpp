/**
 * @file IMUSensor_GY85.cpp
 * @brief Реализация библиотеки для работы с IMU модулем GY-85
 */

#include "IMUSensor_GY85.h"
#include <Preferences.h>
#include "CalibrationUtils.h"

#define ADAPTIVE_GAIN 0.01f
#define AUTO_CALIB_SAVE_INTERVAL 60000 // one time in 10 minutes if we update data 100HZ

//-----------------------------------------------------------------------------
// Конструктор и инициализация
//-----------------------------------------------------------------------------

IMUSensor_GY85::IMUSensor_GY85(const char* prefs_Name, Stream* logStream) : 
    accel(), 
    gyro(), 
    mag(), 
    calibValid(false), 
    log_imu(0), 
    logStream(logStream ? logStream : &Serial), 
    prefsName(prefs_Name),
    imuFrequency(GY85_IMU_DEFAULT_FREQUENCY),
    autoCalibCount(0) {
    
    madgwick.begin(imuFrequency);
}

bool IMUSensor_GY85::begin() {
    // Проверка подключения датчиков
    if(!accel.testConnection() || !gyro.testConnection()) {
        logStream->println("IMU initialization failed!");
        return false;
    }
    
    // Инициализация датчиков
    accel.initialize();
    gyro.initialize();
    mag.init();
    madgwick.begin(imuFrequency);
    
    // Установка базовых параметров
    accel.setRange(ADXL345_RANGE_2G);
    gyro.setFullScaleRange(ITG3200_FULLSCALE_2000);
    gyro.setDLPFBandwidth(ITG3200_DLPF_BW_98);

    // Загрузка калибровки
    readCalibrationData();        
    return true;
}

//-----------------------------------------------------------------------------
// Методы калибровки
//-----------------------------------------------------------------------------

void IMUSensor_GY85::setDefaultCalibration() {
    calibValid = false;
    
    // Акселерометр
    for(int i = 0; i < 3; i++) {
        calibData.accelOffset[i] = 0;
        calibData.accelScale[i] = 0.039f;  // Для диапазона ±2g
    }
    
    // Гироскоп
    for(int i = 0; i < 3; i++) {
        calibData.gyroOffset[i] = 0;
        calibData.gyroScale[i] = 0.069565f;  // Для диапазона ±2000°/s
    }
    
    // Магнитометр
    for(int i = 0; i < 3; i++) {
        calibData.magOffset[i] = 0;
        calibData.magScale[i] = 1.0f;
    }
}

void IMUSensor_GY85::calibrate() {
    logStream->printf("Starting IMU '%s' calibration...\n", prefsName.c_str());
    logStream->println("Keep device still for initial calibration");
    delay(1000);

    setDefaultCalibration();
    // Калибровка акселерометра и гироскопа
    logStream->println("Calibrating accelerometer and gyroscope...");
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    const int samples = 500;
    
    for(int i = 0; i < samples; i++) {
        int16_t ax, ay, az;
        accel.getAcceleration(&ax, &ay, &az);
        ax_sum += ax; ay_sum += ay; az_sum += az;

        int16_t gx, gy, gz;
        gyro.getRotation(&gx, &gy, &gz);
        gx_sum += gx; gy_sum += gy; gz_sum += gz;
        delay(10);
    }

    logStream->println("Calibrate gyroscope done");
    
    // Вычисление и установка смещений акселерометра
    int8_t accel_offset[3];
    accel_offset[0] = -(ax_sum / samples) / 4;
    accel_offset[1] = -(ay_sum / samples) / 4;
    accel_offset[2] = -(az_sum / samples) / 4;
    
    calibData.gyroOffset[0] = -gx_sum / samples;
    calibData.gyroOffset[1] = -gy_sum / samples;
    calibData.gyroOffset[2] = -gz_sum / samples;
    
    initialCalibrateMagnetometer(true, false);
    
    saveCalibrationData();
    calibValid = true;
    logStream->println("Calibration complete!");
}

void IMUSensor_GY85::initialCalibrateMagnetometer(bool ransac, bool geometric) {
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

void IMUSensor_GY87::calibrateCompass() {
    initialCalibrateMagnetometer(true, false);
    saveCalibrationData();
}



//-----------------------------------------------------------------------------
// Работа с данными датчиков
//-----------------------------------------------------------------------------

void IMUSensor_GY85::getData(IMUData& data) {
    #if 0
    data.ax = (accel.getAccelerationX() + calibData.accelOffset[0]) * calibData.accelScale[0];
    data.ay = (accel.getAccelerationY() + calibData.accelOffset[1]) * calibData.accelScale[1];
    data.az = (accel.getAccelerationZ() + calibData.accelOffset[2]) * calibData.accelScale[2];
    data.gx = (gyro.getRotationX() + calibData.gyroOffset[0]) * calibData.gyroScale[0];
    data.gy = (gyro.getRotationY() + calibData.gyroOffset[1]) * calibData.gyroScale[1];
    data.gz = (gyro.getRotationZ() + calibData.gyroOffset[2]) * calibData.gyroScale[2];
    #else
    // Чтение акселерометра
    data.ax = accel.getAccelerationX();
    data.ay = accel.getAccelerationY();
    data.az = accel.getAccelerationZ();

    // Чтение и калибровка гироскопа
    data.gx = (gyro.getRotationX() + calibData.gyroOffset[0]) * calibData.gyroScale[0];
    data.gy = (gyro.getRotationY() + calibData.gyroOffset[1]) * calibData.gyroScale[1];
    data.gz = (gyro.getRotationZ() + calibData.gyroOffset[2]) * calibData.gyroScale[2];

    #endif
    // Чтение и калибровка магнитометра
    int mx, my, mz;
    mag.read(&mx, &my, &mz);
    data.mx = (((float)mx - calibData.magOffset[0]) * calibData.magScale[0]);
    data.my = (((float)my - calibData.magOffset[1]) * calibData.magScale[1]);
    data.mz = (((float)mz - calibData.magOffset[2]) * calibData.magScale[2]);
    

    
    // Получение кватерниона ориентации
    madgwick.getQuaternion(&data.q0);
    
    // Логирование если включено
    if (log_imu) {
        logStream->printf("Accel: %.2f, %.2f, %.2f\n", data.ax, data.ay, data.az);
        logStream->printf("Gyro: %.2f, %.2f, %.2f\n", data.gx, data.gy, data.gz);
        logStream->printf("Mag: %.2f, %.2f, %.2f\n", data.mx, data.my, data.mz);
    }
    
    data.timestamp = millis();
}

void IMUSensor_GY85::update() {
    getData(currentData);
    
    // Обновление фильтра Madgwick
    madgwick.update(
        currentData.gx, currentData.gy, currentData.gz,
        currentData.ax, currentData.ay, currentData.az,
        currentData.mx, currentData.my, currentData.mz
    );
    
    // Адаптивная калибровка магнитометра по данным фильтра Madgwick
    AHRSQuaternion q = madgwick.getQuaternion();
    adaptiveCalibrateMagnetometer((float*)&q);
  
    autoCalibCount++;

        // Периодически сохраняем калибровку в память
    if(autoCalibCount % AUTO_CALIB_SAVE_INTERVAL == 0) {
        saveCalibrationData();
    }
    
}

void IMUSensor_GY85::adaptiveCalibrateMagnetometer(float* q) {
    // Получение оценки направления магнитного поля Земли из кватерниона ориентации
    float bx = 2.0f * (q[1]*q[3] - q[0]*q[2]);
    float by = 2.0f * (q[0]*q[1] + q[2]*q[3]);
    float bz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    
    float bNorm = sqrt(bx*bx + by*by + bz*bz);
    bx /= bNorm;
    by /= bNorm;
    bz /= bNorm;
    
    // Обновление смещений и масштабов по разнице между оценкой и измерением
    float magError;
    magError = currentData.mx - bx;
    calibData.magOffset[0] -= magError * ADAPTIVE_GAIN;
    calibData.magScale[0] *= 1.0f / (bx + magError);

    magError = currentData.my - by;
    calibData.magOffset[1] -= magError * ADAPTIVE_GAIN;
    calibData.magScale[1] *= 1.0f / (by + magError);

    magError = currentData.mz - bz;
    calibData.magOffset[2] -= magError * ADAPTIVE_GAIN;
    calibData.magScale[2] *= 1.0f / (bz + magError);
    
}

//-----------------------------------------------------------------------------
// Сохранение/загрузка калибровки
//-----------------------------------------------------------------------------

void IMUSensor_GY85::saveCalibrationData() {
    Preferences prefs;
    prefs.begin(prefsName.c_str(), false);
    prefs.putBytes("calibData", &calibData, sizeof(IMUCalibData));
    prefs.end();
}

bool IMUSensor_GY85::readCalibrationData() {
    Preferences prefs;
    prefs.begin(prefsName.c_str(), true);
    if(prefs.isKey("calibData")) {
        prefs.getBytes("calibData", &calibData, sizeof(IMUCalibData));
        calibValid = true;
        prefs.end();
        return true;
    }
    setDefaultCalibration();
    prefs.end();
    return false;
}

//-----------------------------------------------------------------------------
// Вспомогательные методы
//-----------------------------------------------------------------------------

void IMUSensor_GY85::setCalibrationData(const IMUCalibData data, bool save) {
    calibData = data;
    if(save) {
        saveCalibrationData();
    }
}

IMUCalibData IMUSensor_GY85::getCalibrationData() {
    return calibData;
}

IMUData IMUSensor_GY85::getData() {
    getData(currentData);
    return currentData;
}

OrientationData IMUSensor_GY85::getOrientation() {
    madgwick.getQuaternion(&currentOrientation.q0);
    currentOrientation.timestamp = millis();
    return currentOrientation;
}

void IMUSensor_GY85::setFrequency(int16_t frequency) {
    imuFrequency = frequency;
    madgwick.begin(imuFrequency);
}

int16_t IMUSensor_GY85::getFrequency() {
    return imuFrequency;
}

void IMUSensor_GY85::setLogLevel(int8_t level) {
    log_imu = level;
}

void IMUSensor_GY85::setLogStream(Stream* stream) {
    logStream = stream ? stream : &Serial;
}

bool IMUSensor_GY85::isCalibrationValid() {
    return calibValid;
}

IMUSensor_GY85::~IMUSensor_GY85() {
    // Деструктор
}