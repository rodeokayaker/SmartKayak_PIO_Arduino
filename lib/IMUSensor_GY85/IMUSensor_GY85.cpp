/**
 * @file IMUSensor_GY85.cpp
 * @brief Реализация библиотеки для работы с IMU модулем GY-85
 */

#include "IMUSensor_GY85.h"
#include <Preferences.h>

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
    imuFrequency(GY85_IMU_DEFAULT_FREQUENCY) {
    
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
    
    // Калибровка магнитометра
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
    data.mx = ((float)mx + calibData.magOffset[0]) * calibData.magScale[0];
    data.my = ((float)my + calibData.magOffset[1]) * calibData.magScale[1];
    data.mz = ((float)mz + calibData.magOffset[2]) * calibData.magScale[2];

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
    madgwick.update(
        currentData.gx, currentData.gy, currentData.gz,
        currentData.ax, currentData.ay, currentData.az,
        currentData.mx, currentData.my, currentData.mz
    );
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

IMUCalibData& IMUSensor_GY85::getCalibrationData() {
    return calibData;
}

IMUSensor_GY85::~IMUSensor_GY85() {
    // Деструктор
}