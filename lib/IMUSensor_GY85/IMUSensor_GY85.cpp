/**
 * @file IMUSensor_GY85.cpp
 * @brief Реализация библиотеки для работы с IMU модулем GY-85
 */

#include "IMUSensor_GY85.h"
#include <Preferences.h>

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
    
    // Калибровка магнитометра методом Ellipsoid Fitting
    logStream->println("Rotate device in all directions for magnetometer calibration");    
    const int samples_mag = 1000;
    float magX[samples_mag], magY[samples_mag], magZ[samples_mag];
    
    for(int i = 0; i < samples_mag; i++) {
        int mx, my, mz;
        mag.read(&mx, &my, &mz);
        magX[i] = mx;
        magY[i] = my;  
        magZ[i] = mz;
        delay(10);
    }
    
    // Вычисление параметров эллипсоида
    float centerX, centerY, centerZ;
    float scaleX, scaleY, scaleZ;
    ellipsoidFitting(magX, magY, magZ, samples_mag, 
                     &centerX, &centerY, &centerZ, 
                     &scaleX, &scaleY, &scaleZ);
    
    // Установка калибровочных параметров
    calibData.magOffset[0] = (int16_t)centerX;
    calibData.magOffset[1] = (int16_t)centerY;
    calibData.magOffset[2] = (int16_t)centerZ;
    
    calibData.magScale[0] = scaleX;
    calibData.magScale[1] = scaleY;
    calibData.magScale[2] = scaleZ;
    
    saveCalibrationData();
    calibValid = true;
    logStream->println("Calibration complete!");
}

void IMUSensor_GY85::ellipsoidFitting(float* x, float* y, float* z, int n,
                                      float* centerX, float* centerY, float* centerZ,
                                      float* scaleX, float* scaleY, float* scaleZ) {
    // Вычисление средних значений
    float avgX = 0, avgY = 0, avgZ = 0;
    for(int i = 0; i < n; i++) {
        avgX += x[i];
        avgY += y[i];
        avgZ += z[i];
    }
    avgX /= n;
    avgY /= n;
    avgZ /= n;

    // Вычисление матрицы ковариации
    float covXX = 0, covXY = 0, covXZ = 0, covYY = 0, covYZ = 0, covZZ = 0;
    for(int i = 0; i < n; i++) {
        float dx = x[i] - avgX;
        float dy = y[i] - avgY;
        float dz = z[i] - avgZ;
        covXX += dx * dx;
        covXY += dx * dy;
        covXZ += dx * dz;
        covYY += dy * dy;
        covYZ += dy * dz;
        covZZ += dz * dz;
    }
    covXX /= n;
    covXY /= n;
    covXZ /= n;
    covYY /= n;
    covYZ /= n;
    covZZ /= n;

    // Вычисление собственных значений и векторов матрицы ковариации
    float a = covYY*covZZ - covYZ*covYZ;
    float b = covXZ*covYZ - covXY*covZZ;
    float c = covXY*covYZ - covXZ*covYY;
    float d = covXX*covZZ - covXZ*covXZ;
    float e = covXX*covYY - covXY*covXY;
    float f = covXX*a + covXY*b + covXZ*c;

    float q = (covXX + covYY + covZZ) / 3;
    float p = (covXX - q)*(covXX - q) + (covYY - q)*(covYY - q) + (covZZ - q)*(covZZ - q) + 2*(covXY*covXY + covXZ*covXZ + covYZ*covYZ);
    p = sqrt(p / 6);

    float r = (1/p) * ((covXX - q)*(covYY - q)*(covZZ - q) + 2*covXY*covXZ*covYZ - (covXX - q)*covYZ*covYZ - (covYY - q)*covXZ*covXZ - (covZZ - q)*covXY*covXY);
    r = r / 2;

    float phi = acos(r) / 3;
    float eig1 = q + 2*p*cos(phi);
    float eig2 = q + 2*p*cos(phi + 2*M_PI/3);
    float eig3 = 3*q - eig1 - eig2;

    float vec1X = (a*(eig1 - covYY) - b*covXY) / (covXY*(eig1 - covYY) - (eig1 - covXX)*covYZ);
    float vec1Y = (b*(eig1 - covXX) - a*covXY) / (covXY*(eig1 - covYY) - (eig1 - covXX)*covYZ);
    float vec1Z = 1;

    float vec2X = (a*(eig2 - covYY) - b*covXY) / (covXY*(eig2 - covYY) - (eig2 - covXX)*covYZ);
    float vec2Y = (b*(eig2 - covXX) - a*covXY) / (covXY*(eig2 - covYY) - (eig2 - covXX)*covYZ);
    float vec2Z = 1;

    float vec3X = vec1Y*vec2Z - vec1Z*vec2Y;
    float vec3Y = vec1Z*vec2X - vec1X*vec2Z;
    float vec3Z = vec1X*vec2Y - vec1Y*vec2X;

    // Нормализация собственных векторов
    float norm1 = sqrt(vec1X*vec1X + vec1Y*vec1Y + vec1Z*vec1Z);
    vec1X /= norm1;
    vec1Y /= norm1;
    vec1Z /= norm1;

    float norm2 = sqrt(vec2X*vec2X + vec2Y*vec2Y + vec2Z*vec2Z);
    vec2X /= norm2;
    vec2Y /= norm2;
    vec2Z /= norm2;

    float norm3 = sqrt(vec3X*vec3X + vec3Y*vec3Y + vec3Z*vec3Z);
    vec3X /= norm3;
    vec3Y /= norm3;
    vec3Z /= norm3;

    // Вычисление параметров эллипсоида
    *centerX = avgX;
    *centerY = avgY;
    *centerZ = avgZ;

    *scaleX = 1.0f / sqrt(eig1);
    *scaleY = 1.0f / sqrt(eig2);
    *scaleZ = 1.0f / sqrt(eig3);
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