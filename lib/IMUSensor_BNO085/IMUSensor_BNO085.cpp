/**
 * @file IMUSensor_BNO085.cpp
 * @brief Реализация библиотеки для работы с IMU модулем BNO085
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#include "IMUSensor_BNO085.h"

// Статическая переменная для обработки прерываний
static IMUSensor_BNO085* instanceForISR = nullptr;

IMUSensor_BNO085::IMUSensor_BNO085(const char* prefsName, uint8_t i2cAddr, 
                                   int8_t interruptPin, Stream* logStream)
    : prefsName(prefsName),
      i2cAddress(i2cAddr),
      interruptPin(interruptPin),
      logStream(logStream ? logStream : &Serial),
      imuFrequency(BNO085_IMU_DEFAULT_FREQUENCY),
      magFrequency(BNO085_MAG_DEFAULT_FREQUENCY),
      sensorReady(false),
      calibrationSaved(false),
      lastSavedCalibrationTime(0),
      lastCalibrationCheck(0),
      calibrationCounter(0),
      quaternionCount(0),
      accelCount(0),
      gyroCount(0),
      magCount(0)
{
    // Инициализация структур данных
    memset(&currentData, 0, sizeof(IMUData));
    currentData.q0 = 1.0f;
    memset(&currentOrientation, 0, sizeof(OrientationData));
    currentOrientation.q0 = 1.0f;
    memset(&calibData, 0, sizeof(IMUCalibData));
    memset(&savedCalibration, 0, sizeof(BNO085CalibrationData));
    memset(&currentCalibration, 0, sizeof(BNO085CalibrationData));
    
    // Установка значений по умолчанию для совместимости
    for(int i = 0; i < 3; i++) {
        calibData.accelScale[i] = 1.0f;
        calibData.gyroScale[i] = 1.0f;
        calibData.magScale[i] = 1.0f;
        calibData.accelOffset[i] = 0;
        calibData.gyroOffset[i] = 0;
        calibData.magOffset[i] = 0.0f;
        calibData.magSI[i] = 0.0f;
    }
    
    instanceForISR = this;
}

bool IMUSensor_BNO085::begin(uint16_t imuFreq, uint16_t magFreq) {
    imuFrequency = imuFreq;
    magFrequency = magFreq;
    
    logStream->println("🚀 Инициализация BNO085...");
    
    // Настройка I2C
    Wire.begin();
    Wire.setClock(100000); // Начинаем с 100 кГц для стабильности
    
    // Сканирование I2C устройств
    logStream->println("Сканирование I2C устройств...");
    bool deviceFound = false;
    uint8_t bno085Address = 0;
    
    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            logStream->printf("I2C устройство найдено по адресу: 0x%02X\n", address);
            if (address == 0x4A || address == 0x4B) {
                bno085Address = address;
                deviceFound = true;
                logStream->printf("✓ BNO085 найден по адресу: 0x%02X\n", address);
                i2cAddress = address;
            }
        }
    }
    
    if (!deviceFound) {
        logStream->println("❌ BNO085 не найден по стандартным адресам (0x4A, 0x4B)");
        return false;
    }
    

    // Инициализация датчика
    logStream->println("Инициализация BNO085...");
    
    // Попробуем несколько способов инициализации
    bool initSuccess = false;
    
    // Способ 1: стандартная инициализация
    if (myIMU.begin() == true) {
        initSuccess = true;
        logStream->println("✓ Инициализация успешна (способ 1)");
    } else {
        logStream->println("Способ 1 не удался, пробуем способ 2...");
        
        // Способ 2: инициализация с указанием адреса
        if (myIMU.begin(bno085Address) == true) {
            initSuccess = true;
            logStream->println("✓ Инициализация успешна (способ 2 с адресом)");
        } else {
            logStream->println("Способ 2 не удался, пробуем способ 3...");
            
            // Способ 3: инициализация с Wire объектом
            if (myIMU.begin(bno085Address, Wire) == true) {
                initSuccess = true;
                logStream->println("✓ Инициализация успешна (способ 3 с Wire)");
            }
        }
    }
    
    if (!initSuccess) {
        logStream->println("❌ Все способы инициализации не удались!");
        return false;
    }
    
    logStream->println("✅ BNO085 подключен успешно!");
    
    // Увеличиваем частоту I2C после успешного подключения
    Wire.setClock(400000); // 400 кГц
    logStream->println("I2C частота увеличена до 400 кГц");
    
    // Настройка отчетов датчика
    setReports();
    
    // Попытка загрузить сохраненную калибровку
    if (readCalibrationData()) {
        logStream->printf("✅ Калибровочные данные загружены (точность: %.4f рад)\n", 
                         savedCalibration.quatAccuracy);
    } else {
        logStream->println("🔄 Начинаем процесс калибровки...");
        logStream->println("Инструкции для высокоточной калибровки:");
        logStream->println("1. Медленно поворачивайте датчик во всех направлениях");
        logStream->println("2. Выполните движения по восьмерке (для магнетометра)");
        logStream->println("3. Держите датчик неподвижно несколько секунд");
        logStream->println("4. Система автоматически сохранит превосходную калибровку");
        logStream->println("5. Цель: точность < 0.05 рад (2.9°)");
    }
    
    logStream->println("Мягкий сброс...");
//    myIMU.softReset(); // Сброс для применения настроек
    delay(100);
    
    sensorReady = true;
    return true;
}

void IMUSensor_BNO085::setReports() {
    logStream->println("🎯 Настройка режима высокой точности на 100 Гц...");
    
    uint16_t interval = 10; // 100 Гц = 10 мс интервал
    
    // Включаем кватернион на точно 100 Гц (10 мс)
    if (myIMU.enableRotationVector(interval) == true) {
        logStream->println("✅ Кватернион включен: 100 Гц (режим высокой точности)");
    } else {
        logStream->println("❌ Не удалось включить кватернион на 100 Гц");
        return;
    }
    
    // Включаем дополнительные сенсоры для повышения точности слияния данных
    
    // Включаем высокоточный гироскоп (100 Гц)
    if (myIMU.enableGyro(interval) == true) {
        logStream->println("✅ Гироскоп включен: 100 Гц");
    } else {
        logStream->println("⚠️  Гироскоп не включен");
    }
    
    // Включаем калиброванный акселерометр (100 Гц)
    if (myIMU.enableAccelerometer(interval) == true) {
        logStream->println("✅ Акселерометр включен: 100 Гц");
    } else {
        logStream->println("⚠️  Акселерометр не включен");
    }
    
    // Включаем калиброванный магнетометр (100 Гц)
    if (myIMU.enableMagnetometer(interval) == true) {
        logStream->println("✅ Магнетометр включен: 100 Гц");
    } else {
        logStream->println("⚠️  Магнетометр не включен");
    }
    
    // Включаем классификатор стабильности (помогает понять точность)
    if (myIMU.enableStabilityClassifier(100) == true) {
        logStream->println("✅ Классификатор стабильности включен: 10 Гц");
    } else {
        logStream->println("⚠️  Классификатор стабильности не включен");
    }
    
    logStream->println("🔧 Режим высокой точности настроен!");
}



IMUData IMUSensor_BNO085::readData() {
    if (!sensorReady) {
        return currentData;
    }

    // Проверяем наличие новых данных
    if (myIMU.getSensorEvent() == true) {
        uint8_t sensorID = myIMU.getSensorEventID();
        
        // Основной кватернион и все сенсоры (100 Гц)
        if (sensorID == SENSOR_REPORTID_ROTATION_VECTOR) {
            // Кватернион
            currentData.q1 = myIMU.getQuatI();
            currentData.q2 = myIMU.getQuatJ();
            currentData.q3 = myIMU.getQuatK();
            currentData.q0 = myIMU.getQuatReal();
            
            // Акселерометр (м/с²)
            currentData.ax = myIMU.getAccelX();
            currentData.ay = myIMU.getAccelY();
            currentData.az = myIMU.getAccelZ();
            
            // Гироскоп (рад/с) 
            currentData.gx = myIMU.getGyroX();
            currentData.gy = myIMU.getGyroY();
            currentData.gz = myIMU.getGyroZ();
            
            // Магнетометр (мкТл)
            currentData.mx = myIMU.getMagX();
            currentData.my = myIMU.getMagY();
            currentData.mz = myIMU.getMagZ();
            
            // Сырые данные магнитометра (для совместимости)
            currentData.mag_x = (int16_t)currentData.mx;
            currentData.mag_y = (int16_t)currentData.my;
            currentData.mag_z = (int16_t)currentData.mz;
            
            currentData.timestamp = millis();
            
            // Увеличиваем счетчики
            quaternionCount++;
            accelCount++;
            gyroCount++;
            magCount++;
            
            // Проверка и сохранение калибровки
            checkAndSaveCalibration();
        }
    }
    
    return currentData;
}

IMUData IMUSensor_BNO085::getData() {
    return currentData;
}

OrientationData IMUSensor_BNO085::updateOrientation() {
    currentOrientation.q0 = currentData.q0;
    currentOrientation.q1 = currentData.q1;
    currentOrientation.q2 = currentData.q2;
    currentOrientation.q3 = currentData.q3;
    currentOrientation.timestamp = millis();
    
    return currentOrientation;
}

OrientationData IMUSensor_BNO085::getOrientation() {
    return currentOrientation;
}

void IMUSensor_BNO085::checkAndSaveCalibration() {
    if (millis() - lastCalibrationCheck < BNO085_CALIBRATION_CHECK_INTERVAL) {
        return; // Проверяем каждые 5 секунд
    }
    
    lastCalibrationCheck = millis();
    
    // Получаем текущую точность кватерниона
    float quatAccuracy = myIMU.getQuatRadianAccuracy();
    uint8_t qualityLevel = evaluateCalibrationQuality(quatAccuracy);
    
    // Обновляем текущую калибровку
    currentCalibration.quatAccuracy = quatAccuracy;
    currentCalibration.qualityLevel = qualityLevel;
    currentCalibration.timestamp = millis();
    currentCalibration.isValid = true;
    currentCalibration.measurementCount++;
    
    // Выводим статус калибровки
//    logStream->printf("📊 Калибровка: уровень %d (точность: %.4f рад)\n", 
//                     qualityLevel, quatAccuracy);
    
    // Проверяем, нужно ли сохранить калибровку
    bool shouldSave = false;
    
    if (!calibrationSaved) {
        // Первое сохранение - требуем высокое качество
        if (qualityLevel >= 2) {
            calibrationCounter++;
            logStream->printf("  ✨ Хорошая точность обнаружена (%d/3)\n", calibrationCounter);
            
            if (calibrationCounter >= 3) {
                shouldSave = true;
                logStream->println("🎉 ПЕРВАЯ КАЛИБРОВКА ДОСТИГНУТА! Сохраняем данные...");
            }
        } else {
            calibrationCounter = 0; // Сброс счетчика если качество упало
        }
    } else {
        // Проверяем улучшение калибровки
        if (hasImprovedCalibration()) {
            shouldSave = true;
            logStream->println("🌟 УЛУЧШЕННАЯ КАЛИБРОВКА! Обновляем данные...");
        }
    }
    
    if (shouldSave) {
        saveCalibrationData();
        calibrationSaved = true;
        calibrationCounter = 0;
    }
}

uint8_t IMUSensor_BNO085::evaluateCalibrationQuality(float accuracy) {
    if (accuracy < 0.05) return 3;        // Превосходная
    else if (accuracy < 0.1) return 2;    // Хорошая
    else if (accuracy < 0.2) return 1;    // Удовлетворительная
    else return 0;                        // Плохая
}

bool IMUSensor_BNO085::hasImprovedCalibration() {
    if (!savedCalibration.isValid) return false;
    
    // Считаем улучшением, если точность стала значительно лучше
    float improvement = savedCalibration.quatAccuracy - currentCalibration.quatAccuracy;
    return (improvement > 0.01); // Улучшение на 0.01 рад или больше
}

bool IMUSensor_BNO085::readCalibrationData() {
    Preferences prefs;
    if (!prefs.begin(prefsName.c_str(), true)) {
        return false;
    }
    
    // Проверяем наличие сохраненных данных
    size_t dataSize = prefs.getBytesLength("bno085_calib");
    if (dataSize != sizeof(BNO085CalibrationData)) {
        prefs.end();
        return false;
    }
    
    // Читаем калибровочные данные BNO085
    size_t readSize = prefs.getBytes("bno085_calib", &savedCalibration, sizeof(BNO085CalibrationData));
    prefs.end();
    
    bool success = (readSize == sizeof(BNO085CalibrationData) && savedCalibration.isValid);
    
    if (success) {
        calibrationSaved = true;
        logStream->printf("Калибровочные данные загружены: точность %.4f рад, уровень %d\n",
                         savedCalibration.quatAccuracy, savedCalibration.qualityLevel);
    }
    
    return success;
}

void IMUSensor_BNO085::saveCalibrationData() {
    Preferences prefs;
    if (!prefs.begin(prefsName.c_str(), false)) {
        logStream->println("❌ Ошибка открытия NVS для сохранения");
        return;
    }
    
    // Обновляем сохраненную калибровку
    savedCalibration = currentCalibration;
    
    // Сохраняем калибровочные данные BNO085
    size_t written = prefs.putBytes("bno085_calib", &savedCalibration, sizeof(BNO085CalibrationData));
    prefs.end();
    
    if (written == sizeof(BNO085CalibrationData)) {
        logStream->printf("✅ Калибровочные данные сохранены! Точность: %.4f рад, уровень: %d\n",
                         savedCalibration.quatAccuracy, savedCalibration.qualityLevel);
        lastSavedCalibrationTime = millis();
    } else {
        logStream->println("❌ Ошибка сохранения калибровочных данных");
    }
}

void IMUSensor_BNO085::calibrate() {
    logStream->println("🔄 BNO085 имеет автоматическую калибровку");
    logStream->println("Следуйте инструкциям для получения лучшей точности:");
    logStream->println("1. Медленно поворачивайте датчик во всех направлениях");
    logStream->println("2. Выполните движения по восьмерке (для магнетометра)");
    logStream->println("3. Держите датчик неподвижно несколько секунд");
    logStream->println("Калибровка будет сохранена автоматически при достижении хорошей точности");
}

void IMUSensor_BNO085::calibrateCompass() {
    logStream->println("🧲 Калибровка компаса BNO085:");
    logStream->println("Медленно двигайте датчик по восьмерке в течение 30 секунд");
    
    uint32_t startTime = millis();
    while (millis() - startTime < 30000) {
        readData();
        float accuracy = getQuaternionAccuracy();
        
        if ((millis() - startTime) % 5000 == 0) {
            logStream->printf("Точность: %.4f рад\n", accuracy);
        }
        
        delay(100);
    }
    
    logStream->println("Калибровка компаса завершена");
}

bool IMUSensor_BNO085::isCalibrationValid() {
    return calibrationSaved && savedCalibration.isValid && (savedCalibration.qualityLevel >= 1);
}

bool IMUSensor_BNO085::isFullyCalibrated() {
    uint8_t quality = getCalibrationQuality();
    return (quality >= 3); // Превосходная калибровка
}

void IMUSensor_BNO085::getCalibrationStatus(uint8_t* quality, float* accuracy) {
    if (quality) *quality = currentCalibration.qualityLevel;
    if (accuracy) *accuracy = currentCalibration.quatAccuracy;
}

float IMUSensor_BNO085::getQuaternionAccuracy() {
    return myIMU.getQuatRadianAccuracy();
}

uint8_t IMUSensor_BNO085::getCalibrationQuality() {
    float accuracy = getQuaternionAccuracy();
    return evaluateCalibrationQuality(accuracy);
}

void IMUSensor_BNO085::printCalibrationStatus() {
    uint8_t quality;
    float accuracy;
    getCalibrationStatus(&quality, &accuracy);
    
    logStream->printf("📊 Статус калибровки BNO085:\n");
    logStream->printf("   Уровень качества: %d (0-3)\n", quality);
    logStream->printf("   Точность кватерниона: %.4f рад\n", accuracy);
    logStream->printf("   Сохранена: %s\n", calibrationSaved ? "Да" : "Нет");
    
    switch(quality) {
        case 3: logStream->println("   🟢 Превосходная точность!"); break;
        case 2: logStream->println("   🟡 Хорошая точность"); break;
        case 1: logStream->println("   🟠 Средняя точность"); break;
        default: logStream->println("   🔴 Требуется калибровка"); break;
    }
}


void IMUSensor_BNO085::printStatistics() {
    uint32_t uptime = millis();
    float uptimeSeconds = uptime / 1000.0f;
    
    logStream->printf("\n📊 Статистика BNO085 (время работы: %.1f сек):\n", uptimeSeconds);
    logStream->printf("   🔄 Кватернион: %lu отсчетов (%.1f Гц)\n", 
                     quaternionCount, quaternionCount / uptimeSeconds);
    logStream->printf("   📐 Акселерометр: %lu отсчетов (%.1f Гц)\n", 
                     accelCount, accelCount / uptimeSeconds);
    logStream->printf("   🌪  Гироскоп: %lu отсчетов (%.1f Гц)\n", 
                     gyroCount, gyroCount / uptimeSeconds);
    logStream->printf("   🧲 Магнетометр: %lu отсчетов (%.1f Гц)\n", 
                     magCount, magCount / uptimeSeconds);
    
    uint8_t quality;
    float accuracy;
    getCalibrationStatus(&quality, &accuracy);
    logStream->printf("   🎯 Калибровка: уровень %d, точность %.4f рад\n", quality, accuracy);
    
    if (calibrationSaved) {
        logStream->printf("   💾 Последнее сохранение: %.1f сек назад\n", 
                         (millis() - lastSavedCalibrationTime) / 1000.0f);
    }
}

IMUCalibData IMUSensor_BNO085::getCalibrationData() {
    return calibData;
}

void IMUSensor_BNO085::setCalibrationData(const IMUCalibData calib, bool save) {
    calibData = calib;
    logStream->println("Установлены новые калибровочные данные (для совместимости)");
}

void IMUSensor_BNO085::softReset() {
    if (sensorReady) {
        myIMU.softReset();
        logStream->println("🔄 Выполнен мягкий сброс BNO085");
    }
}

bool IMUSensor_BNO085::isReady() {
    return sensorReady;
}


