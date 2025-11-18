/**
 * Пример полной интеграции HybridOrientationEstimator в SmartKayak
 * 
 * Этот файл демонстрирует:
 * 1. Настройку системы
 * 2. Калибровку магнитометра
 * 3. Использование в основном цикле
 * 4. Диагностику и отладку
 */

#include <Arduino.h>
#include "SmartKayak.h"
#include "HybridOrientationEstimator.h"
#include "MagnetometerCalibrator.h"

// Глобальные объекты
SmartKayak smartKayak;
HybridOrientationEstimator orientationEstimator;
MagnetometerCalibrator paddleMagCalibrator;
MagnetometerCalibrator kayakMagCalibrator;
MagneticInterferenceDetector interferenceDetector;

// Флаги режимов
bool calibrationMode = false;
bool diagnosticMode = false;

// ============================================================================
// SETUP - Инициализация системы
// ============================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("\n\n");
    Serial.println("╔══════════════════════════════════════════════════╗");
    Serial.println("║     SmartKayak Hybrid Orientation System        ║");
    Serial.println("║         Высокоточная ориентация весла            ║");
    Serial.println("╚══════════════════════════════════════════════════╝");
    Serial.println();
    
    // Инициализация SmartKayak (ваш существующий код)
    smartKayak.begin();
    
    // Настройка гибридного оценщика ориентации
    setupOrientationEstimator();
    
    // Загрузка сохраненной калибровки (если есть)
    loadCalibrations();
    
    // Меню
    printMenu();
}

// ============================================================================
// НАСТРОЙКА ГИБРИДНОГО ОЦЕНЩИКА
// ============================================================================

void setupOrientationEstimator() {
    Serial.println("⚙️  Настройка гибридного оценщика ориентации...");
    
    // 1. Настройка детектора магнитных аномалий
    MagneticAnomalyDetector anomalyDetector;
    
    // Для условий на воде с возможными помехами от мотора
    anomalyDetector.varianceThreshold = 40.0f;   // μT² - средняя чувствительность
    anomalyDetector.magnitudeMin = 30.0f;        // μT - широкий диапазон
    anomalyDetector.magnitudeMax = 70.0f;        // μT - для средних широт
    anomalyDetector.windowSize = 15;             // размер окна анализа
    anomalyDetector.smoothingFactor = 0.85f;     // сглаживание
    
    orientationEstimator.setAnomalyDetector(anomalyDetector);
    
    // 2. Настройка весов фильтра
    // Формат: (gyro, accel, mag, pattern)
    orientationEstimator.setFilterWeights(
        0.98f,  // DMP (гироскоп) - основа
        0.02f,  // Акселерометр (уже в DMP)
        0.35f,  // Магнитометр - адаптивный вес
        0.25f   // Паттерны гребли - для коррекции дрифта
    );
    
    Serial.println("   ✅ Детектор аномалий настроен");
    Serial.println("   ✅ Веса фильтра установлены");
    Serial.println("   ✅ Система готова к работе\n");
}

// ============================================================================
// КАЛИБРОВКА МАГНИТОМЕТРОВ
// ============================================================================

void calibratePaddleMagnetometer() {
    Serial.println("\n╔══════════════════════════════════════════════════╗");
    Serial.println("║       КАЛИБРОВКА МАГНИТОМЕТРА ВЕСЛА              ║");
    Serial.println("╚══════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("📋 Инструкции:");
    Serial.println("   1. Медленно вращайте весло вокруг оси (360°)");
    Serial.println("   2. Наклоняйте весло вперед-назад");
    Serial.println("   3. Делайте восьмерки в воздухе");
    Serial.println("   4. Имитируйте все фазы гребка");
    Serial.println();
    Serial.println("⏱️  Время: 30-60 секунд");
    Serial.println("⌨️  Нажмите 's' для остановки");
    Serial.println();
    
    paddleMagCalibrator.startCalibration();
    calibrationMode = true;
    
    unsigned long startTime = millis();
    
    while (calibrationMode) {
        // Основной цикл обновления
        smartKayak.update();
        
        // Добавляем образцы в калибратор
        IMUData paddleIMU = smartKayak.paddle->getIMUData();
        paddleMagCalibrator.addSample(paddleIMU);
        
        // Показываем прогресс каждые 5 секунд
        if ((millis() - startTime) % 5000 < 20) {
            Serial.printf("   📊 Прогресс: %.1f%%\n", 
                         paddleMagCalibrator.getCalibrationProgress());
        }
        
        // Проверка команд
        if (Serial.available()) {
            char cmd = Serial.read();
            if (cmd == 's' || cmd == 'S') {
                calibrationMode = false;
            }
        }
        
        delay(10);
    }
    
    // Завершение калибровки
    paddleMagCalibrator.stopCalibration();
    
    // Вывод результатов
    if (paddleMagCalibrator.isCalibrationValid()) {
        Serial.println("\n✅ Калибровка весла УСПЕШНА!");
        paddleMagCalibrator.printCalibrationResult(&Serial);
        paddleMagCalibrator.saveCalibration();
    } else {
        Serial.println("\n❌ Калибровка весла НЕУДАЧНА!");
        Serial.println("   Повторите процедуру с большим покрытием движений");
    }
    
    Serial.println("\nНажмите любую клавишу для продолжения...");
    while (!Serial.available()) delay(10);
    while (Serial.available()) Serial.read();
    
    printMenu();
}

void calibrateKayakMagnetometer() {
    Serial.println("\n╔══════════════════════════════════════════════════╗");
    Serial.println("║       КАЛИБРОВКА МАГНИТОМЕТРА КАЯКА              ║");
    Serial.println("╚══════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("📋 Инструкции:");
    Serial.println("   1. Медленно поверните каяк на 360°");
    Serial.println("   2. Наклоните каяк вперед-назад");
    Serial.println("   3. Наклоните каяк влево-вправо");
    Serial.println();
    Serial.println("⏱️  Время: 30-60 секунд");
    Serial.println("⌨️  Нажмите 's' для остановки");
    Serial.println();
    
    kayakMagCalibrator.startCalibration();
    calibrationMode = true;
    
    unsigned long startTime = millis();
    
    while (calibrationMode) {
        smartKayak.update();
        
        IMUData kayakIMU;
        smartKayak.imu->getData(kayakIMU);
        kayakMagCalibrator.addSample(kayakIMU);
        
        if ((millis() - startTime) % 5000 < 20) {
            Serial.printf("   📊 Прогресс: %.1f%%\n", 
                         kayakMagCalibrator.getCalibrationProgress());
        }
        
        if (Serial.available()) {
            char cmd = Serial.read();
            if (cmd == 's' || cmd == 'S') {
                calibrationMode = false;
            }
        }
        
        delay(10);
    }
    
    kayakMagCalibrator.stopCalibration();
    
    if (kayakMagCalibrator.isCalibrationValid()) {
        Serial.println("\n✅ Калибровка каяка УСПЕШНА!");
        kayakMagCalibrator.printCalibrationResult(&Serial);
        kayakMagCalibrator.saveCalibration();
    } else {
        Serial.println("\n❌ Калибровка каяка НЕУДАЧНА!");
        Serial.println("   Повторите процедуру с полным поворотом (360°)");
    }
    
    Serial.println("\nНажмите любую клавишу для продолжения...");
    while (!Serial.available()) delay(10);
    while (Serial.available()) Serial.read();
    
    printMenu();
}

// ============================================================================
// ОСНОВНОЙ ЦИКЛ С ГИБРИДНОЙ ОРИЕНТАЦИЕЙ
// ============================================================================

void loop() {
    // Проверка команд пользователя
    if (Serial.available()) {
        char cmd = Serial.read();
        handleCommand(cmd);
    }
    
    // Обновление SmartKayak с гибридной ориентацией
    updateWithHybridOrientation();
    
    // Диагностика (если включена)
    if (diagnosticMode) {
        printDiagnostics();
    }
    
    delay(10);
}

// ============================================================================
// ОБНОВЛЕНИЕ С ГИБРИДНОЙ ОРИЕНТАЦИЕЙ
// ============================================================================

void updateWithHybridOrientation() {
    // Проверки соединения и данных (из вашего оригинального кода)
    if (!smartKayak.paddle->connected()) {
        smartKayak.motorDriver->stop();
        return;
    }
    
    OrientationData paddleOrientation = smartKayak.paddle->getOrientationData();
    if (paddleOrientation.q0 == 0 && paddleOrientation.q1 == 0 && 
        paddleOrientation.q2 == 0 && paddleOrientation.q3 == 0) {
        return;
    }
    
    // Получение данных IMU
    IMUData paddleIMU = smartKayak.paddle->getIMUData();
    IMUData kayakIMU;
    smartKayak.imu->getData(kayakIMU);
    
    // Применение калибровки магнитометра (если доступна)
    if (paddleMagCalibrator.isCalibrationValid()) {
        paddleMagCalibrator.applyCalibration(paddleIMU);
    }
    if (kayakMagCalibrator.isCalibrationValid()) {
        kayakMagCalibrator.applyCalibration(kayakIMU);
    }
    
    // Анализ магнитных помех
    auto interferenceReport = interferenceDetector.analyze(paddleIMU);
    if (interferenceReport.actionRequired && diagnosticMode) {
        Serial.printf("⚠️  %s\n", interferenceReport.description.c_str());
    }
    
    // Подготовка данных ориентации каяка
    OrientationData kayakOrientation;
    kayakOrientation.q0 = smartKayak.kayakOrientationQuat[0];
    kayakOrientation.q1 = smartKayak.kayakOrientationQuat[1];
    kayakOrientation.q2 = smartKayak.kayakOrientationQuat[2];
    kayakOrientation.q3 = smartKayak.kayakOrientationQuat[3];
    
    // ★★★ ГЛАВНОЕ: Получение высокоточной относительной ориентации ★★★
    SP_Math::Quaternion paddleRelativeQuat = 
        orientationEstimator.updateRelativeOrientation(
            paddleIMU,           // IMU весла
            paddleOrientation,   // DMP кватернион весла
            kayakIMU,            // IMU каяка
            kayakOrientation     // DMP кватернион каяка
        );
    
    // Определение углов весла
    float shaftRotationAngle, shaftTiltAngle, bladeRotationAngle;
    getPaddleAngles(paddleRelativeQuat, 
                    shaftRotationAngle, shaftTiltAngle, bladeRotationAngle);
    
    // Определение активной лопасти
    SP_Math::Quaternion currentPaddleQ(
        paddleOrientation.q0, paddleOrientation.q1, 
        paddleOrientation.q2, paddleOrientation.q3
    );
    BladeSideType bladeSide = getLowerBladeSide(
        currentPaddleQ, 
        smartKayak.paddle->getBladeAngles().YAxisDirection
    );
    
    if (bladeSide == BladeSideType::ALL_BLADES) {
        smartKayak.motorDriver->stop();
        return;
    }
    
    // Получение силы на лопасти
    loadData loads = smartKayak.paddle->getLoadData();
    float bladeForce = (bladeSide == BladeSideType::RIGHT_BLADE) ? 
                       loads.forceR : loads.forceL;
    
    // ★★★ Детекция фазы гребка ★★★
    StrokePhase::Phase currentPhase = orientationEstimator.detectStrokePhase(
        shaftRotationAngle,
        shaftTiltAngle,
        bladeForce
    );
    
    // Расчет силы мотора с учетом фазы (опционально)
    int motorForce = calculateMotorForceWithPhase(
        bladeForce, 
        currentPhase, 
        orientationEstimator.getPhaseConfidence()
    );
    
    // Установка силы мотора
    smartKayak.motorDriver->setForce(motorForce);
    
    // Обновление дисплея
    updateDisplay(shaftRotationAngle, shaftTiltAngle, bladeRotationAngle, 
                  currentPhase, motorForce);
}

// ============================================================================
// РАСЧЕТ СИЛЫ МОТОРА С УЧЕТОМ ФАЗЫ ГРЕБКА
// ============================================================================

int calculateMotorForceWithPhase(float bladeForce, 
                                  StrokePhase::Phase phase, 
                                  float confidence) {
    // Базовая сила (ваша существующая логика)
    int baseForce = (int)(bladeForce * 2.6f);  // Пример: средняя мощность
    
    // Модуляция по фазе (если уверенность высокая)
    if (confidence > 0.7f) {
        switch (phase) {
            case StrokePhase::CATCH:
                // Начало гребка - плавное нарастание
                baseForce *= 0.8f;
                break;
                
            case StrokePhase::PULL:
                // Основная фаза - максимальная поддержка
                baseForce *= 1.2f;
                break;
                
            case StrokePhase::RELEASE:
                // Выход - снижаем
                baseForce *= 0.6f;
                break;
                
            case StrokePhase::RECOVERY:
                // Возврат - минимум
                baseForce *= 0.3f;
                break;
                
            default:
                break;
        }
    }
    
    return baseForce;
}

// ============================================================================
// ДИАГНОСТИКА
// ============================================================================

void printDiagnostics() {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint < 2000) return;  // Каждые 2 секунды
    lastPrint = millis();
    
    Serial.println("\n╔══════════════════════════════════════════════════╗");
    Serial.println("║              ДИАГНОСТИКА СИСТЕМЫ                 ║");
    Serial.println("╚══════════════════════════════════════════════════╝");
    
    // Надежность магнитометра
    float magRel = orientationEstimator.getMagReliability();
    Serial.printf("🧭 Надежность магнитометра: %.1f%% ", magRel * 100);
    if (magRel > 0.7f) {
        Serial.println("✅ ОТЛИЧНО");
    } else if (magRel > 0.4f) {
        Serial.println("⚠️  СРЕДНЕ");
    } else {
        Serial.println("❌ ПЛОХО (коррекция по паттернам)");
    }
    
    // Текущая фаза гребка
    Serial.printf("🚣 Фаза гребка: ");
    switch (orientationEstimator.getCurrentPhase()) {
        case StrokePhase::CATCH:
            Serial.println("🎯 ЗАХВАТ");
            break;
        case StrokePhase::PULL:
            Serial.println("💪 ПРОВОДКА");
            break;
        case StrokePhase::RELEASE:
            Serial.println("🚀 ВЫХОД");
            break;
        case StrokePhase::RECOVERY:
            Serial.println("🔄 ВОЗВРАТ");
            break;
        default:
            Serial.println("❓ НЕИЗВЕСТНО");
            break;
    }
    Serial.printf("   Уверенность: %.1f%%\n", 
                  orientationEstimator.getPhaseConfidence() * 100);
    
    // Калибровки
    Serial.printf("📊 Калибровка весла: %s\n", 
                  paddleMagCalibrator.isCalibrationValid() ? "✅ ДА" : "❌ НЕТ");
    Serial.printf("📊 Калибровка каяка: %s\n", 
                  kayakMagCalibrator.isCalibrationValid() ? "✅ ДА" : "❌ НЕТ");
    
    Serial.println("══════════════════════════════════════════════════\n");
}

// ============================================================================
// МЕНЮ И КОМАНДЫ
// ============================================================================

void printMenu() {
    Serial.println("\n╔══════════════════════════════════════════════════╗");
    Serial.println("║                  ГЛАВНОЕ МЕНЮ                    ║");
    Serial.println("╠══════════════════════════════════════════════════╣");
    Serial.println("║  1 - Калибровка магнитометра весла               ║");
    Serial.println("║  2 - Калибровка магнитометра каяка               ║");
    Serial.println("║  3 - Сброс калибровок                            ║");
    Serial.println("║  d - Диагностический режим (вкл/выкл)            ║");
    Serial.println("║  r - Сброс истории гребков                       ║");
    Serial.println("║  h - Помощь (это меню)                           ║");
    Serial.println("╚══════════════════════════════════════════════════╝\n");
}

void handleCommand(char cmd) {
    switch (cmd) {
        case '1':
            calibratePaddleMagnetometer();
            break;
            
        case '2':
            calibrateKayakMagnetometer();
            break;
            
        case '3':
            Serial.println("🔄 Сброс всех калибровок...");
            paddleMagCalibrator.resetHistory();
            kayakMagCalibrator.resetHistory();
            orientationEstimator.resetHistory();
            Serial.println("✅ Калибровки сброшены\n");
            break;
            
        case 'd':
        case 'D':
            diagnosticMode = !diagnosticMode;
            Serial.printf("📊 Диагностический режим: %s\n\n", 
                         diagnosticMode ? "ВКЛЮЧЕН" : "ВЫКЛЮЧЕН");
            break;
            
        case 'r':
        case 'R':
            orientationEstimator.resetHistory();
            Serial.println("🔄 История гребков сброшена\n");
            break;
            
        case 'h':
        case 'H':
            printMenu();
            break;
            
        default:
            // Игнорируем неизвестные команды
            break;
    }
}

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
// ============================================================================

void loadCalibrations() {
    Serial.println("📂 Загрузка сохраненных калибровок...");
    paddleMagCalibrator.loadCalibration();
    kayakMagCalibrator.loadCalibration();
    
    if (paddleMagCalibrator.isCalibrationValid()) {
        Serial.println("   ✅ Калибровка весла загружена");
    }
    if (kayakMagCalibrator.isCalibrationValid()) {
        Serial.println("   ✅ Калибровка каяка загружена");
    }
    Serial.println();
}

void updateDisplay(float shaftRotation, float shaftTilt, float bladeRotation,
                   StrokePhase::Phase phase, int motorForce) {
    // Обновление вашего дисплея
    // Можно добавить индикацию фазы гребка, надежности магнитометра и т.д.
    
    if (smartKayak.display) {
        KayakDisplayData displayData = smartKayak.display->getCurrentDisplayData();
        displayData.shaftRotationAngle = shaftRotation;
        displayData.shaftTiltAngle = shaftTilt;
        displayData.bladeRotationAngle = bladeRotation;
        displayData.motorForce = motorForce;
        
        // Можно добавить:
        // displayData.currentPhase = phase;
        // displayData.magReliability = orientationEstimator.getMagReliability();
        
        smartKayak.display->update(displayData);
    }
}
