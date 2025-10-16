/**
 * Пример интеграции RelativeOrientationFilter в SmartKayak
 * 
 * Этот пример показывает как заменить существующий метод getRelativeOrientation
 * на новый фильтр с адаптивным доверием к магнитометру и учетом паттернов гребли
 */

#include "SmartKayak.h"

// ============================================================================
// МОДИФИЦИРОВАННЫЙ МЕТОД UPDATE С ФИЛЬТРОМ
// ============================================================================

void SmartKayak::update() {
    // 1. Проверки базовых условий
    if (!paddle->connected()) {
        motorDriver->stop();
        return; 
    }   

    // 2. Получаем данные с тензодатчиков (с сглаживанием)
    int borderLoadForce = 600;
    loadData loads = paddle->getLoadData();
    
    if (LOADCELL_SMOOTHING_FACTOR > 0) {
        currentLoadCellData.forceR = (currentLoadCellData.forceR * (1 - LOADCELL_SMOOTHING_FACTOR) + loads.forceR * LOADCELL_SMOOTHING_FACTOR);
        currentLoadCellData.forceL = (currentLoadCellData.forceL * (1 - LOADCELL_SMOOTHING_FACTOR) + loads.forceL * LOADCELL_SMOOTHING_FACTOR);
        currentLoadCellData.forceR_raw = (currentLoadCellData.forceR_raw * (1 - LOADCELL_SMOOTHING_FACTOR) + loads.forceR_raw * LOADCELL_SMOOTHING_FACTOR);
        currentLoadCellData.forceL_raw = (currentLoadCellData.forceL_raw * (1 - LOADCELL_SMOOTHING_FACTOR) + loads.forceL_raw * LOADCELL_SMOOTHING_FACTOR);
        currentLoadCellData.timestamp = loads.timestamp;
        loads = currentLoadCellData;
    } 

    // 3. Получаем ориентацию весла
    OrientationData paddleOrientation = paddle->getOrientationData();
    if (paddleOrientation.q0 == 0 && paddleOrientation.q1 == 0 && 
        paddleOrientation.q2 == 0 && paddleOrientation.q3 == 0) {
        return; 
    }

    SP_Math::Quaternion currentPaddleQ(
        paddleOrientation.q0,
        paddleOrientation.q1,
        paddleOrientation.q2,
        paddleOrientation.q3
    );

    // 4. НОВЫЙ КОД: Получаем данные IMU и магнитометров
    IMUData kayakIMUData;
    imu->getData(kayakIMUData);  // IMU каяка
    IMUData paddleIMUData = paddle->getIMUData();  // IMU весла
    
    // Формируем векторы магнитометров
    SP_Math::Vector kayakMag(
        kayakIMUData.mag_x, 
        kayakIMUData.mag_y, 
        kayakIMUData.mag_z
    );
    SP_Math::Vector paddleMag(
        paddleIMUData.mag_x, 
        paddleIMUData.mag_y, 
        paddleIMUData.mag_z
    );

    // 5. Определяем какая лопасть ниже
    BladeSideType bladeSide = getLowerBladeSide(
        currentPaddleQ, 
        paddle->getBladeAngles().YAxisDirection
    );
    currentBladeSide = bladeSide;

    if (bladeSide == BladeSideType::ALL_BLADES) {
        motorDriver->stop();
        return; 
    }

    // 6. Калибровка тензодатчиков с учетом гравитации
    loadCellCalibrator.updateTare(
        (bladeSide == BladeSideType::RIGHT_BLADE),
        loads.forceL,
        loads.forceR,
        paddle->getIMUData(),
        paddle->getBladeAngles()
    );
    
    // 7. Получаем откалиброванную силу
    float bladeForce = loadCellCalibrator.getCalibratedForce(
        (bladeSide == BladeSideType::RIGHT_BLADE),
        (bladeSide == BladeSideType::RIGHT_BLADE) ? loads.forceR : loads.forceL,
        paddle->getIMUData(),
        paddle->getBladeAngles()
    );

    // 8. Предварительный расчет углов для фильтра (грубая оценка для определения фазы)
    float prelimShaftRotation, prelimShaftTilt, prelimBladeRotation;
    SP_Math::Quaternion prelimRelativeQuat = getRelativeOrientation(currentPaddleQ, paddle);
    getPaddleAngles(prelimRelativeQuat, prelimShaftRotation, prelimShaftTilt, prelimBladeRotation);

    // ============================================================================
    // 9. КЛЮЧЕВОЙ МОМЕНТ: ИСПОЛЬЗУЕМ НОВЫЙ ФИЛЬТР!
    // ============================================================================
    SP_Math::Quaternion paddleRelativeQuat = orientationFilter.update(
        kayakOrientationQuat,   // Кватернион каяка из DMP
        currentPaddleQ,          // Кватернион весла из DMP
        kayakMag,                // Магнитометр каяка
        paddleMag,               // Магнитометр весла
        prelimShaftRotation,     // Угол для определения фазы
        bladeForce               // Сила для определения фазы
    );
    // ============================================================================

    // 10. Вычисляем финальные углы из отфильтрованного кватерниона
    float shaftRotationAngle, shaftTiltAngle, bladeRotationAngle;
    getPaddleAngles(paddleRelativeQuat, shaftRotationAngle, shaftTiltAngle, bladeRotationAngle);

    // 11. Получаем нормаль лопасти и проецируем в систему каяка
    SP_Math::Vector paddleNormal( 
        (bladeSide == BladeSideType::RIGHT_BLADE) ? 
        paddle->getBladeAngles().rightBladeVector : 
        paddle->getBladeAngles().leftBladeVector
    );
    
    SP_Math::Vector kayakPaddleCorrectedNormal = paddleRelativeQuat.rotate(paddleNormal);
    kayakPaddleCorrectedNormal.normalize();
    
    // 12. Вычисляем эффективную силу
    float cosAngle = kayakPaddleCorrectedNormal.x();
    float fForce = bladeForce * cosAngle;

    // Применяем мертвую зону
    if ((bladeForce < borderLoadForce) && (bladeForce > -borderLoadForce)) {
        fForce = 0;
    }   

    currentForceGramms = (int)fForce;

    // 13. Применяем масштаб в зависимости от режима
    int force = 0;
    
    if (modeSwitch->getMode() == MOTOR_OFF){
        force = 0;
    }
    else if (modeSwitch->getMode() == MOTOR_LOW_POWER){
        force = (int)(fForce * POWER_LOW_SCALE);
    }
    else if (modeSwitch->getMode() == MOTOR_MEDIUM_POWER){
        force = (int)(fForce * POWER_MEDIUM_SCALE);
    }
    else if (modeSwitch->getMode() == MOTOR_HIGH_POWER){
        force = (int)(fForce * POWER_HIGH_SCALE);
    }

    // 14. Обновляем дисплей с диагностической информацией
    displayData = display->getCurrentDisplayData();
    displayData.shaftRotationAngle = shaftRotationAngle;
    displayData.shaftTiltAngle = shaftTiltAngle;
    displayData.bladeRotationAngle = bladeRotationAngle;
    displayData.leftForce = loads.forceL;
    displayData.rightForce = loads.forceR;
    displayData.leftTare = loadCellCalibrator.getLeftTare();
    displayData.rightTare = loadCellCalibrator.getRightTare();
    displayData.isRightBlade = (bladeSide == BladeSideType::RIGHT_BLADE);
    displayData.leftBladeAngle = paddle->getBladeAngles().leftBladeAngle;
    displayData.rightBladeAngle = paddle->getBladeAngles().rightBladeAngle;
    displayData.motorForce = motorDriver->getForce();
    
    // НОВОЕ: Добавляем диагностику фильтра
    displayData.strokePhase = (int)orientationFilter.getCurrentPhase();
    displayData.kayakMagOK = orientationFilter.isKayakMagReliable();
    displayData.paddleMagOK = orientationFilter.isPaddleMagReliable();
    
    if (display) {
        display->update(displayData);
    }

    // 15. Финальная проверка режима и установка силы мотора
    if (modeSwitch->getMode() == MOTOR_OFF){
        if (motorDriver->getForce() != 0) {
            motorDriver->stop();
        }
        return; 
    }

    motorDriver->setForce(forceAdapter.GetAdaptedForce(force));
}

// ============================================================================
// ПРИМЕР НАСТРОЙКИ В SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    
    // ... инициализация компонентов ...
    
    SmartKayak kayak;
    
    // Настраиваем параметры фильтра
    FilterParameters filterParams;
    
    // 1. Базовые параметры доверия
    filterParams.gyroTrust = 0.98f;       // Высокое доверие DMP
    filterParams.magTrustMax = 0.02f;     // Низкое доверие магнитометру
    filterParams.magTrustMin = 0.0f;      // Минимум при помехах
    
    // 2. Настройка детекции помех магнитометра
    // ВАЖНО: Определите для вашего региона!
    // Используйте https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
    filterParams.nominalMagMagnitude = 50.0f;  // μT для вашей локации
    filterParams.magTolerancePercent = 30.0f;  // ±30% допуск
    filterParams.maxMagChangeRate = 100.0f;    // μT/s
    
    // 3. Параметры циклической коррекции
    filterParams.useStrokePhaseCorrection = true;
    filterParams.strokePhaseWeight = 0.1f;     // 10% влияние паттернов
    
    // Применяем параметры
    kayak.getOrientationFilter().setParameters(filterParams);
    
    Serial.println("✓ Фильтр относительной ориентации настроен");
    
    // ... остальной setup ...
}

// ============================================================================
// ПРИМЕР ЛОГИРОВАНИЯ ДЛЯ ОТЛАДКИ
// ============================================================================

void loop() {
    kayak.update();
    
    // Периодически выводим диагностику
    static unsigned long lastLog = 0;
    if (millis() - lastLog > 1000) {  // Каждую секунду
        lastLog = millis();
        
        Serial.println("\n=== Диагностика фильтра ориентации ===");
        
        // Статус магнитометров
        Serial.printf("Магнитометр каяка: %s\n", 
            kayak.isKayakMagnetometerReliable() ? "✓ Надежен" : "✗ Ненадежен");
        Serial.printf("Магнитометр весла: %s\n", 
            kayak.isPaddleMagnetometerReliable() ? "✓ Надежен" : "✗ Ненадежен");
        
        // Текущая фаза гребка
        StrokePhase phase = kayak.getCurrentStrokePhase();
        const char* phaseNames[] = {"CATCH", "PULL", "RELEASE", "RECOVERY", "UNKNOWN"};
        Serial.printf("Фаза гребка: %s\n", phaseNames[(int)phase]);
        
        // Коррекция дрифта
        float yawDrift = kayak.getOrientationFilter().getYawDriftCorrection();
        Serial.printf("Коррекция yaw: %.2f°\n", yawDrift * RAD_TO_DEG);
        
        Serial.println("====================================\n");
    }
}

// ============================================================================
// ЭКСПОРТ ДАННЫХ ДЛЯ АНАЛИЗА (CSV формат)
// ============================================================================

void SmartKayak::logFilterDataToSD(File& logFile) {
    // Заголовки (вызвать один раз):
    // timestamp,qw,qx,qy,qz,kayakMagX,kayakMagY,kayakMagZ,paddleMagX,paddleMagY,paddleMagZ,
    // kayakMagOK,paddleMagOK,strokePhase,yawCorrection
    
    logFile.printf("%lu,", millis());
    
    // Отфильтрованный кватернион
    SP_Math::Quaternion q = orientationFilter.getFilteredQuaternion(); // добавить геттер!
    logFile.printf("%.6f,%.6f,%.6f,%.6f,", q.w(), q.x(), q.y(), q.z());
    
    // Данные магнитометров
    IMUData kayakIMU, paddleIMU;
    imu->getData(kayakIMU);
    paddleIMU = paddle->getIMUData();
    logFile.printf("%d,%d,%d,", kayakIMU.mag_x, kayakIMU.mag_y, kayakIMU.mag_z);
    logFile.printf("%d,%d,%d,", paddleIMU.mag_x, paddleIMU.mag_y, paddleIMU.mag_z);
    
    // Статус
    logFile.printf("%d,%d,", 
        orientationFilter.isKayakMagReliable() ? 1 : 0,
        orientationFilter.isPaddleMagReliable() ? 1 : 0);
    
    // Фаза гребка
    logFile.printf("%d,", (int)orientationFilter.getCurrentPhase());
    
    // Накопленная коррекция
    logFile.printf("%.4f\n", orientationFilter.getYawDriftCorrection());
}

// ============================================================================
// РЕКОМЕНДАЦИИ ПО НАСТРОЙКЕ ДЛЯ РАЗНЫХ УСЛОВИЙ
// ============================================================================

/* 
СЦЕНАРИЙ 1: Чистая вода, хорошие условия
----------------------------------------
filterParams.gyroTrust = 0.96f;           // Можно снизить немного
filterParams.magTrustMax = 0.04f;         // Больше доверия магнитометру
filterParams.strokePhaseWeight = 0.05f;   // Меньше влияние паттернов

СЦЕНАРИЙ 2: Городские условия, мосты, металл
--------------------------------------------
filterParams.gyroTrust = 0.99f;           // Максимум доверия DMP
filterParams.magTrustMax = 0.0f;          // Отключаем магнитометр
filterParams.strokePhaseWeight = 0.15f;   // Больше влияние паттернов

СЦЕНАРИЙ 3: Соревнования (максимальная стабильность)
---------------------------------------------------
filterParams.gyroTrust = 0.985f;          // Очень высокое доверие DMP
filterParams.magTrustMax = 0.015f;        // Минимум коррекции
filterParams.strokePhaseWeight = 0.12f;   // Умеренное влияние паттернов
filterParams.useStrokePhaseCorrection = true;

СЦЕНАРИЙ 4: Тренировка, анализ техники
-------------------------------------
filterParams.gyroTrust = 0.97f;           
filterParams.magTrustMax = 0.03f;         
filterParams.strokePhaseWeight = 0.20f;   // Высокое влияние для обучения
// + Включить подробное логирование для анализа
*/
