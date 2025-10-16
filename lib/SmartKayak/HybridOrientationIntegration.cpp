// Пример интеграции HybridOrientationEstimator в SmartKayak.cpp
// Добавьте этот код в ваш SmartKayak.cpp

/*
=============================================================================
ИНТЕГРАЦИЯ ГИБРИДНОГО ОЦЕНЩИКА ОРИЕНТАЦИИ
=============================================================================

1. В SmartKayak.h добавьте:
    #include "HybridOrientationEstimator.h"
    
    В private секцию класса SmartKayak:
    HybridOrientationEstimator orientationEstimator;

2. В конструкторе SmartKayak настройте параметры:
*/

void setupHybridOrientationEstimator() {
    // Настройка детектора магнитных аномалий
    MagneticAnomalyDetector detector;
    detector.varianceThreshold = 30.0f;      // μT² - для каяка на воде
    detector.magnitudeMin = 25.0f;           // μT - минимальное поле
    detector.magnitudeMax = 65.0f;           // μT - максимальное поле
    detector.windowSize = 15;                // Размер окна анализа
    detector.smoothingFactor = 0.85f;        // Агрессивное сглаживание
    
    // orientationEstimator.setAnomalyDetector(detector);
    
    // Настройка весов фильтра
    // gyro, accel, mag, pattern
    // orientationEstimator.setFilterWeights(0.98f, 0.02f, 0.3f, 0.2f);
}

/*
3. В методе SmartKayak::update() ЗАМЕНИТЕ вызов getRelativeOrientation на:
*/

void SmartKayak_update_EXAMPLE() {
    // ... существующий код ...
    
    OrientationData paddleOrientation = paddle->getOrientationData();
    if (paddleOrientation.q0 == 0 && paddleOrientation.q1 == 0 && 
        paddleOrientation.q2 == 0 && paddleOrientation.q3 == 0) {
        return; 
    }

    // НОВЫЙ КОД: Используем гибридный оценщик
    IMUData paddleIMU = paddle->getIMUData();
    IMUData kayakIMU;
    imu->getData(kayakIMU);
    
    OrientationData kayakOrientation;
    kayakOrientation.q0 = kayakOrientationQuat[0];
    kayakOrientation.q1 = kayakOrientationQuat[1];
    kayakOrientation.q2 = kayakOrientationQuat[2];
    kayakOrientation.q3 = kayakOrientationQuat[3];
    
    // ГЛАВНОЕ УЛУЧШЕНИЕ: Получаем высокоточную относительную ориентацию
    SP_Math::Quaternion paddleRelativeQuat = orientationEstimator.updateRelativeOrientation(
        paddleIMU,           // IMU весла
        paddleOrientation,   // DMP кватернион весла
        kayakIMU,           // IMU каяка
        kayakOrientation    // DMP кватернион каяка
    );
    
    // Определяем фазу гребка для дополнительной логики
    float shaftRotationAngle, shaftTiltAngle, bladeRotationAngle;
    getPaddleAngles(paddleRelativeQuat, shaftRotationAngle, shaftTiltAngle, bladeRotationAngle);
    
    // Детекция фазы с учетом силы
    float bladeForce = 0; // Получите из вашей логики
    StrokePhase::Phase currentPhase = orientationEstimator.detectStrokePhase(
        shaftRotationAngle,
        shaftTiltAngle,
        bladeForce
    );
    
    // Диагностика (опционально)
    static unsigned long lastDiagnostic = 0;
    if (millis() - lastDiagnostic > 5000) {
        orientationEstimator.printDiagnostics(&Serial);
        Serial.printf("Current Phase: ");
        switch(currentPhase) {
            case StrokePhase::CATCH: Serial.println("CATCH"); break;
            case StrokePhase::PULL: Serial.println("PULL"); break;
            case StrokePhase::RELEASE: Serial.println("RELEASE"); break;
            case StrokePhase::RECOVERY: Serial.println("RECOVERY"); break;
            default: Serial.println("UNKNOWN"); break;
        }
        lastDiagnostic = millis();
    }
    
    // ... продолжение существующего кода с использованием paddleRelativeQuat ...
}

/*
=============================================================================
ПРЕИМУЩЕСТВА НОВОГО ПОДХОДА:
=============================================================================

1. АДАПТИВНОЕ ИСПОЛЬЗОВАНИЕ МАГНИТОМЕТРА:
   - Автоматически детектирует магнитные помехи
   - Использует магнитометр только когда данные надежны
   - Плавное изменение весов в зависимости от качества данных

2. КОРРЕКЦИЯ ДРИФТА ПО ПАТТЕРНАМ ГРЕБЛИ:
   - Накапливает историю гребков
   - Распознает фазы: захват, проводка, выход, возврат
   - Корректирует yaw дрифт на основе типичных углов для каждой фазы
   - Валидирует логичность переходов между фазами

3. ДЕТЕКЦИЯ МАГНИТНЫХ ПОМЕХ:
   - Проверка магнитуды поля (должна быть ~50 μT)
   - Анализ вариации (стабильности) поля
   - Проверка допустимого диапазона значений
   - Сглаживание оценки надежности

4. ГИБРИДНАЯ ФУЗИЯ:
   - DMP (гиро + акселерометр) - основа
   - Магнитометр - коррекция yaw когда надежен
   - Паттерны гребли - коррекция при низкой надежности магнитометра
   - Плавное смешивание источников данных

=============================================================================
НАСТРОЙКА ПОД ВАШИ УСЛОВИЯ:
=============================================================================

Параметры для калибровки:

1. magnitudeMin/Max - зависит от географического положения:
   - Экватор: ~25-30 μT
   - Средние широты: ~45-55 μT  
   - Полюса: ~60-70 μT

2. varianceThreshold - зависит от условий:
   - Спокойная вода: 20-30 μT²
   - Волны: 40-60 μT²
   - Сильные помехи: увеличьте до 80-100 μT²

3. Веса фильтра:
   - magWeight: 0.1-0.4 (больше = сильнее влияние магнитометра)
   - patternWeight: 0.1-0.3 (больше = сильнее коррекция по паттернам)

4. Размер окна анализа (windowSize):
   - Быстрая гребля: 10-15 сэмплов
   - Медленная: 20-30 сэмплов

=============================================================================
ДИАГНОСТИКА ПРОБЛЕМ:
=============================================================================

Если точность все еще плохая:

1. Проверьте Mag Reliability:
   - < 0.3: Сильные помехи, полагаемся на DMP и паттерны
   - 0.3-0.7: Умеренная надежность, частичная коррекция
   - > 0.7: Хорошие данные, полная коррекция yaw

2. Проверьте фазы гребка:
   - Если фазы не детектируются (UNKNOWN) - проверьте пороги углов
   - Если неправильные переходы - проверьте логику в detectStrokePhase

3. Калибровка магнитометра:
   - Соберите данные в чистом месте (вдали от металла)
   - Постройте 3D график векторов магнитного поля
   - Должна получиться сфера с центром в (0,0,0)
   - Если эллипсоид - нужна soft iron коррекция
   - Если смещен от нуля - нужна hard iron коррекция

4. Логирование для анализа:
*/

void logOrientationDebug(HybridOrientationEstimator& estimator) {
    Serial.printf("MagRel:%.2f,Phase:%d,PhaseConf:%.2f,",
        estimator.getMagReliability(),
        (int)estimator.getCurrentPhase(),
        estimator.getPhaseConfidence()
    );
}

/*
=============================================================================
ДОПОЛНИТЕЛЬНЫЕ УЛУЧШЕНИЯ:
=============================================================================

1. РАСШИРЕННАЯ КАЛИБРОВКА МАГНИТОМЕТРА:
   - Добавьте метод calibrateMagnetometer() для автоматической калибровки
   - Используйте алгоритм TWOSTEP или ellipsoid fitting
   
2. МАШИННОЕ ОБУЧЕНИЕ ДЛЯ ПАТТЕРНОВ:
   - Накапливайте данные успешных гребков
   - Обучите простую модель (напр., k-means) для классификации фаз
   - Предсказывайте следующее положение весла

3. КОМПЛЕМЕНТАРНЫЙ ФИЛЬТР С КАЛМАНОМ:
   - Для еще большей точности замените простую фузию на Extended Kalman Filter
   - Позволит учитывать динамику системы и шум датчиков

4. ДЕТЕКЦИЯ ВЫБРОСОВ:
   - Добавьте RANSAC для фильтрации выбросов в данных
   - Игнорируйте явно ошибочные измерения

=============================================================================
*/

