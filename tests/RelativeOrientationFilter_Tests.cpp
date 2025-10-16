/**
 * Тесты и проверки для RelativeOrientationFilter
 * 
 * Этот файл содержит:
 * - Юнит-тесты для отдельных компонентов
 * - Интеграционные тесты
 * - Процедуры калибровки и верификации
 */

#include <Arduino.h>
#include "RelativeOrientationFilter.h"

// ============================================================================
// ТЕСТ 1: Проверка детекции магнитных помех
// ============================================================================

void test_magnetometer_interference_detection() {
    Serial.println("\n=== ТЕСТ: Детекция магнитных помех ===");
    
    RelativeOrientationFilter filter;
    FilterParameters params;
    params.nominalMagMagnitude = 50.0f;
    params.magTolerancePercent = 30.0f;
    params.maxMagChangeRate = 100.0f;
    filter.setParameters(params);
    
    // Тест 1: Нормальное магнитное поле
    {
        SP_Math::Vector normalMag(30, 30, 20);  // ~50 μT
        SP_Math::Quaternion dummyQ(1, 0, 0, 0);
        
        filter.update(dummyQ, dummyQ, normalMag, normalMag, 0, 0);
        
        bool reliable = filter.isKayakMagReliable();
        Serial.printf("Нормальное поле (50μT): %s\n", 
            reliable ? "✓ НАДЕЖНО" : "✗ НЕНАДЕЖНО");
    }
    
    // Тест 2: Аномальная величина (помеха)
    {
        SP_Math::Vector anomalyMag(100, 100, 100);  // ~173 μT - слишком много!
        SP_Math::Quaternion dummyQ(1, 0, 0, 0);
        
        filter.update(dummyQ, dummyQ, anomalyMag, anomalyMag, 0, 0);
        
        bool reliable = filter.isKayakMagReliable();
        Serial.printf("Аномальное поле (173μT): %s\n", 
            reliable ? "✗ ОШИБКА!" : "✓ КОРРЕКТНО ОТКЛОНЕНО");
    }
    
    // Тест 3: Быстрое изменение (проезжаем под мостом)
    {
        SP_Math::Vector mag1(30, 30, 20);
        SP_Math::Vector mag2(80, 80, 50);  // Резкий скачок
        SP_Math::Quaternion dummyQ(1, 0, 0, 0);
        
        filter.update(dummyQ, dummyQ, mag1, mag1, 0, 0);
        delay(100);  // 100мс
        filter.update(dummyQ, dummyQ, mag2, mag2, 0, 0);
        
        bool reliable = filter.isKayakMagReliable();
        Serial.printf("Резкое изменение поля: %s\n", 
            reliable ? "✗ ОШИБКА!" : "✓ КОРРЕКТНО ОТКЛОНЕНО");
    }
}

// ============================================================================
// ТЕСТ 2: Проверка определения фаз гребка
// ============================================================================

void test_stroke_phase_detection() {
    Serial.println("\n=== ТЕСТ: Определение фаз гребка ===");
    
    RelativeOrientationFilter filter;
    SP_Math::Quaternion dummyQ(1, 0, 0, 0);
    SP_Math::Vector dummyMag(30, 30, 20);
    
    // Тест CATCH (захват)
    {
        float shaftAngle = -25.0f;  // Наклон вниз
        float bladeForce = 500.0f;   // Начинается сила
        
        filter.update(dummyQ, dummyQ, dummyMag, dummyMag, shaftAngle, bladeForce);
        
        StrokePhase phase = filter.getCurrentPhase();
        Serial.printf("CATCH фаза (angle=-25°, force=500): %s\n",
            phase == StrokePhase::CATCH ? "✓ ВЕРНО" : "✗ ОШИБКА");
    }
    
    // Тест PULL (тяга)
    {
        float shaftAngle = 0.0f;
        float bladeForce = 800.0f;   // Высокая сила
        
        filter.update(dummyQ, dummyQ, dummyMag, dummyMag, shaftAngle, bladeForce);
        
        StrokePhase phase = filter.getCurrentPhase();
        Serial.printf("PULL фаза (force=800): %s\n",
            phase == StrokePhase::PULL ? "✓ ВЕРНО" : "✗ ОШИБКА");
    }
    
    // Тест RELEASE (выход)
    {
        float shaftAngle = 25.0f;    // Наклон вверх
        float bladeForce = 200.0f;   // Сила падает
        
        // Сначала устанавливаем PULL
        filter.update(dummyQ, dummyQ, dummyMag, dummyMag, 0, 800);
        // Потом переходим в RELEASE
        filter.update(dummyQ, dummyQ, dummyMag, dummyMag, shaftAngle, bladeForce);
        
        StrokePhase phase = filter.getCurrentPhase();
        Serial.printf("RELEASE фаза (angle=+25°, force=200): %s\n",
            phase == StrokePhase::RELEASE ? "✓ ВЕРНО" : "✗ ОШИБКА");
    }
    
    // Тест RECOVERY (проводка)
    {
        float shaftAngle = 0.0f;
        float bladeForce = 50.0f;    // Очень малая сила
        
        filter.update(dummyQ, dummyQ, dummyMag, dummyMag, shaftAngle, bladeForce);
        
        StrokePhase phase = filter.getCurrentPhase();
        Serial.printf("RECOVERY фаза (force=50): %s\n",
            phase == StrokePhase::RECOVERY ? "✓ ВЕРНО" : "✗ ОШИБКА");
    }
}

// ============================================================================
// ТЕСТ 3: Проверка коррекции yaw от магнитометра
// ============================================================================

void test_yaw_correction() {
    Serial.println("\n=== ТЕСТ: Коррекция yaw магнитометром ===");
    
    RelativeOrientationFilter filter;
    
    // Создаем ситуацию: каяк смотрит на север, весло повернуто на 90° (восток)
    // Кватернион поворота на 90° вокруг Z: [cos(45°), 0, 0, sin(45°)]
    SP_Math::Quaternion kayakQ(1, 0, 0, 0);  // Север
    SP_Math::Quaternion paddleQ(0.7071f, 0, 0, 0.7071f);  // +90° от севера
    
    // Магнитометры должны показать эту же разницу
    SP_Math::Vector kayakMag(50, 0, 0);    // Север
    SP_Math::Vector paddleMag(0, 50, 0);   // Восток
    
    // Обновляем фильтр много раз для накопления коррекции
    for (int i = 0; i < 100; i++) {
        filter.update(kayakQ, paddleQ, kayakMag, paddleMag, 0, 100);
    }
    
    float yawCorrection = filter.getYawDriftCorrection();
    Serial.printf("Накопленная коррекция yaw: %.2f° ", yawCorrection * RAD_TO_DEG);
    
    if (abs(yawCorrection) < 0.1f) {  // Должна быть близка к 0, т.к. DMP и mag согласны
        Serial.println("✓ КОРРЕКТНО");
    } else {
        Serial.println("⚠ Есть расхождение (возможно нормально)");
    }
}

// ============================================================================
// ПРОЦЕДУРА: Калибровка номинального магнитного поля
// ============================================================================

float calibrate_nominal_magnetic_field(ImuSensor* kayakIMU, int samples = 100) {
    Serial.println("\n=== КАЛИБРОВКА: Определение магнитного поля ===");
    Serial.println("Поместите систему вдали от металла и помех");
    Serial.println("Измерение через 3 секунды...");
    
    delay(3000);
    
    float sumMagnitude = 0;
    float minMag = 999999;
    float maxMag = 0;
    
    for (int i = 0; i < samples; i++) {
        IMUData data;
        kayakIMU->getData(data);
        
        float magnitude = sqrt(data.mag_x * data.mag_x + 
                              data.mag_y * data.mag_y + 
                              data.mag_z * data.mag_z);
        
        sumMagnitude += magnitude;
        minMag = min(minMag, magnitude);
        maxMag = max(maxMag, magnitude);
        
        if (i % 10 == 0) {
            Serial.printf(".");
        }
        
        delay(50);  // 50мс между измерениями
    }
    
    float avgMagnitude = sumMagnitude / samples;
    float variation = maxMag - minMag;
    
    Serial.println("\n\nРезультаты калибровки:");
    Serial.printf("Средняя величина: %.2f μT\n", avgMagnitude);
    Serial.printf("Разброс: %.2f μT (%.1f%%)\n", variation, variation/avgMagnitude*100);
    
    Serial.println("\nРекомендуемые настройки:");
    Serial.printf("params.nominalMagMagnitude = %.1ff;\n", avgMagnitude);
    
    if (variation / avgMagnitude < 0.1f) {
        Serial.println("params.magTolerancePercent = 20.0f;  // Отличные условия");
    } else if (variation / avgMagnitude < 0.3f) {
        Serial.println("params.magTolerancePercent = 30.0f;  // Хорошие условия");
    } else {
        Serial.println("params.magTolerancePercent = 50.0f;  // Условия с помехами");
    }
    
    return avgMagnitude;
}

// ============================================================================
// ПРОЦЕДУРА: Проверка качества магнитных данных в реальном времени
// ============================================================================

void monitor_magnetometer_quality(SmartKayak* kayak, int duration_seconds = 30) {
    Serial.println("\n=== МОНИТОРИНГ: Качество магнитометров ===");
    Serial.println("Формат: timestamp, kayakMag, paddleMag, kayakOK, paddleOK, phase");
    Serial.println("timestamp,kayakMag,paddleMag,kayakOK,paddleOK,phase");
    
    unsigned long startTime = millis();
    unsigned long lastPrint = 0;
    
    while (millis() - startTime < duration_seconds * 1000) {
        if (millis() - lastPrint > 100) {  // Каждые 100мс
            lastPrint = millis();
            
            RelativeOrientationFilter& filter = kayak->getOrientationFilter();
            
            // Получаем статистику
            MagnetometerStats kayakStats = filter.getKayakMagStats();
            MagnetometerStats paddleStats = filter.getPaddleMagStats();
            
            Serial.printf("%lu,%.1f,%.1f,%d,%d,%d\n",
                millis() - startTime,
                kayakStats.magnitude,
                paddleStats.magnitude,
                kayakStats.isReliable ? 1 : 0,
                paddleStats.isReliable ? 1 : 0,
                (int)filter.getCurrentPhase()
            );
        }
        
        kayak->update();  // Обновляем систему
    }
    
    Serial.println("\nМониторинг завершен. Импортируйте данные в Excel для анализа.");
}

// ============================================================================
// ПРОЦЕДУРА: Тест полного цикла гребка
// ============================================================================

void test_full_stroke_cycle(SmartKayak* kayak) {
    Serial.println("\n=== ТЕСТ: Полный цикл гребка ===");
    Serial.println("Выполните 5 гребков...\n");
    
    int strokeCount = 0;
    StrokePhase lastPhase = StrokePhase::RECOVERY;
    unsigned long phaseEnterTime[4] = {0, 0, 0, 0};
    int phaseDurations[4] = {0, 0, 0, 0};  // CATCH, PULL, RELEASE, RECOVERY
    
    while (strokeCount < 5) {
        kayak->update();
        
        StrokePhase currentPhase = kayak->getCurrentStrokePhase();
        
        // Детекция перехода фаз
        if (currentPhase != lastPhase) {
            unsigned long now = millis();
            
            // Сохраняем длительность предыдущей фазы
            if (lastPhase != StrokePhase::UNKNOWN) {
                int duration = now - phaseEnterTime[(int)lastPhase];
                phaseDurations[(int)lastPhase] += duration;
            }
            
            // Регистрируем вход в новую фазу
            phaseEnterTime[(int)currentPhase] = now;
            
            // Если перешли в CATCH - это новый гребок
            if (currentPhase == StrokePhase::CATCH) {
                strokeCount++;
                Serial.printf("Гребок %d/5 начат\n", strokeCount);
            }
            
            lastPhase = currentPhase;
        }
        
        delay(10);
    }
    
    Serial.println("\n=== Результаты анализа ===");
    Serial.printf("Средняя длительность CATCH: %d мс\n", phaseDurations[0] / strokeCount);
    Serial.printf("Средняя длительность PULL: %d мс\n", phaseDurations[1] / strokeCount);
    Serial.printf("Средняя длительность RELEASE: %d мс\n", phaseDurations[2] / strokeCount);
    Serial.printf("Средняя длительность RECOVERY: %d мс\n", phaseDurations[3] / strokeCount);
    
    int totalCycle = (phaseDurations[0] + phaseDurations[1] + 
                      phaseDurations[2] + phaseDurations[3]) / strokeCount;
    Serial.printf("Средняя длительность полного цикла: %d мс\n", totalCycle);
    Serial.printf("Частота гребков: %.1f гребков/мин\n", 60000.0f / totalCycle);
}

// ============================================================================
// ПРОЦЕДУРА: Проверка дрифта системы
// ============================================================================

void test_orientation_drift(SmartKayak* kayak, int duration_minutes = 5) {
    Serial.println("\n=== ТЕСТ: Проверка дрифта ориентации ===");
    Serial.printf("Тест будет выполняться %d минут\n", duration_minutes);
    Serial.println("Оставьте каяк и весло неподвижными\n");
    
    delay(3000);
    
    // Запоминаем начальную ориентацию
    SP_Math::Quaternion initialQuat = kayak->getOrientationFilter().getFilteredQuaternion();
    
    unsigned long startTime = millis();
    unsigned long lastReport = 0;
    float maxDrift = 0;
    
    while (millis() - startTime < duration_minutes * 60000) {
        kayak->update();
        
        if (millis() - lastReport > 10000) {  // Каждые 10 секунд
            lastReport = millis();
            
            SP_Math::Quaternion currentQuat = kayak->getOrientationFilter().getFilteredQuaternion();
            
            // Вычисляем разницу ориентаций
            SP_Math::Quaternion diffQuat = initialQuat.conjugate() * currentQuat;
            
            // Извлекаем угол поворота
            float angle = 2.0f * acos(diffQuat.w()) * RAD_TO_DEG;
            maxDrift = max(maxDrift, angle);
            
            int elapsed = (millis() - startTime) / 1000;
            Serial.printf("[%02d:%02d] Дрифт: %.2f° (макс: %.2f°)\n", 
                elapsed / 60, elapsed % 60, angle, maxDrift);
        }
    }
    
    Serial.println("\n=== Результаты теста дрифта ===");
    Serial.printf("Максимальный дрифт за %d минут: %.2f°\n", duration_minutes, maxDrift);
    Serial.printf("Дрифт в минуту: %.2f°/мин\n", maxDrift / duration_minutes);
    
    if (maxDrift / duration_minutes < 1.0f) {
        Serial.println("✓ ОТЛИЧНО: Дрифт < 1°/мин");
    } else if (maxDrift / duration_minutes < 3.0f) {
        Serial.println("✓ ХОРОШО: Дрифт < 3°/мин");
    } else {
        Serial.println("⚠ ВНИМАНИЕ: Высокий дрифт, проверьте калибровку");
    }
}

// ============================================================================
// MAIN: Запуск всех тестов
// ============================================================================

void runAllFilterTests(SmartKayak* kayak) {
    Serial.println("\n");
    Serial.println("╔════════════════════════════════════════════════════════╗");
    Serial.println("║  ТЕСТИРОВАНИЕ ФИЛЬТРА ОТНОСИТЕЛЬНОЙ ОРИЕНТАЦИИ         ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
    
    // Базовые тесты (не требуют реального оборудования)
    test_magnetometer_interference_detection();
    test_stroke_phase_detection();
    test_yaw_correction();
    
    Serial.println("\n\n╔════════════════════════════════════════════════════════╗");
    Serial.println("║  КАЛИБРОВКА И РЕАЛЬНЫЕ ТЕСТЫ                           ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
    
    // Калибровка
    float nominalMag = calibrate_nominal_magnetic_field(kayak->imu, 100);
    
    // Применяем результаты калибровки
    FilterParameters params = kayak->getOrientationFilter().getParameters();
    params.nominalMagMagnitude = nominalMag;
    kayak->getOrientationFilter().setParameters(params);
    
    // Интерактивные тесты
    Serial.println("\n1. Мониторинг качества магнитометров (30 сек)");
    monitor_magnetometer_quality(kayak, 30);
    
    Serial.println("\n2. Анализ цикла гребка");
    test_full_stroke_cycle(kayak);
    
    Serial.println("\n3. Тест дрифта (5 минут)");
    test_orientation_drift(kayak, 5);
    
    Serial.println("\n\n╔════════════════════════════════════════════════════════╗");
    Serial.println("║  ВСЕ ТЕСТЫ ЗАВЕРШЕНЫ                                   ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
}

// ============================================================================
// Пример использования в основной программе
// ============================================================================

/*
void setup() {
    Serial.begin(115200);
    
    // Инициализация системы...
    SmartKayak kayak;
    // ...
    
    // Запуск тестов
    runAllFilterTests(&kayak);
}
*/
