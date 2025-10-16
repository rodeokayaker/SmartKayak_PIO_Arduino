/**
 * Тестовый скетч для проверки системы адаптивной ориентации весла
 * 
 * Выводит в Serial:
 * - Качество магнитометра
 * - Фазу гребка
 * - Углы ориентации
 * - Статистику обучения
 * - Веса фильтра
 */

#include <Arduino.h>
#include "PaddleOrientationFusion.h"
#include "SmartKayak.h"
#include "SmartPaddle.h"

// Глобальные объекты
SmartKayak kayak;
PaddleOrientationFusion orientationFusion;

// Для мониторинга
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 100; // 100мс = 10 Гц

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("\n=== Тест системы ориентации весла ===\n");
    
    // Инициализация каяка и весла
    kayak.begin();
    
    // Настройка fusion системы
    orientationFusion.setFilterWeights(
        0.97f,  // DMP вес
        0.03f,  // Магнитометр вес
        0.0f    // Паттерн вес (вырастет после обучения)
    );
    
    orientationFusion.setMagnetometerThresholds(
        500.0f,  // Порог дисперсии
        100.0f   // Порог отклонения
    );
    
    Serial.println("✓ Инициализация завершена");
    Serial.println("✓ Начинайте грести для обучения паттернам...\n");
    
    // Заголовок таблицы
    Serial.println("Time\tPhase\t\tMagQ\t\tShaftRot\tShaftTilt\tBladeRot\tStrokes\tLearned\tDrift");
    Serial.println("----\t-----\t\t----\t\t--------\t---------\t--------\t-------\t-------\t-----");
}

void loop() {
    // Обновление основной логики каяка
    kayak.update();
    
    // Получение данных
    IMUData kayakIMU, paddleIMU;
    OrientationData kayakOrientation, paddleOrientation;
    
    // Получаем данные каяка
    kayak.imu->getData(kayakIMU);
    kayakOrientation.q0 = kayak.kayakOrientationQuat[0];
    kayakOrientation.q1 = kayak.kayakOrientationQuat[1];
    kayakOrientation.q2 = kayak.kayakOrientationQuat[2];
    kayakOrientation.q3 = kayak.kayakOrientationQuat[3];
    
    // Получаем данные весла
    if (kayak.paddle && kayak.paddle->connected()) {
        paddleIMU = kayak.paddle->getIMUData();
        paddleOrientation = kayak.paddle->getOrientationData();
        
        // Текущая сила для определения фазы
        loadData loads = kayak.paddle->getLoadData();
        float bladeForce = (loads.forceL + loads.forceR) / 2.0f;
        
        // Обновление fusion системы
        SP_Math::Quaternion relQuat = orientationFusion.update(
            kayakIMU,
            paddleIMU,
            kayakOrientation,
            paddleOrientation,
            bladeForce
        );
        
        // Вычисление углов
        float shaftRotation, shaftTilt, bladeRotation;
        getPaddleAngles(relQuat, shaftRotation, shaftTilt, bladeRotation);
        
        // Вывод данных каждые 100мс
        unsigned long currentTime = millis();
        if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
            lastPrintTime = currentTime;
            
            // Получение статистики
            int strokeCount;
            bool patternsLearned;
            float driftRate;
            orientationFusion.getStats(strokeCount, patternsLearned, driftRate);
            
            // Качество магнитометра
            MagnetometerQuality magQ = orientationFusion.getCurrentMagQuality(
                kayakIMU, paddleIMU
            );
            
            // Фаза гребка
            StrokePhase phase = orientationFusion.getStrokePhase();
            
            // Форматированный вывод
            printData(
                currentTime,
                phase,
                magQ,
                shaftRotation,
                shaftTilt,
                bladeRotation,
                strokeCount,
                patternsLearned,
                driftRate
            );
            
            // Дополнительная диагностика при изменении фазы
            static StrokePhase lastPhase = StrokePhase::UNKNOWN;
            if (phase != lastPhase) {
                printPhaseChange(lastPhase, phase);
                lastPhase = phase;
            }
            
            // Предупреждение о плохом качестве магнитометра
            if (magQ == MagnetometerQuality::POOR || magQ == MagnetometerQuality::INVALID) {
                Serial.println("⚠️  ВНИМАНИЕ: Плохое качество магнитометра!");
            }
        }
    } else {
        if (millis() - lastPrintTime >= 1000) {
            Serial.println("⏳ Ожидание подключения весла...");
            lastPrintTime = millis();
        }
    }
    
    delay(1); // Малая задержка для стабильности
}

// Функция вывода данных
void printData(
    unsigned long time,
    StrokePhase phase,
    MagnetometerQuality magQ,
    float shaftRot,
    float shaftTilt,
    float bladeRot,
    int strokes,
    bool learned,
    float drift
) {
    // Время в секундах
    Serial.print(time / 1000.0f, 1);
    Serial.print("\t");
    
    // Фаза гребка
    printPhaseShort(phase);
    Serial.print("\t\t");
    
    // Качество магнитометра
    printMagQualityShort(magQ);
    Serial.print("\t\t");
    
    // Углы
    Serial.print(shaftRot, 1);
    Serial.print("\t\t");
    Serial.print(shaftTilt, 1);
    Serial.print("\t\t");
    Serial.print(bladeRot, 1);
    Serial.print("\t\t");
    
    // Статистика
    Serial.print(strokes);
    Serial.print("\t");
    Serial.print(learned ? "ДА" : "НЕТ");
    Serial.print("\t");
    Serial.print(drift, 3);
    
    Serial.println();
}

// Вывод короткого названия фазы
void printPhaseShort(StrokePhase phase) {
    switch (phase) {
        case StrokePhase::RECOVERY:  Serial.print("ВОЗВРАТ "); break;
        case StrokePhase::CATCH:     Serial.print("ЗАХВАТ  "); break;
        case StrokePhase::DRIVE:     Serial.print("ГРЕБОК  "); break;
        case StrokePhase::RELEASE:   Serial.print("ВЫХОД   "); break;
        case StrokePhase::UNKNOWN:   Serial.print("?       "); break;
    }
}

// Вывод короткого названия качества
void printMagQualityShort(MagnetometerQuality magQ) {
    switch (magQ) {
        case MagnetometerQuality::EXCELLENT: Serial.print("ОТЛИЧНО"); break;
        case MagnetometerQuality::GOOD:      Serial.print("ХОРОШО "); break;
        case MagnetometerQuality::POOR:      Serial.print("ПЛОХО  "); break;
        case MagnetometerQuality::INVALID:   Serial.print("НЕВЕРНО"); break;
    }
}

// Вывод при смене фазы
void printPhaseChange(StrokePhase from, StrokePhase to) {
    Serial.print("🔄 Фаза: ");
    printPhaseShort(from);
    Serial.print(" → ");
    printPhaseShort(to);
    Serial.println();
}

// Команды из Serial для управления
void serialEvent() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 'r': // Reset - сброс паттернов
                orientationFusion.resetPatterns();
                Serial.println("\n✓ Паттерны сброшены. Начинайте обучение заново.\n");
                break;
                
            case '1': // Режим: чистая вода (больше доверия магнитометру)
                orientationFusion.setFilterWeights(0.95f, 0.05f, 0.0f);
                orientationFusion.setMagnetometerThresholds(800.0f, 150.0f);
                Serial.println("\n✓ Режим: Чистая вода (Mag 5%)\n");
                break;
                
            case '2': // Режим: нормальный
                orientationFusion.setFilterWeights(0.97f, 0.03f, 0.0f);
                orientationFusion.setMagnetometerThresholds(500.0f, 100.0f);
                Serial.println("\n✓ Режим: Нормальный (Mag 3%)\n");
                break;
                
            case '3': // Режим: сильные помехи (меньше доверия магнитометру)
                orientationFusion.setFilterWeights(0.98f, 0.02f, 0.0f);
                orientationFusion.setMagnetometerThresholds(300.0f, 50.0f);
                Serial.println("\n✓ Режим: Помехи (Mag 2%)\n");
                break;
                
            case 'h': // Help
                printHelp();
                break;
        }
    }
}

void printHelp() {
    Serial.println("\n=== Команды управления ===");
    Serial.println("r - Сброс паттернов");
    Serial.println("1 - Режим: Чистая вода (Mag вес 5%)");
    Serial.println("2 - Режим: Нормальный (Mag вес 3%)");
    Serial.println("3 - Режим: Помехи (Mag вес 2%)");
    Serial.println("h - Эта справка");
    Serial.println("========================\n");
}
