/**
 * Пример использования фильтра относительной ориентации
 * 
 * Демонстрирует:
 * 1. Детекцию магнитных помех
 * 2. Адаптивное использование магнитометра
 * 3. Коррекцию на основе паттернов гребли
 * 4. Диагностику и мониторинг
 */

#include <Arduino.h>
#include "SmartKayak.h"
#include "RelativeOrientationFilter.h"

SmartKayak kayak;
RelativeOrientationFilter orientationFilter;

// Тайминги
uint32_t lastUpdate = 0;
uint32_t lastDiagnostic = 0;
uint32_t lastCalibration = 0;

// Флаги
bool systemReady = false;
bool autoCalibrationEnabled = true;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("🚣 Система фильтрации ориентации весла");
    Serial.println("=====================================");
    
    // Инициализация SmartKayak
    kayak.begin();
    
    // Ждем стабилизации IMU
    Serial.println("⏳ Ожидание стабилизации IMU (3 сек)...");
    delay(3000);
    
    // Первичная калибровка
    Serial.println("🎯 Первичная калибровка магнитометра...");
    Serial.println("   Убедитесь что каяк неподвижен и нет помех!");
    delay(2000);
    
    orientationFilter.calibrateReference();
    systemReady = true;
    
    Serial.println("✅ Система готова!");
    Serial.println("\nКоманды:");
    Serial.println("  'c' - Рекалибровка магнитометра");
    Serial.println("  'd' - Показать детальную диагностику");
    Serial.println("  'r' - Сброс статистики");
    Serial.println("  'a' - Вкл/выкл автокалибровку");
    Serial.println();
}

void loop() {
    if (!systemReady) return;
    
    uint32_t currentTime = millis();
    
    // Обработка команд с Serial
    handleSerialCommands();
    
    // Основной цикл обновления (100 Hz)
    if (currentTime - lastUpdate >= 10) {
        float dt = (currentTime - lastUpdate) / 1000.0f;
        lastUpdate = currentTime;
        
        updateOrientation(dt);
    }
    
    // Диагностика каждую секунду
    if (currentTime - lastDiagnostic >= 1000) {
        lastDiagnostic = currentTime;
        printDiagnostics();
    }
    
    // Автокалибровка каждые 30 секунд (если включена и есть низкое доверие)
    if (autoCalibrationEnabled && currentTime - lastCalibration >= 30000) {
        lastCalibration = currentTime;
        attemptAutoCalibration();
    }
}

void updateOrientation(float dt) {
    // Получаем данные IMU
    OrientationData kayakOrientation;
    kayak.imu->getOrientation(kayakOrientation);
    SP_Math::Quaternion kayakQ(
        kayakOrientation.q0, kayakOrientation.q1,
        kayakOrientation.q2, kayakOrientation.q3
    );
    
    OrientationData paddleOrientation = kayak.paddle->getOrientationData();
    SP_Math::Quaternion paddleQ(
        paddleOrientation.q0, paddleOrientation.q1,
        paddleOrientation.q2, paddleOrientation.q3
    );
    
    // Получаем сырые данные IMU
    IMUData kayakIMU, paddleIMU;
    kayak.imu->getData(kayakIMU);
    paddleIMU = kayak.paddle->getIMUData();
    
    // Создаем векторы
    SP_Math::Vector kayakMag(kayakIMU.mag_x, kayakIMU.mag_y, kayakIMU.mag_z);
    SP_Math::Vector paddleMag(paddleIMU.mag_x, paddleIMU.mag_y, paddleIMU.mag_z);
    SP_Math::Vector kayakGyro(kayakIMU.gx, kayakIMU.gy, kayakIMU.gz);
    SP_Math::Vector paddleGyro(paddleIMU.gx, paddleIMU.gy, paddleIMU.gz);
    
    // ОСНОВНОЙ ВЫЗОВ ФИЛЬТРА
    SP_Math::Quaternion filteredRelative = orientationFilter.update(
        kayakQ, paddleQ,
        kayakMag, paddleMag,
        kayakGyro, paddleGyro,
        dt
    );
    
    // Обновляем состояние гребка
    loadData loads = kayak.paddle->getLoadData();
    float totalForce = abs(loads.forceL) + abs(loads.forceR);
    bool strokeActive = (totalForce > 600); // порог силы
    
    BladeSideType bladeSide = getLowerBladeSide(
        paddleQ, 
        kayak.paddle->getBladeAngles().YAxisDirection
    );
    
    float shaftRotation, shaftTilt, bladeRotation;
    getPaddleAngles(filteredRelative, shaftRotation, shaftTilt, bladeRotation);
    
    orientationFilter.updateStrokeState(
        strokeActive,
        bladeSide,
        shaftRotation,
        totalForce
    );
    
    // Используем отфильтрованную ориентацию
    kayak.paddle->setFilteredOrientation(filteredRelative);
}

void printDiagnostics() {
    float kayakTrust, paddleTrust;
    orientationFilter.getMagnetometerTrust(kayakTrust, paddleTrust);
    
    uint32_t interferenceCount, correctionCount;
    orientationFilter.getStats(interferenceCount, correctionCount);
    
    // Компактный вывод
    Serial.printf("🧭 Mag Trust: Kayak=%.2f Paddle=%.2f | ", kayakTrust, paddleTrust);
    Serial.printf("📊 Помехи=%lu Коррекций=%lu\n", interferenceCount, correctionCount);
    
    // Индикация качества
    if (kayakTrust < 0.3 || paddleTrust < 0.3) {
        Serial.println("⚠️  ВНИМАНИЕ: Низкое доверие к магнитометру - возможны помехи!");
    } else if (kayakTrust > 0.8 && paddleTrust > 0.8) {
        Serial.println("✅ Отличное качество данных магнитометра");
    }
}

void printDetailedDiagnostics() {
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║   ДЕТАЛЬНАЯ ДИАГНОСТИКА ФИЛЬТРА       ║");
    Serial.println("╚════════════════════════════════════════╝");
    
    float kayakTrust, paddleTrust;
    orientationFilter.getMagnetometerTrust(kayakTrust, paddleTrust);
    
    uint32_t interferenceCount, correctionCount;
    orientationFilter.getStats(interferenceCount, correctionCount);
    
    // Доверие к магнитометрам
    Serial.println("\n📡 МАГНИТОМЕТРЫ:");
    Serial.printf("  Каяк:  %.2f ", kayakTrust);
    printTrustBar(kayakTrust);
    Serial.printf("  Весло: %.2f ", paddleTrust);
    printTrustBar(paddleTrust);
    
    // Статистика
    Serial.println("\n📊 СТАТИСТИКА:");
    Serial.printf("  Обнаружено помех: %lu\n", interferenceCount);
    Serial.printf("  Применено коррекций: %lu\n", correctionCount);
    
    float correctionRate = correctionCount > 0 ? 
        (float)correctionCount / (float)(interferenceCount + correctionCount) * 100 : 0;
    Serial.printf("  Процент коррекций: %.1f%%\n", correctionRate);
    
    // Рекомендации
    Serial.println("\n💡 РЕКОМЕНДАЦИИ:");
    if (interferenceCount > 100) {
        Serial.println("  ⚠️  Высокий уровень помех - проверьте окружение");
        Serial.println("      • Удалитесь от металлических объектов");
        Serial.println("      • Проверьте экранирование проводов");
    }
    if (kayakTrust < 0.3 || paddleTrust < 0.3) {
        Serial.println("  ⚠️  Низкое доверие к магнитометру");
        Serial.println("      • Рассмотрите рекалибровку (команда 'c')");
        Serial.println("      • Проверьте стабильность датчиков");
    }
    if (correctionCount < 10 && interferenceCount > 50) {
        Serial.println("  ⚠️  Мало коррекций при многих помехах");
        Serial.println("      • Система может не использовать паттерны гребли");
        Serial.println("      • Проверьте детекцию гребков");
    }
    
    Serial.println("\n═══════════════════════════════════════\n");
}

void printTrustBar(float trust) {
    int bars = (int)(trust * 20);
    Serial.print("[");
    for (int i = 0; i < 20; i++) {
        if (i < bars) {
            if (trust > 0.8) Serial.print("█");
            else if (trust > 0.5) Serial.print("▓");
            else Serial.print("░");
        } else {
            Serial.print(" ");
        }
    }
    Serial.println("]");
}

void attemptAutoCalibration() {
    float kayakTrust, paddleTrust;
    orientationFilter.getMagnetometerTrust(kayakTrust, paddleTrust);
    
    // Автокалибровка только если оба датчика имеют высокое доверие
    if (kayakTrust > 0.9 && paddleTrust > 0.9) {
        Serial.println("🔄 Автокалибровка: обнаружены стабильные условия");
        orientationFilter.calibrateReference();
        Serial.println("✅ Автокалибровка завершена");
    }
}

void handleSerialCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 'c':
            case 'C':
                Serial.println("\n🎯 Ручная рекалибровка...");
                Serial.println("   Убедитесь что система стабильна!");
                delay(1000);
                orientationFilter.calibrateReference();
                Serial.println("✅ Рекалибровка завершена\n");
                break;
                
            case 'd':
            case 'D':
                printDetailedDiagnostics();
                break;
                
            case 'r':
            case 'R':
                orientationFilter.resetStats();
                Serial.println("📊 Статистика сброшена\n");
                break;
                
            case 'a':
            case 'A':
                autoCalibrationEnabled = !autoCalibrationEnabled;
                Serial.printf("🔄 Автокалибровка: %s\n\n", 
                    autoCalibrationEnabled ? "ВКЛ" : "ВЫКЛ");
                break;
                
            case '\n':
            case '\r':
                break;
                
            default:
                Serial.println("❓ Неизвестная команда");
                Serial.println("Доступные: c (калибровка), d (диагностика), r (сброс), a (автокалибровка)");
                break;
        }
    }
}

// Вспомогательные функции (если нужны)
BladeSideType getLowerBladeSide(const SP_Math::Quaternion& paddleQ, int Y_axis_sign) {
    if (Y_axis_sign == 0) {
        return BladeSideType::ALL_BLADES;
    }
    SP_Math::Vector paddleYAxis(0, Y_axis_sign, 0);
    SP_Math::Vector globalYAxis = paddleQ.rotate(paddleYAxis);
    
    if (globalYAxis.z() < 0) {
        return BladeSideType::RIGHT_BLADE;
    } else {
        return BladeSideType::LEFT_BLADE;
    }
}

void getPaddleAngles(const SP_Math::Quaternion& relativePaddleQ, 
                     float& shaftRotationAngle,
                     float& shaftTiltAngle,
                     float& bladeRotationAngle)
{
    SP_Math::Vector paddleY(0, 1, 0);
    SP_Math::Vector globalZ(0, 0, 1);
    
    SP_Math::Vector currentShaftDir = relativePaddleQ.rotate(paddleY);
    SP_Math::Vector currentZinPaddle = relativePaddleQ.conjugate().rotate(globalZ);
    
    SP_Math::Vector sideDir(0,1,0);
    float cosAngle = sideDir.dot(currentShaftDir);
    float crossZ = sideDir.x() * currentShaftDir.y() - sideDir.y() * currentShaftDir.x();
    
    shaftRotationAngle = atan2(crossZ, cosAngle) * RAD_TO_DEG;
    shaftTiltAngle = asin(currentShaftDir.z()) * RAD_TO_DEG;
    bladeRotationAngle = atan2(currentZinPaddle.x(), currentZinPaddle.z()) * RAD_TO_DEG;
}
