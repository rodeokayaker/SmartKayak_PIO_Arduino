#include "MagnetometerCalibrator.h"
#include <cmath>

MagnetometerCalibrator::MagnetometerCalibrator() :
    sampleIndex(0),
    sampleCount(0),
    isCalibrating(false),
    calibrationStartTime(0)
{
    result.valid = false;
    result.quality = 0.0f;
    result.sampleCount = 0;
    
    for (int i = 0; i < 3; i++) {
        result.offset[i] = 0.0f;
        result.scale[i] = 1.0f;
        result.rotation[i] = 0.0f;
        minVals[i] = 999999.0f;
        maxVals[i] = -999999.0f;
    }
}

void MagnetometerCalibrator::startCalibration() {
    isCalibrating = true;
    sampleIndex = 0;
    sampleCount = 0;
    calibrationStartTime = millis();
    
    for (int i = 0; i < 3; i++) {
        minVals[i] = 999999.0f;
        maxVals[i] = -999999.0f;
    }
    
    Serial.println("🧭 Магнитометр: Калибровка запущена!");
    Serial.println("   Медленно поворачивайте устройство во всех направлениях");
    Serial.println("   Делайте восьмерки и круги для лучшего покрытия");
}

void MagnetometerCalibrator::stopCalibration() {
    if (!isCalibrating) return;
    
    isCalibrating = false;
    
    if (sampleCount >= settings.minSamples) {
        result = computeCalibration();
        
        if (result.valid) {
            Serial.println("✅ Калибровка завершена успешно!");
            printCalibrationResult();
            
            if (settings.autoCalibrate) {
                saveCalibration();
            }
        } else {
            Serial.println("❌ Калибровка неудачна - недостаточно данных или плохое качество");
        }
    } else {
        Serial.printf("⚠️  Калибровка отменена - недостаточно образцов (%d/%d)\n", 
                     sampleCount, settings.minSamples);
    }
}

void MagnetometerCalibrator::addSample(const IMUData& imuData) {
    addSample(imuData.mx, imuData.my, imuData.mz);
}

void MagnetometerCalibrator::addSample(float mx, float my, float mz) {
    if (!isCalibrating) return;
    if (sampleCount >= settings.maxSamples) return;
    
    // Сохраняем образец
    samples[sampleIndex][0] = mx;
    samples[sampleIndex][1] = my;
    samples[sampleIndex][2] = mz;
    
    // Обновляем min/max для быстрой калибровки
    minVals[0] = std::min(minVals[0], mx);
    minVals[1] = std::min(minVals[1], my);
    minVals[2] = std::min(minVals[2], mz);
    
    maxVals[0] = std::max(maxVals[0], mx);
    maxVals[1] = std::max(maxVals[1], my);
    maxVals[2] = std::max(maxVals[2], mz);
    
    sampleIndex = (sampleIndex + 1) % settings.maxSamples;
    sampleCount++;
    
    // Вывод прогресса каждые 50 образцов
    if (sampleCount % 50 == 0) {
        Serial.printf("📊 Собрано образцов: %d/%d (%.1f%%)\n", 
                     sampleCount, settings.minSamples, getCalibrationProgress());
    }
}

MagnetometerCalibrator::CalibrationResult MagnetometerCalibrator::computeCalibration() {
    CalibrationResult res;
    res.valid = false;
    res.sampleCount = sampleCount;
    
    if (sampleCount < settings.minSamples) {
        Serial.println("❌ Недостаточно образцов для калибровки");
        return res;
    }
    
    // Используем простую калибровку min/max
    simpleMinMaxCalibration();
    
    // Вычисляем смещение (hard iron)
    for (int i = 0; i < 3; i++) {
        res.offset[i] = (maxVals[i] + minVals[i]) / 2.0f;
        
        // Вычисляем масштаб (soft iron - упрощенный)
        float range = maxVals[i] - minVals[i];
        if (range > 0.001f) {
            res.scale[i] = settings.expectedMagnitude / range;
        } else {
            res.scale[i] = 1.0f;
        }
        
        res.rotation[i] = 0.0f;  // Пока не используем поворот
    }
    
    // Оценка качества
    res.quality = evaluateCalibrationQuality();
    res.valid = (res.quality >= settings.qualityThreshold) && checkSampleCoverage();
    
    return res;
}

void MagnetometerCalibrator::simpleMinMaxCalibration() {
    // Уже вычислено в addSample
}

float MagnetometerCalibrator::evaluateCalibrationQuality() {
    if (sampleCount < settings.minSamples) return 0.0f;
    
    // Проверяем диапазон по каждой оси
    float rangeScores[3];
    float minExpectedRange = settings.expectedMagnitude * 1.5f;  // Минимум 1.5x от ожидаемого
    
    for (int i = 0; i < 3; i++) {
        float range = maxVals[i] - minVals[i];
        rangeScores[i] = std::min(range / minExpectedRange, 1.0f);
    }
    
    // Средний балл по осям
    float avgRangeScore = (rangeScores[0] + rangeScores[1] + rangeScores[2]) / 3.0f;
    
    // Проверяем баланс (все оси должны иметь похожий диапазон)
    float maxRange = std::max({maxVals[0] - minVals[0], 
                               maxVals[1] - minVals[1], 
                               maxVals[2] - minVals[2]});
    float minRange = std::min({maxVals[0] - minVals[0], 
                               maxVals[1] - minVals[1], 
                               maxVals[2] - minVals[2]});
    
    float balanceScore = (minRange > 0) ? (minRange / maxRange) : 0.0f;
    
    // Итоговое качество
    float quality = avgRangeScore * 0.6f + balanceScore * 0.4f;
    
    return quality;
}

bool MagnetometerCalibrator::checkSampleCoverage() {
    // Упрощенная проверка - все оси должны иметь достаточный диапазон
    for (int i = 0; i < 3; i++) {
        float range = maxVals[i] - minVals[i];
        if (range < settings.expectedMagnitude * 0.8f) {
            Serial.printf("⚠️  Недостаточный диапазон по оси %d: %.1f μT\n", i, range);
            return false;
        }
    }
    return true;
}

void MagnetometerCalibrator::applyCalibration(float& mx, float& my, float& mz) const {
    if (!result.valid) return;
    
    // Применяем hard iron коррекцию (смещение)
    mx -= result.offset[0];
    my -= result.offset[1];
    mz -= result.offset[2];
    
    // Применяем soft iron коррекцию (масштаб)
    mx *= result.scale[0];
    my *= result.scale[1];
    mz *= result.scale[2];
}

void MagnetometerCalibrator::applyCalibration(IMUData& imuData) const {
    applyCalibration(imuData.mx, imuData.my, imuData.mz);
}

float MagnetometerCalibrator::getCalibrationProgress() const {
    if (!isCalibrating) return 0.0f;
    return (float)sampleCount / settings.minSamples * 100.0f;
}

void MagnetometerCalibrator::printCalibrationResult(Stream* stream) const {
    stream->println("\n╔══════════════════════════════════════════╗");
    stream->println("║   РЕЗУЛЬТАТЫ КАЛИБРОВКИ МАГНИТОМЕТРА     ║");
    stream->println("╚══════════════════════════════════════════╝");
    
    stream->printf("Статус: %s\n", result.valid ? "✅ ВАЛИДНА" : "❌ НЕВАЛИДНА");
    stream->printf("Качество: %.1f%% (порог: %.1f%%)\n", 
                   result.quality * 100, settings.qualityThreshold * 100);
    stream->printf("Образцов: %d\n\n", result.sampleCount);
    
    stream->println("Hard Iron смещение (μT):");
    stream->printf("  X: %7.2f\n", result.offset[0]);
    stream->printf("  Y: %7.2f\n", result.offset[1]);
    stream->printf("  Z: %7.2f\n", result.offset[2]);
    
    stream->println("\nSoft Iron масштаб:");
    stream->printf("  X: %7.4f\n", result.scale[0]);
    stream->printf("  Y: %7.4f\n", result.scale[1]);
    stream->printf("  Z: %7.4f\n", result.scale[2]);
    
    stream->println("\nДиапазоны измерений:");
    stream->printf("  X: [%7.2f, %7.2f] = %7.2f μT\n", 
                   minVals[0], maxVals[0], maxVals[0] - minVals[0]);
    stream->printf("  Y: [%7.2f, %7.2f] = %7.2f μT\n", 
                   minVals[1], maxVals[1], maxVals[1] - minVals[1]);
    stream->printf("  Z: [%7.2f, %7.2f] = %7.2f μT\n", 
                   minVals[2], maxVals[2], maxVals[2] - minVals[2]);
    
    stream->println("══════════════════════════════════════════\n");
}

void MagnetometerCalibrator::saveCalibration() {
    // TODO: Реализовать сохранение в EEPROM или файл
    Serial.println("💾 Сохранение калибровки... (не реализовано)");
}

void MagnetometerCalibrator::loadCalibration() {
    // TODO: Реализовать загрузку из EEPROM или файла
    Serial.println("📂 Загрузка калибровки... (не реализовано)");
}

// ============================================================================
// MagneticInterferenceDetector
// ============================================================================

MagneticInterferenceDetector::MagneticInterferenceDetector() :
    historyIndex(0),
    historyCount(0),
    baselineMagnitude(50.0f),
    baselineEstablished(false)
{
    for (int i = 0; i < WINDOW_SIZE; i++) {
        magHistory[i][0] = 0;
        magHistory[i][1] = 0;
        magHistory[i][2] = 0;
    }
}

MagneticInterferenceDetector::InterferenceReport 
MagneticInterferenceDetector::analyze(const IMUData& imuData) {
    InterferenceReport report;
    report.type = NONE;
    report.severity = 0.0f;
    report.reliability = 0.0f;
    report.actionRequired = false;
    
    // Добавляем в историю
    magHistory[historyIndex][0] = imuData.mx;
    magHistory[historyIndex][1] = imuData.my;
    magHistory[historyIndex][2] = imuData.mz;
    historyIndex = (historyIndex + 1) % WINDOW_SIZE;
    if (historyCount < WINDOW_SIZE) historyCount++;
    
    if (historyCount < 5) {
        report.description = "Накопление данных...";
        return report;
    }
    
    // Вычисляем статистику
    float mean[3], stdDev[3], magnitude;
    calculateStatistics(mean, stdDev, magnitude);
    
    // Устанавливаем baseline при первом запуске
    if (!baselineEstablished) {
        baselineMagnitude = magnitude;
        baselineEstablished = true;
    }
    
    float deviation = std::abs(magnitude - baselineMagnitude);
    float relativeDeviation = deviation / baselineMagnitude;
    
    // 1. Детекция Hard Iron (постоянное смещение)
    if (relativeDeviation > 0.3f) {  // 30% отклонение
        report.type = HARD_IRON;
        report.severity = std::min(relativeDeviation, 1.0f);
        report.reliability = 0.8f;
        report.actionRequired = true;
        report.description = "Обнаружено постоянное смещение (hard iron). Требуется калибровка!";
        return report;
    }
    
    // 2. Детекция Soft Iron (неравномерное искажение по осям)
    float avgStdDev = (stdDev[0] + stdDev[1] + stdDev[2]) / 3.0f;
    float maxStdDev = std::max({stdDev[0], stdDev[1], stdDev[2]});
    float minStdDev = std::min({stdDev[0], stdDev[1], stdDev[2]});
    
    if (maxStdDev > 0 && (maxStdDev / (minStdDev + 0.001f)) > 3.0f) {
        report.type = SOFT_IRON;
        report.severity = 0.6f;
        report.reliability = 0.7f;
        report.actionRequired = true;
        report.description = "Обнаружено искажение поля (soft iron). Рекомендуется калибровка.";
        return report;
    }
    
    // 3. Детекция динамических помех
    if (avgStdDev > 8.0f) {  // Высокая вариация
        report.type = DYNAMIC;
        report.severity = std::min(avgStdDev / 15.0f, 1.0f);
        report.reliability = 0.6f;
        report.actionRequired = false;
        report.description = "Динамические помехи. Снижена надежность магнитометра.";
        return report;
    }
    
    // 4. Критические помехи
    if (relativeDeviation > 0.7f || avgStdDev > 20.0f) {
        report.type = SEVERE;
        report.severity = 1.0f;
        report.reliability = 0.9f;
        report.actionRequired = true;
        report.description = "КРИТИЧЕСКИЕ ПОМЕХИ! Магнитометр ненадежен!";
        return report;
    }
    
    // Все хорошо
    report.type = NONE;
    report.severity = 0.0f;
    report.reliability = 1.0f - (relativeDeviation + avgStdDev / 20.0f);
    report.description = "Магнитное поле в норме";
    
    return report;
}

void MagneticInterferenceDetector::setBaseline(const IMUData& imuData) {
    baselineMagnitude = calculateMagnitude(imuData.mx, imuData.my, imuData.mz);
    baselineEstablished = true;
    Serial.printf("📍 Базовая линия установлена: %.2f μT\n", baselineMagnitude);
}

void MagneticInterferenceDetector::resetBaseline() {
    baselineEstablished = false;
    Serial.println("🔄 Базовая линия сброшена");
}

void MagneticInterferenceDetector::calculateStatistics(
    float mean[3], float stdDev[3], float& magnitude) {
    
    // Среднее
    mean[0] = mean[1] = mean[2] = 0;
    for (int i = 0; i < historyCount; i++) {
        mean[0] += magHistory[i][0];
        mean[1] += magHistory[i][1];
        mean[2] += magHistory[i][2];
    }
    mean[0] /= historyCount;
    mean[1] /= historyCount;
    mean[2] /= historyCount;
    
    // Стандартное отклонение
    stdDev[0] = stdDev[1] = stdDev[2] = 0;
    for (int i = 0; i < historyCount; i++) {
        stdDev[0] += (magHistory[i][0] - mean[0]) * (magHistory[i][0] - mean[0]);
        stdDev[1] += (magHistory[i][1] - mean[1]) * (magHistory[i][1] - mean[1]);
        stdDev[2] += (magHistory[i][2] - mean[2]) * (magHistory[i][2] - mean[2]);
    }
    stdDev[0] = std::sqrt(stdDev[0] / historyCount);
    stdDev[1] = std::sqrt(stdDev[1] / historyCount);
    stdDev[2] = std::sqrt(stdDev[2] / historyCount);
    
    // Магнитуда
    magnitude = std::sqrt(mean[0]*mean[0] + mean[1]*mean[1] + mean[2]*mean[2]);
}

float MagneticInterferenceDetector::calculateMagnitude(float mx, float my, float mz) {
    return std::sqrt(mx*mx + my*my + mz*mz);
}

void MagneticInterferenceDetector::printReport(
    const InterferenceReport& report, Stream* stream) {
    
    stream->println("\n╔═══════════════════════════════════════╗");
    stream->println("║   АНАЛИЗ МАГНИТНЫХ ПОМЕХ              ║");
    stream->println("╚═══════════════════════════════════════╝");
    
    stream->print("Тип: ");
    switch(report.type) {
        case NONE: stream->println("✅ НЕТ ПОМЕХ"); break;
        case HARD_IRON: stream->println("🧲 HARD IRON"); break;
        case SOFT_IRON: stream->println("🔧 SOFT IRON"); break;
        case DYNAMIC: stream->println("⚡ ДИНАМИЧЕСКИЕ"); break;
        case SEVERE: stream->println("🚨 КРИТИЧЕСКИЕ"); break;
    }
    
    stream->printf("Серьезность: %.1f%%\n", report.severity * 100);
    stream->printf("Надежность: %.1f%%\n", report.reliability * 100);
    stream->printf("Действие: %s\n", report.actionRequired ? "⚠️  ТРЕБУЕТСЯ" : "✓ НЕ ТРЕБУЕТСЯ");
    stream->printf("Описание: %s\n", report.description.c_str());
    stream->println("═══════════════════════════════════════\n");
}

