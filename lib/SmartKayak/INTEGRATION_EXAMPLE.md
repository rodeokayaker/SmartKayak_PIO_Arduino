# Пример интеграции StrokePredictorGrid в SmartKayak

## 1. Добавление в SmartKayak.h

```cpp
#ifndef SMARTKAYAK_H
#define SMARTKAYAK_H

// ... существующие include ...
#include "StrokePredictorGrid.h"  // ← ДОБАВИТЬ

class SmartKayak {
private:
    // ... существующие поля ...
    
    // ← ДОБАВИТЬ
    StrokePredictorGrid* strokePredictor;
    bool usePredictorGrid;  // Флаг использования нового предиктора
    
public:
    SmartKayak();
    ~SmartKayak();
    
    // ... существующие методы ...
    
    // ← ДОБАВИТЬ
    void enableGridPredictor(bool enable);
    void configureGridPredictor(float gridStep = 10.0f, 
                               float alphaEMA = 0.2f,
                               float threshold = 400.0f);
    void printPredictorStatistics();
};

#endif
```

## 2. Модификация SmartKayak.cpp

### Конструктор
```cpp
SmartKayak::SmartKayak():
    paddle(nullptr),
    motorDriver(nullptr),
    // ... остальные инициализации ...
    strokePredictor(nullptr),      // ← ДОБАВИТЬ
    usePredictorGrid(false)        // ← ДОБАВИТЬ
{
    // ... существующий код ...
    
    // ← ДОБАВИТЬ: Создаем предиктор
    strokePredictor = new StrokePredictorGrid();
    
    // Настройка по умолчанию
    strokePredictor->config.gridStep = 10.0f;
    strokePredictor->config.alphaEMA = 0.2f;
    strokePredictor->config.forceThreshold = 400.0f;
    strokePredictor->config.predictionTime = 200.0f;
    strokePredictor->learningMethod = LearningMethod::METHOD_B;
}
```

### Деструктор
```cpp
SmartKayak::~SmartKayak() {
    // ... существующий код ...
    
    // ← ДОБАВИТЬ
    if (strokePredictor != nullptr) {
        delete strokePredictor;
        strokePredictor = nullptr;
    }
}
```

### Модификация update()
```cpp
void SmartKayak::update(const OrientationData& paddleOrientation) {
    // ... существующий код до расчета силы ...
    
    // Получаем данные IMU весла
    IMUData imuData = paddle->getIMUData();
    SP_Math::Vector angularVelocity(
        imuData.gyroX,
        imuData.gyroY,
        imuData.gyroZ
    );
    
    SP_Math::Quaternion currentPaddleQ(
        paddleOrientation.q0,
        paddleOrientation.q1,
        paddleOrientation.q2,
        paddleOrientation.q3
    );
    
    // ← ДОБАВИТЬ: Обучение предиктора
    if (usePredictorGrid && strokePredictor != nullptr) {
        bool isForward = (modeSwitch->getMode() != MOTOR_OFF);
        strokePredictor->learn(
            currentPaddleQ,
            kayakOrientationQuat,
            angularVelocity,
            bladeForce,
            isForward
        );
    }
    
    // ... существующий расчет силы ...
    
    int force = 0;
    
    // ← МОДИФИЦИРОВАТЬ: Используем предиктор если включен
    if (usePredictorGrid && strokePredictor != nullptr) {
        bool isForward = (modeSwitch->getMode() != MOTOR_OFF);
        float confidence = 0.0f;
        
        float predictedForce = strokePredictor->predict(
            currentPaddleQ,
            kayakOrientationQuat,
            angularVelocity,
            strokePredictor->config.predictionTime,
            isForward,
            true,  // use acceleration
            &confidence
        );
        
        // Применяем предсказанную силу если уверенность достаточная
        if (confidence > 0.3f) {  // Порог уверенности 30%
            force = (int)(predictedForce * confidence);
            
            // Применяем масштабирование по режиму мотора
            if (modeSwitch->getMode() == MOTOR_LOW_POWER) {
                force = (int)(force * POWER_LOW_SCALE);
            } else if (modeSwitch->getMode() == MOTOR_MEDIUM_POWER) {
                force = (int)(force * POWER_MEDIUM_SCALE);
            } else if (modeSwitch->getMode() == MOTOR_HIGH_POWER) {
                force = (int)(force * POWER_HIGH_SCALE);
            }
        }
    } else {
        // ← Существующий код расчета силы
        if (modeSwitch->getMode() == MOTOR_OFF) {
            force = 0;
        }
        if (modeSwitch->getMode() == MOTOR_LOW_POWER) {
            force = (int)(fForce * POWER_LOW_SCALE);
        }
        if (modeSwitch->getMode() == MOTOR_MEDIUM_POWER) {
            force = (int)(fForce * POWER_MEDIUM_SCALE);
        }
        if (modeSwitch->getMode() == MOTOR_HIGH_POWER) {
            force = (int)(fForce * POWER_HIGH_SCALE);
        }
    }
    
    // ... остальной код ...
}
```

### Новые методы
```cpp
void SmartKayak::enableGridPredictor(bool enable) {
    usePredictorGrid = enable;
    
    if (enable) {
        Serial.println("Grid Predictor ENABLED");
        if (strokePredictor != nullptr) {
            strokePredictor->printGridStatistics();
        }
    } else {
        Serial.println("Grid Predictor DISABLED");
    }
}

void SmartKayak::configureGridPredictor(
    float gridStep,
    float alphaEMA,
    float threshold
) {
    if (strokePredictor != nullptr) {
        strokePredictor->config.gridStep = gridStep;
        strokePredictor->config.alphaEMA = alphaEMA;
        strokePredictor->config.forceThreshold = threshold;
        
        Serial.println("Grid Predictor configured:");
        Serial.printf("  Grid step: %.1f°\n", gridStep);
        Serial.printf("  Alpha EMA: %.2f\n", alphaEMA);
        Serial.printf("  Threshold: %.1f\n", threshold);
    }
}

void SmartKayak::printPredictorStatistics() {
    if (strokePredictor != nullptr) {
        strokePredictor->printGridStatistics();
    }
}
```

## 3. Использование в основной программе

### В setup()
```cpp
void setup() {
    Serial.begin(115200);
    
    // ... инициализация SmartKayak ...
    smartKayak = new SmartKayak();
    
    // Настройка предиктора (опционально)
    smartKayak->configureGridPredictor(
        10.0f,   // gridStep
        0.2f,    // alphaEMA
        400.0f   // threshold
    );
    
    // Включение предиктора
    smartKayak->enableGridPredictor(true);
    
    Serial.println("SmartKayak initialized with Grid Predictor!");
}
```

### В loop()
```cpp
void loop() {
    // Обычная работа - update вызывается автоматически
    
    // Периодически выводим статистику (каждые 10 секунд)
    static unsigned long lastStatTime = 0;
    if (millis() - lastStatTime > 10000) {
        lastStatTime = millis();
        smartKayak->printPredictorStatistics();
    }
}
```

## 4. Управление через Serial команды

Можно добавить команды для управления предиктором:

```cpp
void processSerialCommand() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 'p':  // Enable predictor
                smartKayak->enableGridPredictor(true);
                break;
                
            case 'P':  // Disable predictor
                smartKayak->enableGridPredictor(false);
                break;
                
            case 's':  // Statistics
                smartKayak->printPredictorStatistics();
                break;
                
            case 'c':  // Clear grid
                if (smartKayak->strokePredictor != nullptr) {
                    smartKayak->strokePredictor->clearGrid();
                }
                break;
                
            case 'a':  // Switch to Method A
                if (smartKayak->strokePredictor != nullptr) {
                    smartKayak->strokePredictor->learningMethod = LearningMethod::METHOD_A;
                    Serial.println("Switched to Learning Method A");
                }
                break;
                
            case 'b':  // Switch to Method B
                if (smartKayak->strokePredictor != nullptr) {
                    smartKayak->strokePredictor->learningMethod = LearningMethod::METHOD_B;
                    Serial.println("Switched to Learning Method B");
                }
                break;
        }
    }
}

void loop() {
    processSerialCommand();
    // ... остальной код ...
}
```

## 5. Отображение на дисплее

Если есть дисплей, можно показывать статус предиктора:

```cpp
void updateDisplay() {
    if (display != nullptr && smartKayak->strokePredictor != nullptr) {
        char buffer[32];
        
        // Количество заполненных ячеек
        uint32_t cells = smartKayak->strokePredictor->getFilledCellsCount();
        sprintf(buffer, "Grid: %u cells", cells);
        display->printLine(0, buffer);
        
        // Средняя уверенность
        float avgConf = smartKayak->strokePredictor->getAverageConfidence();
        sprintf(buffer, "Conf: %.1f", avgConf);
        display->printLine(1, buffer);
        
        // Статус
        sprintf(buffer, "Pred: %s", 
                smartKayak->usePredictorGrid ? "ON" : "OFF");
        display->printLine(2, buffer);
    }
}
```

## 6. Постепенное включение

Рекомендуется постепенно переходить на новый предиктор:

```cpp
// Фаза 1: Только обучение (без применения)
void SmartKayak::update(const OrientationData& paddleOrientation) {
    // ... существующий код ...
    
    // Обучаем в фоне
    strokePredictor->learn(...);
    
    // Но используем старую логику для управления мотором
    force = calculateForceOldWay();
    
    // Выводим сравнение
    float predictedForce = strokePredictor->predict(...);
    Serial.printf("Old: %d, Predicted: %.1f\n", force, predictedForce);
}

// Фаза 2: Смешанное использование
force = (int)(0.7 * oldForce + 0.3 * predictedForce);

// Фаза 3: Только новый предиктор (с fallback)
if (confidence > 0.5f) {
    force = predictedForce;
} else {
    force = oldForce;  // Fallback на старую логику
}
```

## 7. Мониторинг и настройка

### Логирование для анализа
```cpp
void SmartKayak::logPredictorData() {
    static File logFile;
    
    if (!logFile) {
        logFile = SD.open("/predictor_log.csv", FILE_WRITE);
        logFile.println("time,angle1,angle2,angle3,force,predicted,confidence");
    }
    
    // Записываем данные
    logFile.printf("%lu,%.2f,%.2f,%.2f,%.1f,%.1f,%.2f\n",
                   millis(),
                   shaftRotationAngle,
                   shaftTiltAngle,
                   bladeRotationAngle,
                   realForce,
                   predictedForce,
                   confidence);
}
```

### Автоматическая настройка
```cpp
void SmartKayak::autoTunePredictor() {
    // После N гребков настраиваем параметры
    static int strokeCount = 0;
    strokeCount++;
    
    if (strokeCount == 50) {
        // Первая настройка
        float avgConf = strokePredictor->getAverageConfidence();
        if (avgConf < 5.0f) {
            // Увеличиваем alpha для более быстрого обучения
            strokePredictor->config.alphaEMA = 0.3f;
        }
    }
    
    if (strokeCount == 200) {
        // Финальная настройка
        strokePredictor->config.alphaEMA = 0.15f;  // Замедляем обучение
        Serial.println("Predictor auto-tuned!");
    }
}
```

---

## Резюме интеграции

1. ✅ Добавить поле `strokePredictor` в `SmartKayak`
2. ✅ Инициализировать в конструкторе
3. ✅ Вызывать `learn()` в каждом `update()`
4. ✅ Использовать `predict()` для управления мотором
5. ✅ Добавить методы управления и мониторинга
6. ✅ Реализовать плавный переход от старой системы
7. ✅ Настроить параметры под реальные условия

**Рекомендуется:** начать с фазы 1 (только обучение + логирование), затем перейти к фазам 2 и 3.

