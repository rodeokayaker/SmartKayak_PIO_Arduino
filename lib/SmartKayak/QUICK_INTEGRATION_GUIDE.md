# ⚡ Быстрая интеграция HybridOrientationEstimator

## 🎯 Цель
Заменить простую функцию `getRelativeOrientation()` на продвинутую систему с:
- ✅ Адаптивным использованием магнитометра (только когда данные надежны)
- ✅ Коррекцией дрифта гироскопа по паттернам гребли
- ✅ Детекцией фаз гребка
- ✅ Детекцией магнитных помех

---

## 📦 Шаг 1: Добавьте файлы

Скопируйте в `lib/SmartKayak/`:
```
✅ HybridOrientationEstimator.h
✅ HybridOrientationEstimator.cpp
✅ MagnetometerCalibrator.h (опционально)
✅ MagnetometerCalibrator.cpp (опционально)
```

---

## 🔧 Шаг 2: Измените SmartKayak.h

### Добавьте включения:
```cpp
#include "HybridOrientationEstimator.h"
```

### Добавьте в private секцию класса:
```cpp
private:
    HybridOrientationEstimator orientationEstimator;
```

---

## ⚙️ Шаг 3: Настройте в конструкторе

В `SmartKayak.cpp`, конструктор:

```cpp
SmartKayak::SmartKayak() : /* ... существующая инициализация ... */
{
    // ... существующий код ...
    
    // Настройка гибридного оценщика
    MagneticAnomalyDetector detector;
    detector.varianceThreshold = 40.0f;
    detector.magnitudeMin = 30.0f;
    detector.magnitudeMax = 70.0f;
    orientationEstimator.setAnomalyDetector(detector);
    
    orientationEstimator.setFilterWeights(
        0.98f,  // gyro
        0.02f,  // accel
        0.3f,   // mag (адаптивный)
        0.2f    // pattern
    );
}
```

---

## 🔄 Шаг 4: Замените код в update()

### ❌ СТАРЫЙ КОД (удалите):
```cpp
SP_Math::Quaternion currentPaddleQ(paddleOrientation.q0, ...);
SP_Math::Quaternion paddleRelativeQuat = getRelativeOrientation(currentPaddleQ, paddle);
```

### ✅ НОВЫЙ КОД (вставьте):
```cpp
// Получение данных IMU
IMUData paddleIMU = paddle->getIMUData();
IMUData kayakIMU;
imu->getData(kayakIMU);

// Подготовка ориентации каяка
OrientationData kayakOrientation;
kayakOrientation.q0 = kayakOrientationQuat[0];
kayakOrientation.q1 = kayakOrientationQuat[1];
kayakOrientation.q2 = kayakOrientationQuat[2];
kayakOrientation.q3 = kayakOrientationQuat[3];

// Получение высокоточной относительной ориентации
SP_Math::Quaternion paddleRelativeQuat = 
    orientationEstimator.updateRelativeOrientation(
        paddleIMU,           // IMU весла
        paddleOrientation,   // DMP весла
        kayakIMU,            // IMU каяка
        kayakOrientation     // DMP каяка
    );
```

---

## 🎨 Шаг 5: Добавьте детекцию фаз (опционально)

После получения углов весла:

```cpp
float shaftRotationAngle, shaftTiltAngle, bladeRotationAngle;
getPaddleAngles(paddleRelativeQuat, shaftRotationAngle, shaftTiltAngle, bladeRotationAngle);

// Детекция фазы гребка
StrokePhase::Phase currentPhase = orientationEstimator.detectStrokePhase(
    shaftRotationAngle,
    shaftTiltAngle,
    bladeForce
);

// Модуляция силы мотора по фазе
if (orientationEstimator.getPhaseConfidence() > 0.7f) {
    if (currentPhase == StrokePhase::PULL) {
        force *= 1.2f;  // +20% во время проводки
    } else if (currentPhase == StrokePhase::RECOVERY) {
        force *= 0.5f;  // Снижаем во время возврата
    }
}
```

---

## 📊 Шаг 6: Добавьте диагностику

В `loop()` или в вашей функции отладки:

```cpp
static unsigned long lastDiag = 0;
if (millis() - lastDiag > 3000) {  // Каждые 3 секунды
    float magRel = orientationEstimator.getMagReliability();
    
    Serial.printf("Mag: %.0f%% | Phase: ", magRel * 100);
    switch(orientationEstimator.getCurrentPhase()) {
        case StrokePhase::CATCH: Serial.print("CATCH"); break;
        case StrokePhase::PULL: Serial.print("PULL"); break;
        case StrokePhase::RELEASE: Serial.print("RELEASE"); break;
        case StrokePhase::RECOVERY: Serial.print("RECOVERY"); break;
        default: Serial.print("UNKNOWN"); break;
    }
    Serial.printf(" (%.0f%%)\n", orientationEstimator.getPhaseConfidence() * 100);
    
    lastDiag = millis();
}
```

---

## 🧭 Шаг 7: Калибровка (рекомендуется)

### Добавьте функцию калибровки:

```cpp
void calibrateMagnetometer() {
    Serial.println("Калибровка магнитометра весла...");
    Serial.println("Медленно вращайте весло во всех направлениях 30 сек");
    
    // Используйте MagnetometerCalibrator из примера
    // или простую процедуру записи min/max значений
    
    delay(30000);
    Serial.println("Калибровка завершена!");
}
```

### Запустите при первом старте:
```cpp
void setup() {
    // ... инициализация ...
    
    if (!loadCalibration()) {  // Если нет сохраненной калибровки
        calibrateMagnetometer();
    }
}
```

---

## ✅ Готово!

Система теперь:
- 🎯 Автоматически детектирует магнитные помехи
- 🔄 Использует магнитометр только когда данные надежны
- 📊 Корректирует дрифт гироскопа по паттернам гребли
- 🚣 Определяет фазы гребка

---

## 🔍 Проверка работы

### Хорошие показатели:
- ✅ Mag Reliability > 70% → система использует магнитометр
- ✅ Phase Confidence > 70% → фазы гребка распознаются правильно
- ✅ Плавное изменение углов без скачков

### Проблемы:
- ❌ Mag Reliability < 30% → сильные помехи, нужна калибровка
- ❌ Phase всегда UNKNOWN → проверьте пороги в detectStrokePhase
- ❌ Резкие скачки yaw → увеличьте patternWeight

---

## 📝 Настройка под ваши условия

### Для спокойной воды:
```cpp
detector.varianceThreshold = 25.0f;  // Низкий порог
orientationEstimator.setFilterWeights(0.98f, 0.02f, 0.4f, 0.15f);
```

### Для волн и помех:
```cpp
detector.varianceThreshold = 60.0f;  // Высокий порог
orientationEstimator.setFilterWeights(0.98f, 0.02f, 0.2f, 0.35f);
```

### Без магнитометра (критические помехи):
```cpp
orientationEstimator.setFilterWeights(0.98f, 0.02f, 0.0f, 0.5f);
```

---

## 📚 Дополнительно

- 📖 Полная документация: `HYBRID_ORIENTATION_README.md`
- 💡 Примеры: `examples/HybridOrientation_Example.cpp`
- 🔧 Тонкая настройка: см. раздел "Тонкая настройка" в README

**Удачи! 🚣‍♂️**
