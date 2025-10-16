# Быстрый старт: Фильтр относительной ориентации

## 🚀 За 5 минут до точной ориентации весла

### Шаг 1: Добавьте файлы в проект

Скопируйте в ваш проект:
- `lib/SmartKayak/RelativeOrientationFilter.h`
- `lib/SmartKayak/RelativeOrientationFilter.cpp`

### Шаг 2: Обновите SmartKayak.h

```cpp
#include "RelativeOrientationFilter.h"

class SmartKayak {
    // ... существующие поля ...
    
private:
    RelativeOrientationFilter orientationFilter;
    uint32_t lastFilterUpdateTime;
    
public:
    // Добавьте метод
    void calibrateMagnetometer() {
        orientationFilter.calibrateReference();
    }
};
```

### Шаг 3: Инициализация в конструкторе

```cpp
SmartKayak::SmartKayak():
    // ... существующая инициализация ...
    lastFilterUpdateTime(0)
{
    // ... существующий код ...
}
```

### Шаг 4: Замените код в update()

**Найдите эту строку:**
```cpp
SP_Math::Quaternion paddleRelativeQuat = getRelativeOrientation(currentPaddleQ, paddle);
```

**Замените на:**
```cpp
// Получаем сырые данные IMU
IMUData kayakIMU, paddleIMU;
imu->getData(kayakIMU);
paddleIMU = paddle->getIMUData();

// Создаем векторы
SP_Math::Vector kayakMag(kayakIMU.mag_x, kayakIMU.mag_y, kayakIMU.mag_z);
SP_Math::Vector paddleMag(paddleIMU.mag_x, paddleIMU.mag_y, paddleIMU.mag_z);
SP_Math::Vector kayakGyro(kayakIMU.gx, kayakIMU.gy, kayakIMU.gz);
SP_Math::Vector paddleGyro(paddleIMU.gx, paddleIMU.gy, paddleIMU.gz);

// Вычисляем dt
uint32_t currentTime = millis();
float dt = (currentTime - lastFilterUpdateTime) / 1000.0f;
if (lastFilterUpdateTime == 0) dt = 0.01f; // первый раз
lastFilterUpdateTime = currentTime;

// ОСНОВНОЙ ВЫЗОВ ФИЛЬТРА
SP_Math::Quaternion paddleRelativeQuat = orientationFilter.update(
    kayakOrientationQuat,
    currentPaddleQ,
    kayakMag,
    paddleMag,
    kayakGyro,
    paddleGyro,
    dt
);
```

**И добавьте после вычисления углов:**
```cpp
// Обновляем состояние гребка для фильтра
bool strokeActive = (abs(bladeForce) > borderLoadForce);
orientationFilter.updateStrokeState(
    strokeActive,
    bladeSide,
    shaftRotationAngle,
    bladeForce
);
```

### Шаг 5: Калибровка при старте

В вашем `setup()` или в начале работы:

```cpp
void setup() {
    // ... инициализация ...
    
    // Ждем стабилизации IMU
    delay(3000);
    
    // ВАЖНО: Каяк должен быть неподвижен!
    Serial.println("Калибровка магнитометра...");
    smartKayak.calibrateMagnetometer();
    Serial.println("Готово!");
}
```

### Шаг 6: (Опционально) Диагностика

Добавьте в loop() для мониторинга:

```cpp
void loop() {
    smartKayak.update();
    
    // Каждые 2 секунды выводим статус
    static uint32_t lastDiag = 0;
    if (millis() - lastDiag > 2000) {
        lastDiag = millis();
        
        float kayakTrust, paddleTrust;
        smartKayak.orientationFilter.getMagnetometerTrust(kayakTrust, paddleTrust);
        
        Serial.printf("Mag trust: K=%.2f P=%.2f", kayakTrust, paddleTrust);
        
        if (kayakTrust < 0.3 || paddleTrust < 0.3) {
            Serial.print(" ⚠️ ПОМЕХИ!");
        }
        Serial.println();
    }
}
```

## ✅ Готово!

Теперь у вас:
- ✨ Точная ориентация весла без скачков
- 🛡️ Защита от магнитных помех
- 🔄 Коррекция на основе паттернов гребли
- 📊 Автоматическая адаптация к условиям

## 🎯 Проверка работы

### Хорошие показатели:
```
Mag trust: K=0.85 P=0.92  <- Отлично!
```

### Помехи обнаружены:
```
Mag trust: K=0.15 P=0.25 ⚠️ ПОМЕХИ!
```
В этом случае система использует только DMP - это нормально!

### Если нужна рекалибровка:

1. Остановите каяк
2. Подождите 2 секунды (стабилизация)
3. Вызовите:
```cpp
smartKayak.calibrateMagnetometer();
```

## 🔧 Быстрая настройка

Если точность недостаточна, попробуйте в `RelativeOrientationFilter.h`:

**Слишком чувствительно к помехам?**
```cpp
#define MAG_VARIANCE_THRESHOLD 1200.0f  // было 800
```

**Медленная коррекция дрифта?**
```cpp
#define YAW_DRIFT_CORRECTION_RATE 0.05f  // было 0.02
```

**Ориентация прыгает?**
```cpp
#define GYRO_INTEGRATION_WEIGHT 0.99f  // было 0.98
```

## 📞 Поддержка

Если что-то не работает:

1. Проверьте, что магнитометры возвращают разумные значения (не нули)
2. Убедитесь, что DMP кватернионы нормализованы
3. Проверьте частоту вызова update() (должна быть 50-100 Hz)
4. При старте дайте IMU 2-3 секунды на стабилизацию

## 🎓 Подробнее

См. полную документацию:
- `docs/OrientationFilterTheory_RU.md` - теория и математика
- `lib/SmartKayak/RelativeOrientationIntegration.md` - подробная интеграция
- `examples/OrientationFilterExample/` - пример с диагностикой

---

**Вопросы?** Проверьте Serial вывод - там будут подсказки!
