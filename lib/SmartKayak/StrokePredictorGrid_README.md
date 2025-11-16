# StrokePredictorGrid - Предиктор гребка на основе 3D сетки

## Описание

`StrokePredictorGrid` - это система предсказания нагрузки на лопатку весла, основанная на обучаемой 3D сетке положений весла. Система использует текущую ориентацию весла и угловую скорость для предсказания нагрузки через заданный интервал времени (по умолчанию 200мс).

## Принцип работы

### 1. Трехмерная сетка
Пространство положений весла разбито на 3D сетку по трем углам:
- **Shaft Rotation** (поворот шафта): ±180° с шагом 10°
- **Shaft Tilt** (наклон шафта): ±90° с шагом 10°
- **Blade Rotation** (поворот лопасти): ±180° с шагом 10°

Всего возможных ячеек: **36 × 18 × 36 = 23,328**

Используется **Sparse Grid** (разреженная сетка) - хранятся только заполненные ячейки в `unordered_map`.

### 2. Структура ячейки
Каждая ячейка хранит:
```cpp
struct GridCell {
    float force_forward;      // Усилие для гребка вперед
    float force_backward;     // Усилие для гребка назад
    uint16_t confidence;      // Количество обновлений (уверенность)
};
```

### 3. Обучение

Два метода обучения:

#### Метод A: Корректировка по ошибке
```
predicted = интерполяция_из_сетки(текущие_углы)
error = real_value - predicted
для каждой вершины куба:
    correction = error × вес_вершины × alpha
    vertex.value += correction
```

#### Метод B: Прямое взвешенное обновление (по умолчанию)
```
для каждой вершины куба:
    alpha_weighted = alpha × вес_вершины
    vertex.value = vertex.value × (1 - alpha_weighted) + real_value × alpha_weighted
```

При первом обновлении ячейки значение устанавливается напрямую.

### 4. Предсказание

1. **Экстраполяция ориентации** через deltaT:
   ```
   omega_future = omega + angular_acceleration × deltaT
   predicted_quat = current_quat × exp(omega_future × deltaT)
   ```

2. **Извлечение углов** из предсказанной ориентации

3. **Trilinear интерполяция** по 8 соседним вершинам куба:
   ```
   value = Σ(vertex_value × weight) / Σ(weight)
   ```
   где вес учитывает расстояние до вершины.

4. **Обработка незаполненных вершин**: используются только заполненные вершины, их веса перенормализуются.

## Использование

### Инициализация
```cpp
#include "StrokePredictorGrid.h"

StrokePredictorGrid predictor;

// Настройка
predictor.config.gridStep = 10.0f;           // Шаг сетки
predictor.config.alphaEMA = 0.2f;            // Скорость обучения
predictor.config.forceThreshold = 400.0f;    // Порог силы
predictor.config.predictionTime = 200.0f;    // Время предсказания
predictor.learningMethod = LearningMethod::METHOD_B;
```

### Обучение
```cpp
void loop() {
    // Получаем данные
    SP_Math::Quaternion paddleQuat = getPaddleOrientation();
    SP_Math::Quaternion kayakQuat = getKayakOrientation();
    SP_Math::Vector omega = getAngularVelocity();
    float currentForce = getLoadCellForce();
    bool isForward = (motorMode == FORWARD);
    
    // Обучаем
    predictor.learn(paddleQuat, kayakQuat, omega, currentForce, isForward);
}
```

### Предсказание
```cpp
// Предсказываем силу через 200мс
float confidence = 0.0f;
float predictedForce = predictor.predict(
    paddleQuat,           // Текущая ориентация весла
    kayakQuat,            // Текущая ориентация каяка
    omega,                // Угловая скорость
    200.0f,               // Время предсказания (мс)
    true,                 // Направление (forward)
    true,                 // Использовать ускорение
    &confidence           // Уверенность предсказания (0..1)
);

if (confidence > 0.5f) {
    // Предсказание достаточно уверенное
    applyMotorForce(predictedForce);
}
```

### Отладка
```cpp
// Статистика
Serial.printf("Filled cells: %u\n", predictor.getFilledCellsCount());
Serial.printf("Avg confidence: %.2f\n", predictor.getAverageConfidence());

// Полная статистика
predictor.printGridStatistics();

// Очистка сетки
predictor.clearGrid();
```

## Параметры конфигурации

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|--------------|----------|
| `gridStep` | float | 10.0 | Шаг сетки в градусах (5-20°) |
| `alphaEMA` | float | 0.2 | Коэффициент экспоненциального скользящего среднего |
| `forceThreshold` | float | 400.0 | Порог силы для обучения (граммы) |
| `predictionTime` | float | 200.0 | Время предсказания по умолчанию (мс) |
| `omegaHistorySize` | uint8_t | 5 | Размер истории для вычисления ускорения |

## Использование памяти

### Минимальное (пустая сетка)
- Базовый объект: ~100 байт
- История omega: 5 × 12 = 60 байт
- **Итого: ~160 байт**

### Максимальное (полностью заполненная сетка)
- 23,328 ячеек × 24 байта (10 данных + 14 overhead map) = **560 КБ**

### Реальное использование
- После 100 гребков: ~200-500 заполненных ячеек = **5-12 КБ**
- После 1000 гребков: ~1000-2000 ячеек = **24-48 КБ**

## Производительность

На ESP32 @ 240MHz:
- Обучение: ~1000 операций = **~40 мкс**
- Предсказание: ~1500 операций = **~60 мкс**
- При частоте 100Hz нагрузка: **0.4-0.6%**

## Преимущества

✅ Адаптируется к стилю гребли пользователя  
✅ Учитывает все 3 степени свободы весла  
✅ Постоянное обучение - улучшается со временем  
✅ Экономит память - sparse grid  
✅ Быстрые вычисления  
✅ Два метода обучения на выбор  

## Ограничения

⚠️ Требуется время на обучение (~20-50 гребков для базовой точности)  
⚠️ Точность зависит от постоянства техники гребли  
⚠️ При шаге 10° разрешение может быть недостаточным для очень точных предсказаний  

## Пример вывода статистики

```
=== Grid Statistics ===
Filled cells: 342
Average confidence: 12.45
Grid step: 10.0 degrees
Alpha EMA: 0.20
Force threshold: 400.0
Learning method: METHOD_B
Force forward range: -50.2 .. 1234.5
Force backward range: -120.3 .. 890.2
Confidence range: 1 .. 45
=======================
```

## См. также

- `examples/test_stroke_predictor_grid.cpp` - пример использования
- `StrokePredictor.h` - базовый интерфейс предикторов
- `StrokePredictorOne.h` - альтернативная реализация на основе паттернов

