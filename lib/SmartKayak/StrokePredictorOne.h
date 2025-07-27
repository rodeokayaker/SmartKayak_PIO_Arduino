#ifndef STROKEPREDICTORONE_H
#define STROKEPREDICTORONE_H

#include "StrokePredictor.h"

// Конфигурация алгоритма
#define PATTERN_SAMPLE_INTERVAL_MS 50    // Интервал сэмплирования паттерна (50ms)
#define MAX_PATTERNS_PER_DIRECTION 5     // Максимум паттернов на направление (левый/правый вперед/назад)
#define ANTICIPATION_TIME_MS 168         // Время опережения из анализа
#define SIMILARITY_THRESHOLD 0.60        // Порог похожести
#define MIN_STROKE_DURATION_MS 400       // Минимальная длительность гребка
#define MAX_STROKE_DURATION_MS 3000      // Максимальная длительность гребка
#define PATTERN_LENGTH (MAX_STROKE_DURATION_MS/PATTERN_SAMPLE_INTERVAL_MS) // Максимальный размер паттерна (60 сэмплов)
#define MIN_SHAFT_Z_VELOCITY 0.5         // Минимальная скорость изменения z составляющей шафта для начала гребка
#define UPDATE_FREQUENCY_HZ 100          // Частота обновления данных

// Веса для расчета схожести паттернов
#define SHAFT_Z_SIMILARITY_WEIGHT 0.5    // Вес схожести по shaft_z профилю
#define BLADE_Z_SIMILARITY_WEIGHT 0.5    // Вес схожести по blade_z профилю

struct StrokePattern {
    float shaft_z_profile[PATTERN_LENGTH];   // Сжатая кривая z составляющей вектора шафта
    float blade_z_profile[PATTERN_LENGTH];   // Сжатая кривая z составляющей вектора нормали лопасти
    float motor_force_profile[PATTERN_LENGTH]; // Сжатая кривая усилий мотора
    uint16_t duration_ms;                    // Длительность гребка
    uint8_t stroke_number;                   // Номер гребка в последовательности
    bool is_braking;                         // Тормозящий гребок (повтор стороны)
    uint32_t timestamp;                      // Время создания паттерна
    uint32_t usage_count;                    // Счетчик использования паттерна
    float avg_velocity;                      // Средняя скорость для нормализации
    float max_motor_force;                   // Максимальное усилие мотора
    BladeSideType bladeSide;                 // Сторона лопасти
    bool is_forward;                         // Направление гребка (определяется по знаку force)
    bool is_valid;                           // Валиден ли паттерн
};

// Буфер для текущего гребка
struct CurrentStroke {
    StrokePattern* matched_pattern;          // Подходящий паттерн
    uint8_t pattern_index;                   // Индекс паттерна
    uint32_t start_time;                     // Время начала гребка
    uint32_t current_time;                   // Текущее время в гребке
    bool is_active;                          // Активен ли гребок
    bool motor_triggered;                    // Был ли запущен мотор
    uint32_t motor_trigger_time;             // Время запуска мотора
    float confidence;                        // Уверенность предсказания
    bool pattern_switched;                   // Был ли переключен паттерн во время гребка
    float current_shaft_z;                   // Текущая z составляющая шафта
    float prev_shaft_z;                      // Предыдущая z составляющая шафта
    bool shaft_z_crossed_zero;               // Пересекла ли z составляющая ноль
};

#define PADDLE_BUFFER_SIZE 200
struct PaddleBuffer {
    float shaft_z[PADDLE_BUFFER_SIZE];       // Буфер z состовляющей вектора оси весла
    float blade_z[PADDLE_BUFFER_SIZE];       // Буфер z состовляющей вектора нормали лопасти
    float motor_force[PADDLE_BUFFER_SIZE];   // Буфер усилий мотора
    uint32_t timestamps[PADDLE_BUFFER_SIZE]; // Буфер временных меток
    uint8_t buffer_index;                    // Текущий индекс буфера
    bool buffer_full;                        // Заполнен ли буфер
};

class StrokePredictorOne : public StrokePredictor {
private:
    // Паттерны для каждого направления: left forward, right forward, left reverse, right reverse
    StrokePattern patterns[4][MAX_PATTERNS_PER_DIRECTION];
    uint8_t pattern_count[4];                // Количество паттернов для каждого направления
    
    CurrentStroke currentStroke;
    PaddleBuffer paddleBuffer;
    
    uint32_t nextTimePaddle;
    uint32_t nextTimePattern;
    
    // Состояние для детекции пересечения горизонтали
    float last_shaft_z;
    float shaft_z_velocity;
    uint32_t last_update_time;
    bool recording_pattern;                  // Записываем ли паттерн
    StrokePattern new_pattern;               // Новый записываемый паттерн
    uint8_t new_pattern_samples;             // Количество записанных семплов

public:
    StrokePredictorOne();
    virtual ~StrokePredictorOne();
    
    // Обновленная сигнатура: работаем только с z составляющими векторов
    virtual float PredictForce(float shaft_z, float blade_z, BladeSideType bladeSide, 
                              float current_force, float& coefPredict);

    void resetPattern(BladeSideType bladeSide, bool forward);

private:
    // Вспомогательные методы
    uint8_t getDirectionIndex(BladeSideType bladeSide, bool forward);
    bool detectHorizonCrossing(float shaft_z);
    void startPatternRecording(BladeSideType bladeSide, bool forward);
    void recordPatternSample(float shaft_z, float blade_z, float motor_force);
    void finishPatternRecording();
    float calculatePatternSimilarity(const StrokePattern& pattern1, const StrokePattern& pattern2);
    StrokePattern* findBestMatchingPattern(BladeSideType bladeSide, bool forward, float& confidence);
    void addNewPattern(const StrokePattern& pattern, BladeSideType bladeSide, bool forward);
    void removeOldestLeastUsedPattern(uint8_t direction_index);
    float predictForceFromPattern(const StrokePattern& pattern, uint32_t current_time_in_stroke);
    bool isForwardStroke(float current_force);
    
    float coefPredict;
};

#endif