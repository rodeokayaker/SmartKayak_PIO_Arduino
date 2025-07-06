#ifndef STROKEPREDICTORONE_H
#define STROKEPREDICTORONE_H

#include "StrokePredictor.h"


// Конфигурация алгоритма
#define PATTERN_DIMENSION 50
#define MAX_PATTERNS_PER_SIDE 1          // Максимум паттернов на сторону
#define ANTICIPATION_TIME_MS 168         // Время опережения из анализа
#define SIMILARITY_THRESHOLD 0.60        // Порог похожести
#define MIN_STROKE_DURATION_MS 500       // Минимальная длительность гребка
#define MAX_STROKE_DURATION_MS 3000      // Максимальная длительность гребка
#define PITCH_VELOCITY_THRESHOLD 5.0     // Порог скорости pitch для детекции
#define PATTERN_LENGTH MAX_STROKE_DURATION_MS/PATTERN_DIMENSION                // Сжатый размер паттерна


struct StrokePattern {
    float strain_profile[PATTERN_LENGTH];    // Сжатая кривая нагрузки
    float motor_profile[PATTERN_LENGTH];     // Сжатая кривая мотора
    float pitch_start[10];                   // Начальный сегмент pitch для быстрого сравнения
    uint16_t duration_ms;                    // Длительность гребка
    uint8_t stroke_number;                   // Номер гребка в последовательности
    bool is_braking;                         // Тормозящий гребок (повтор стороны)
    uint32_t timestamp;                      // Время создания паттерна
    float avg_strain;                        // Средняя нагрузка для нормализации
    float max_strain;                        // Максимальная нагрузка
    BladeSideType bladeSide;                 // Сторона лопасти
};

// Буфер для текущего гребка
struct CurrentStroke {
    StrokePattern* pattern;
    uint8_t pattern_index;
    StrokePattern modified_pattern;
    uint32_t next_time;
    float pitch_buffer[100];                 // Буфер pitch данных
    float strain_buffer[100];                // Буфер нагрузки
    uint8_t buffer_index;                    // Текущий индекс буфера
    uint32_t start_time;                     // Время начала гребка
    bool is_active;                          // Активен ли гребок
    bool motor_triggered;                    // Был ли запущен мотор
    uint32_t motor_trigger_time;             // Время запуска мотора
    float confidence;                        // Уверенность предсказания
};

struct PaddleBuffer {
    float pitch_buffer[100];                 // Буфер pitch данных
    float strain_buffer[100];                // Буфер нагрузки
    uint8_t buffer_index;                    // Текущий индекс буфера
}



class StrokePredictorOne : public StrokePredictor {
    private:
        StrokePattern Pattern[4]; //left forward, right forward, left reverse, right reverse;
        bool PatternIsFilled[4];

        CurrentStroke currentStroke[2];
        int8_t currentStrokeIndex;
        int8_t nextStrokeIndex;
        
        PaddleBuffer paddleBuffer;
        
        uint32_t nextTimePaddle;
        uint32_t nextTimePattern;

        StrokePattern AdoptedPattern;
    // IMU данные
        float last_pitch;
        float pitch_velocity;
        uint32_t last_imu_time;

    public:
        StrokePredictorOne();
        virtual ~StrokePredictorOne();
        virtual float PredictForce (float shaftRotationAngle,
         float shaftTiltAngle,
         float bladeRotationAngle, 
         BladeSideType bladeSide, 
         float CalculatedForce, 
         float& coefPredict, 
         float& coefCalculate); 

        void resetPattern(BladeSideType bladeSide, bool forward) 
        {
            if (bladeSide == BladeSideType::LEFT_BLADE) {
                if (forward) {
                    PatternIsFilled[0] = false;
                } else {
                    PatternIsFilled[2] = false;
                }
            } else {
                if (forward) {
                    PatternIsFilled[1] = false;
                } else {
                    PatternIsFilled[3] = false;
                }
            }
        };


    private:
        float coefPredict;
        float coefCalculate;
};
#endif