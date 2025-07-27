#include "StrokePredictorOne.h"
#include <math.h>

StrokePredictorOne::StrokePredictorOne() {
    // Инициализация массивов паттернов
    for (int i = 0; i < 4; i++) {
        pattern_count[i] = 0;
        for (int j = 0; j < MAX_PATTERNS_PER_DIRECTION; j++) {
            patterns[i][j].is_valid = false;
            patterns[i][j].usage_count = 0;
        }
    }
    
    // Инициализация буферов
    paddleBuffer.buffer_index = 0;
    paddleBuffer.buffer_full = false;
    
    // Инициализация состояния гребка
    currentStroke.is_active = false;
    currentStroke.matched_pattern = nullptr;
    currentStroke.motor_triggered = false;
    currentStroke.pattern_switched = false;
    currentStroke.shaft_z_crossed_zero = false;
    
    // Инициализация детекции пересечения
    last_shaft_z = 0.0f;
    shaft_z_velocity = 0.0f;
    last_update_time = 0;
    recording_pattern = false;
    new_pattern_samples = 0;
    
    nextTimePaddle = 0;
    nextTimePattern = 0;
    coefPredict = 0.0f;
}

StrokePredictorOne::~StrokePredictorOne() {
    // Очистка не требуется для простых типов
}

float StrokePredictorOne::PredictForce(float shaft_z, float blade_z, BladeSideType bladeSide, 
                                      float current_force, float& coefPredict) {
    uint32_t current_time = millis();
    
    // Обновляем буфер каждые ~10ms (100Hz)
    if (current_time >= nextTimePaddle) {
        // Добавляем данные в буфер
        paddleBuffer.shaft_z[paddleBuffer.buffer_index] = shaft_z;
        paddleBuffer.blade_z[paddleBuffer.buffer_index] = blade_z;
        paddleBuffer.motor_force[paddleBuffer.buffer_index] = current_force;
        paddleBuffer.timestamps[paddleBuffer.buffer_index] = current_time;
        
        paddleBuffer.buffer_index++;
        if (paddleBuffer.buffer_index >= PADDLE_BUFFER_SIZE) {
            paddleBuffer.buffer_index = 0;
            paddleBuffer.buffer_full = true;
        }
        
        nextTimePaddle = current_time + (1000 / UPDATE_FREQUENCY_HZ);
    }
    
    // Обновляем состояние детекции пересечения горизонтали
    if (last_update_time > 0) {
        float dt = (current_time - last_update_time) / 1000.0f; // в секундах
        if (dt > 0) {
            shaft_z_velocity = (shaft_z - last_shaft_z) / dt;
        }
    }
    
    // Проверяем пересечение горизонтали
    bool horizon_crossed = detectHorizonCrossing(shaft_z);
    
    if (horizon_crossed) {
        // Если был активный гребок - завершаем его
        if (currentStroke.is_active) {
            finishPatternRecording();
        }
        
        // Определяем направление нового гребка
        bool is_forward = isForwardStroke(current_force);
        
        // Начинаем новый гребок
        currentStroke.is_active = true;
        currentStroke.start_time = current_time;
        currentStroke.motor_triggered = false;
        currentStroke.pattern_switched = false;
        currentStroke.shaft_z_crossed_zero = true;
        currentStroke.current_shaft_z = shaft_z;
        currentStroke.prev_shaft_z = last_shaft_z;
        
        // Ищем подходящий паттерн
        float confidence = 0.0f;
        currentStroke.matched_pattern = findBestMatchingPattern(bladeSide, is_forward, confidence);
        currentStroke.confidence = confidence;
        
        if (currentStroke.matched_pattern == nullptr || confidence < SIMILARITY_THRESHOLD) {
            // Начинаем запись нового паттерна
            startPatternRecording(bladeSide, is_forward);
        } else {
            // Увеличиваем счетчик использования паттерна
            currentStroke.matched_pattern->usage_count++;
        }
    }
    
    // Записываем паттерн если активна запись
    if (recording_pattern && current_time >= nextTimePattern) {
        recordPatternSample(shaft_z, blade_z, current_force);
        nextTimePattern = current_time + PATTERN_SAMPLE_INTERVAL_MS;
    }
    
    // Предсказываем усилие если есть подходящий паттерн
    float predicted_force = 0.0f;
    if (currentStroke.is_active && currentStroke.matched_pattern != nullptr) {
        uint32_t time_in_stroke = current_time - currentStroke.start_time;
        
        // Проверяем соответствие текущих данных паттерну
        if (time_in_stroke > 100) { // Проверяем только после 100ms
            // Простая проверка на соответствие - можно улучшить
            float pattern_time_ratio = (float)time_in_stroke / currentStroke.matched_pattern->duration_ms;
            if (pattern_time_ratio > 1.5f) {
                // Гребок слишком долгий - переключаемся на запись нового паттерна
                currentStroke.pattern_switched = true;
                currentStroke.matched_pattern = nullptr;
                startPatternRecording(bladeSide, isForwardStroke(current_force));
            }
        }
        
        if (currentStroke.matched_pattern != nullptr) {
            predicted_force = predictForceFromPattern(*currentStroke.matched_pattern, 
                                                    time_in_stroke + ANTICIPATION_TIME_MS);
            coefPredict = currentStroke.confidence;
        }
    }
    
    // Обновляем последние значения
    last_shaft_z = shaft_z;
    last_update_time = current_time;
    
    return predicted_force;
}

bool StrokePredictorOne::detectHorizonCrossing(float shaft_z) {
    // Проверяем изменение знака
    bool sign_changed = (last_shaft_z * shaft_z < 0) && (last_update_time > 0);
    
    // Проверяем достаточную скорость изменения
    bool sufficient_velocity = fabs(shaft_z_velocity) > MIN_SHAFT_Z_VELOCITY;
    
    return sign_changed && sufficient_velocity;
}

void StrokePredictorOne::startPatternRecording(BladeSideType bladeSide, bool forward) {
    recording_pattern = true;
    new_pattern_samples = 0;
    
    // Инициализируем новый паттерн
    new_pattern.bladeSide = bladeSide;
    new_pattern.is_forward = forward;
    new_pattern.timestamp = millis();
    new_pattern.usage_count = 1;
    new_pattern.is_valid = false;
    
    // Очищаем профили
    for (int i = 0; i < PATTERN_LENGTH; i++) {
        new_pattern.shaft_z_profile[i] = 0.0f;
        new_pattern.blade_z_profile[i] = 0.0f;
        new_pattern.motor_force_profile[i] = 0.0f;
    }
}

void StrokePredictorOne::recordPatternSample(float shaft_z, float blade_z, float motor_force) {
    if (new_pattern_samples < PATTERN_LENGTH) {
        new_pattern.shaft_z_profile[new_pattern_samples] = shaft_z;
        new_pattern.blade_z_profile[new_pattern_samples] = blade_z;
        new_pattern.motor_force_profile[new_pattern_samples] = motor_force;
        new_pattern_samples++;
    }
}

void StrokePredictorOne::finishPatternRecording() {
    if (recording_pattern && new_pattern_samples >= (MIN_STROKE_DURATION_MS / PATTERN_SAMPLE_INTERVAL_MS)) {
        // Завершаем паттерн
        new_pattern.duration_ms = new_pattern_samples * PATTERN_SAMPLE_INTERVAL_MS;
        new_pattern.is_valid = true;
        
        // Вычисляем статистику
        float sum_velocity = 0.0f;
        float max_force = 0.0f;
        for (int i = 0; i < new_pattern_samples; i++) {
            if (i > 0) {
                float velocity = fabs(new_pattern.shaft_z_profile[i] - new_pattern.shaft_z_profile[i-1]);
                sum_velocity += velocity;
            }
            if (fabs(new_pattern.motor_force_profile[i]) > max_force) {
                max_force = fabs(new_pattern.motor_force_profile[i]);
            }
        }
        new_pattern.avg_velocity = sum_velocity / (new_pattern_samples - 1);
        new_pattern.max_motor_force = max_force;
        
        // Добавляем паттерн в базу
        addNewPattern(new_pattern, new_pattern.bladeSide, new_pattern.is_forward);
    }
    
    recording_pattern = false;
    currentStroke.is_active = false;
}

float StrokePredictorOne::calculatePatternSimilarity(const StrokePattern& pattern1, const StrokePattern& pattern2) {
    // Вычисляем корреляцию для shaft_z профилей
    float shaft_z_correlation = 0.0f;
    float blade_z_correlation = 0.0f;
    
    int min_length = min(pattern1.duration_ms, pattern2.duration_ms) / PATTERN_SAMPLE_INTERVAL_MS;
    
    if (min_length > 5) {
        // Простая корреляция
        float sum1_shaft = 0.0f, sum2_shaft = 0.0f;
        float sum1_blade = 0.0f, sum2_blade = 0.0f;
        
        for (int i = 0; i < min_length; i++) {
            // Масштабируем индексы для разных длин паттернов
            int idx1 = i * (pattern1.duration_ms / PATTERN_SAMPLE_INTERVAL_MS) / min_length;
            int idx2 = i * (pattern2.duration_ms / PATTERN_SAMPLE_INTERVAL_MS) / min_length;
            
            sum1_shaft += pattern1.shaft_z_profile[idx1] * pattern2.shaft_z_profile[idx2];
            sum2_shaft += pattern1.shaft_z_profile[idx1] * pattern1.shaft_z_profile[idx1] + 
                         pattern2.shaft_z_profile[idx2] * pattern2.shaft_z_profile[idx2];
            
            sum1_blade += pattern1.blade_z_profile[idx1] * pattern2.blade_z_profile[idx2];
            sum2_blade += pattern1.blade_z_profile[idx1] * pattern1.blade_z_profile[idx1] + 
                         pattern2.blade_z_profile[idx2] * pattern2.blade_z_profile[idx2];
        }
        
        if (sum2_shaft > 0) shaft_z_correlation = sum1_shaft / sqrt(sum2_shaft);
        if (sum2_blade > 0) blade_z_correlation = sum1_blade / sqrt(sum2_blade);
    }
    
    // Комбинируем корреляции
    return SHAFT_Z_SIMILARITY_WEIGHT * fabs(shaft_z_correlation) + 
           BLADE_Z_SIMILARITY_WEIGHT * fabs(blade_z_correlation);
}

StrokePattern* StrokePredictorOne::findBestMatchingPattern(BladeSideType bladeSide, bool forward, float& confidence) {
    uint8_t direction_idx = getDirectionIndex(bladeSide, forward);
    
    StrokePattern* best_pattern = nullptr;
    float best_similarity = 0.0f;
    
    for (int i = 0; i < pattern_count[direction_idx]; i++) {
        if (patterns[direction_idx][i].is_valid) {
            // Создаем временный паттерн из текущих данных для сравнения
            StrokePattern temp_pattern;
            temp_pattern.bladeSide = bladeSide;
            temp_pattern.is_forward = forward;
            
            // Заполняем первые несколько семплов из буфера
            int samples_to_compare = min(10, (int)new_pattern_samples); // Сравниваем первые 10 семплов
            if (samples_to_compare >= 5) {
                for (int j = 0; j < samples_to_compare; j++) {
                    temp_pattern.shaft_z_profile[j] = new_pattern.shaft_z_profile[j];
                    temp_pattern.blade_z_profile[j] = new_pattern.blade_z_profile[j];
                }
                temp_pattern.duration_ms = samples_to_compare * PATTERN_SAMPLE_INTERVAL_MS;
                
                float similarity = calculatePatternSimilarity(temp_pattern, patterns[direction_idx][i]);
                
                if (similarity > best_similarity) {
                    best_similarity = similarity;
                    best_pattern = &patterns[direction_idx][i];
                }
            }
        }
    }
    
    confidence = best_similarity;
    return (best_similarity >= SIMILARITY_THRESHOLD) ? best_pattern : nullptr;
}

void StrokePredictorOne::addNewPattern(const StrokePattern& pattern, BladeSideType bladeSide, bool forward) {
    uint8_t direction_idx = getDirectionIndex(bladeSide, forward);
    
    if (pattern_count[direction_idx] < MAX_PATTERNS_PER_DIRECTION) {
        // Есть свободное место
        patterns[direction_idx][pattern_count[direction_idx]] = pattern;
        pattern_count[direction_idx]++;
    } else {
        // Нужно заменить самый старый малоиспользуемый
        removeOldestLeastUsedPattern(direction_idx);
        patterns[direction_idx][pattern_count[direction_idx] - 1] = pattern;
    }
}

void StrokePredictorOne::removeOldestLeastUsedPattern(uint8_t direction_index) {
    uint32_t min_usage = UINT32_MAX;
    uint32_t oldest_time = UINT32_MAX;
    int remove_idx = 0;
    
    for (int i = 0; i < pattern_count[direction_index]; i++) {
        if (patterns[direction_index][i].usage_count < min_usage || 
            (patterns[direction_index][i].usage_count == min_usage && 
             patterns[direction_index][i].timestamp < oldest_time)) {
            min_usage = patterns[direction_index][i].usage_count;
            oldest_time = patterns[direction_index][i].timestamp;
            remove_idx = i;
        }
    }
    
    // Сдвигаем массив
    for (int i = remove_idx; i < pattern_count[direction_index] - 1; i++) {
        patterns[direction_index][i] = patterns[direction_index][i + 1];
    }
}

float StrokePredictorOne::predictForceFromPattern(const StrokePattern& pattern, uint32_t target_time_in_stroke) {
    // Вычисляем индекс в паттерне с учетом времени
    float pattern_progress = (float)target_time_in_stroke / pattern.duration_ms;
    
    // Ограничиваем прогресс
    if (pattern_progress > 1.0f) pattern_progress = 1.0f;
    if (pattern_progress < 0.0f) pattern_progress = 0.0f;
    
    // Вычисляем индекс в массиве профиля
    float float_index = pattern_progress * (pattern.duration_ms / PATTERN_SAMPLE_INTERVAL_MS - 1);
    int index = (int)float_index;
    float fraction = float_index - index;
    
    // Интерполируем значение
    if (index >= PATTERN_LENGTH - 1) {
        return pattern.motor_force_profile[PATTERN_LENGTH - 1];
    } else {
        return pattern.motor_force_profile[index] * (1.0f - fraction) + 
               pattern.motor_force_profile[index + 1] * fraction;
    }
}

uint8_t StrokePredictorOne::getDirectionIndex(BladeSideType bladeSide, bool forward) {
    if (bladeSide == BladeSideType::LEFT_BLADE) {
        return forward ? 0 : 2; // left forward : left reverse
    } else {
        return forward ? 1 : 3; // right forward : right reverse
    }
}

bool StrokePredictorOne::isForwardStroke(float current_force) {
    return current_force > 0.0f;
}

void StrokePredictorOne::resetPattern(BladeSideType bladeSide, bool forward) {
    uint8_t direction_idx = getDirectionIndex(bladeSide, forward);
    pattern_count[direction_idx] = 0;
    
    for (int i = 0; i < MAX_PATTERNS_PER_DIRECTION; i++) {
        patterns[direction_idx][i].is_valid = false;
        patterns[direction_idx][i].usage_count = 0;
    }
}