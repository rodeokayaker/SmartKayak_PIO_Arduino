#include <ArduinoEigenDense.h>
//#include <Eigen/Geometry>

#define ORIENTATION_ESTIMATOR_HISTORY_SIZE 10

class OrientationEstimator {
private:
    // Параметры фильтра
    float alpha;                // Коэффициент фильтрации
    float maxDriftRate;         // Максимальный дрейф в радианах в секунду
    uint16_t updateFrequency;      // Частота обновления в Гц
    float magVariance;          // Дисперсия шума магнитометра
    
    // Состояние фильтра
    float driftAngle;           // Текущий угол дрейфа
    float lastMagAngle;         // Последний валидный угол по магнитометру
    uint32_t lastMagUpdateTime; // Время последнего обновления по магнитометру
    float magReliability;       // Надежность магнитометра (0-1)
    
    // Добавляем новые поля для отслеживания длины вектора
    float lastMagNorm;          // Последняя норма вектора магнитометра
    float magNormHistory[ORIENTATION_ESTIMATOR_HISTORY_SIZE];    // История норм вектора
    int magNormIndex;           // Индекс в истории
    bool magNormHistoryFilled;  // Флаг заполнения истории
    int validTimer;
    bool firstRun;
    // Константы
    static constexpr float _PI = 3.14159265359f;
    static constexpr float MAX_DRIFT_PER_MINUTE = _PI/4.0f; // π/4 радиан в минуту
    static constexpr float MAG_RELIABILITY_DECAY = 0.1f;   // Скорость уменьшения надежности
    static constexpr float MAG_RELIABILITY_GAIN = 0.005f;   // Скорость увеличения надежности
    static constexpr float MAG_ANGLE_THRESHOLD = _PI/6.0f;  // Порог отклонения угла магнитометра
    
    // Константы для проверки стабильности
    static constexpr float MAG_NORM_TOLERANCE = 0.1f;  // Допустимое отклонение нормы
    static constexpr float MAG_NORM_SMOOTHING = 0.3f;  // Коэффициент сглаживания
    
    // Константы для проверки вертикальности
    static constexpr float VERTICAL_THRESHOLD = 0.97f;  // Косинус угла с вертикалью (примерно 25 градусов)
    
public:
    OrientationEstimator(uint16_t frequency = 100, float variance = 0.01f) 
        : updateFrequency(frequency)
        , magVariance(variance)
        , driftAngle(0.0f)
        , lastMagAngle(0.0f)
        , lastMagUpdateTime(0)
        , magReliability(0.0f)
        , lastMagNorm(0.0f)
        , magNormIndex(0)
        , magNormHistoryFilled(false)
        , validTimer(100),
        firstRun(true)
    {
        // Рассчитываем максимальный дрейф в секунду
        maxDriftRate = MAX_DRIFT_PER_MINUTE / 60.0f;
        
        // Рассчитываем alpha на основе частоты обновления и дисперсии
        float dt = 1.0f / frequency;
        float processNoise = maxDriftRate * dt;
        alpha = (processNoise / (processNoise + magVariance))/100;
        
        // Инициализируем историю норм нулями
        for (int i = 0; i < ORIENTATION_ESTIMATOR_HISTORY_SIZE; i++) {
            magNormHistory[i] = 0.0f;
        }
    }

    void setFrequency(uint16_t frequency) {
        updateFrequency = frequency;
        float dt = 1.0f / frequency;
        float processNoise = maxDriftRate * dt;
        alpha = (processNoise / (processNoise + magVariance))/100;
    }

    void printData() {
        Serial.printf("Alpha: %f, MaxDriftRate: %f, MagVariance: %f, \nDriftAngle: %f, LastMagAngle: %f, LastMagUpdateTime: %d, \nMagReliability: %f, LastMagNorm: %f, MagNormIndex: %d, MagNormHistoryFilled: %d\n", alpha, maxDriftRate, magVariance, driftAngle, lastMagAngle, lastMagUpdateTime, magReliability, lastMagNorm, magNormIndex, magNormHistoryFilled);
        Serial.flush();
    }
    
    void update(const Eigen::Quaternionf& dmpQuat, const Eigen::Vector3f& magData) {
        // Вычисляем текущее время
        uint32_t currentTime = millis();
        float dt = (currentTime - lastMagUpdateTime) / 1000.0f;

        
        // Рассчитываем текущий максимальный дрейф с учетом надежности
        float timeFactor = std::min(1.0f + dt * 0.1f, 3.0f); // Увеличиваем дрейф со временем, но не более чем в 3 раза
        float currentMaxDriftRate = maxDriftRate * (1.0f + 2.0f * (1.0f - magReliability)) * timeFactor;
        
        // Преобразуем вектор магнитного поля в глобальную систему координат
        Eigen::Vector3f globalMag = dmpQuat._transformVector(magData);

//        Serial.printf("magData: [%.2f, %.2f, %.2f] ", magData.x(), magData.y(), magData.z());        
//        Serial.printf("globalMag: [%.2f, %.2f, %.2f]\n", globalMag.x(), globalMag.y(), globalMag.z());        
        // Проекция на глобальную горизонтальную плоскость (XY)
        Eigen::Vector3f horizontalMag = globalMag;
        horizontalMag.z() = 0.0f;  // Обнуляем вертикальную составляющую
        
        // Проверяем стабильность длины вектора и его вертикальность
        float currentMagNorm = magData.norm();
        bool isNormStable = isMagNormStable(currentMagNorm);
        bool isNotVertical = isMagNotVertical(globalMag);  // Проверяем в глобальной СК
        float currentMagAngle = atan2(horizontalMag.y(), horizontalMag.x());
//        Serial.printf("DriftAngle: %f, currentMagAngle: %f\n", driftAngle, currentMagAngle);
        
        if (horizontalMag.norm() > 0.001f && isNormStable && isNotVertical) {

//            Serial.print("ok1\n");
            // Нормализуем проекцию
            horizontalMag.normalize();
            
            // Вычисляем угол в глобальной системе координат
            float currentMagAngle = atan2(horizontalMag.y(), horizontalMag.x());
            
            // Проверяем достоверность данных магнитометра
            if (isMagDataReliable(currentMagAngle)) {


                // Обновляем надежность магнитометра
                updateMagReliability(true);
                
                // Вычисляем разницу углов
                float angleDiff = normalizeAngle(currentMagAngle - driftAngle);
                
                // Применяем комплиментарный фильтр
                if (firstRun){
                    driftAngle = currentMagAngle;
                    firstRun = false;
                } else {
                    driftAngle += angleDiff * alpha * magReliability;
                }
                
                // Запоминаем последний валидный угол
                lastMagAngle = currentMagAngle;
                lastMagUpdateTime = currentTime;
            } else {
                // Уменьшаем надежность при недостоверных данных
                updateMagReliability(false);
            }
        } else {
            // Если вектор нестабилен или слишком вертикален, резко уменьшаем надежность
  /*          if (horizontalMag.norm() <= 0.001f){
                Serial.printf("horizontalMag.norm() > 0.001f\n");
            }
            if (!isNotVertical){
                Serial.printf("!isNotVertical\n");
            }
            if (!isNormStable){
                Serial.printf("!isNormStable\n");
            }*/
            updateMagReliability(false);
            magReliability *= 0.5f; // Дополнительное уменьшение надежности
        }
        
        // Обновляем историю норм
        updateMagNormHistory(currentMagNorm);
        
        // Ограничиваем дрейф
        driftAngle = normalizeAngle(driftAngle);
    }

    Eigen::Quaternionf applyCorrection(const Eigen::Quaternionf& dmpQuat) const {
        return dmpQuat * Eigen::Quaternionf(Eigen::AngleAxisf(-driftAngle, Eigen::Vector3f::UnitZ()));
    }

    
    
    float getDriftAngle() const {
        return driftAngle;
    }
    
    float getMagReliability() const {
        return magReliability;
    }


    void reset(){
        driftAngle = 0.0f;
        lastMagAngle = 0.0f;
        lastMagUpdateTime = 0;
        magReliability = 0.0f;
        lastMagNorm = 0.0f;
        magNormIndex = 0;
        magNormHistoryFilled = false;
        validTimer = 100;
        firstRun = true;
        for (int i = 0; i < ORIENTATION_ESTIMATOR_HISTORY_SIZE; i++) {
            magNormHistory[i] = 0.0f;
        }
        magVariance = 0.01f;
        maxDriftRate = MAX_DRIFT_PER_MINUTE / 60.0f;
        float dt = 1.0f / updateFrequency;
        float processNoise = maxDriftRate * dt;
        alpha = (processNoise / (processNoise + magVariance))/100;
    }
    
private:
    bool isMagDataReliable(float currentMagAngle) {
        // Вычисляем разницу с последним валидным углом
        if (validTimer>0){
            validTimer --;
            return true;
        }
        if (lastMagUpdateTime == 0){
            return true;
        }
        
        // Вычисляем время с последнего обновления в секундах
        float dt = (millis() - lastMagUpdateTime) / 1000.0f;
        
        // Увеличиваем порог в зависимости от времени и надежности
        float adjustedThreshold = MAG_ANGLE_THRESHOLD * (1.0f + dt * 0.1f) * (1.0f + 2.0f * (1.0f - magReliability));
        
        float angleDiff = normalizeAngle(currentMagAngle - lastMagAngle);
        
        // Если разница слишком большая, данные считаем недостоверными
        return fabs(angleDiff) < adjustedThreshold;
    }
    
    void updateMagReliability(bool isReliable) {
        if (isReliable) {
            // Постепенно увеличиваем надежность при достоверных данных
            magReliability = std::min(1.0f, magReliability + MAG_RELIABILITY_GAIN);
        } else {
            // Постепенно уменьшаем надежность при недостоверных данных
            magReliability = std::max(0.0f, magReliability - MAG_RELIABILITY_DECAY);
        }
    }
    
    float normalizeAngle(float angle) {
        // Нормализуем угол в диапазон [-π, π]
        while (angle > _PI) angle -= 2.0f * _PI;
        while (angle < -_PI) angle += 2.0f * _PI;
        return angle;
    }
    
    bool isMagNormStable(float currentNorm) {
        if (!magNormHistoryFilled) {
            return true; // Пока история не заполнена, считаем данные стабильными
        }
        
        // Вычисляем среднюю норму из истории
        float avgNorm = 0.0f;
        for (int i = 0; i < ORIENTATION_ESTIMATOR_HISTORY_SIZE; i++) {
            avgNorm += magNormHistory[i];
        }
        avgNorm /= ORIENTATION_ESTIMATOR_HISTORY_SIZE;
        
        // Проверяем отклонение от средней нормы
        float normDeviation = fabs(currentNorm - avgNorm);
//        Serial.printf("currnetNorm: %f, avgNorm: %f, normDeviation: %f\n", currentNorm, avgNorm, normDeviation);
        return normDeviation < MAG_NORM_TOLERANCE*avgNorm;
    }
    
    void updateMagNormHistory(float currentNorm) {
        // Сглаживаем новое значение
        if (lastMagNorm == 0.0f) {
            lastMagNorm = currentNorm;
        } else {
            lastMagNorm = lastMagNorm * (1.0f - MAG_NORM_SMOOTHING) + 
                         currentNorm * MAG_NORM_SMOOTHING;
        }
        
        // Добавляем в историю
        magNormHistory[magNormIndex] = lastMagNorm;
        magNormIndex = (magNormIndex + 1) % ORIENTATION_ESTIMATOR_HISTORY_SIZE;
        
        if (!magNormHistoryFilled && magNormIndex == 0) {
            magNormHistoryFilled = true;
        }
    }
    
    bool isMagNotVertical(const Eigen::Vector3f& magData) {
        // Нормализуем вектор
        Eigen::Vector3f normMag = magData.normalized();
        
        // Вычисляем косинус угла с вертикалью (ось Z)
        float cosAngle = fabs(normMag.z());
        
        // Если угол слишком мал (вектор слишком вертикален), данные ненадежны
        return cosAngle < VERTICAL_THRESHOLD;
    }

};