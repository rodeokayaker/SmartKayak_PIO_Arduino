#pragma once
#include "SP_Math.h"
#include <vector>
#include <ArduinoEigenDense.h>


namespace SP_Filters {

class AdaptiveMagCalibrator {
public:
    struct CalibrationParams {
        Eigen::Vector3f offset;     // Смещение нуля
        Eigen::Vector3f scale;      // Масштабные коэффициенты
        Eigen::Matrix3f soft_iron; // Матрица искажений мягкого железа
        float quality;              // Качество калибровки [0-1]
    };

    struct Config {
        int min_samples = 200;      // Минимум точек для калибровки
        float motion_threshold = 0.1f; // Порог определения движения
        float convergence_rate = 0.01f; // Скорость сходимости
        int buffer_size = 1000;     // Размер буфера измерений
    };

    AdaptiveMagCalibrator(const Config& config);

    // Обновление калибровки в реальном времени
    void update(
        const Eigen::Vector3f& mag_raw,
        const Eigen::Vector3f& gyro,
        const Eigen::Vector3f& acc
    );

    // Получение текущих параметров калибровки
    CalibrationParams getCalibration() const;
    
    // Проверка готовности калибровки
    bool isCalibrated() const;
    
    // Сброс калибровки
    void reset();

private:
    Config config;
    CalibrationParams current_params;
    
    struct Sample {
        Eigen::Vector3f mag;
        float weight;
        bool is_valid;
    };
    
    std::vector<Sample> samples;
    size_t sample_count = 0;
    bool is_calibrated = false;

    // Матрицы фильтра Калмана
    Eigen::Matrix<float, 12, 12> P; // Ковариация ошибки
    Eigen::Matrix<float, 12, 12> Q; // Шум процесса
    Eigen::Matrix<float, 3, 3> R;   // Шум измерений

    void updateKalmanFilter(const Sample& sample);
    void updateSampleWeight(Sample& sample, const Eigen::Vector3f& gyro);
    bool checkSampleDistribution() const;
    void removeOldSamples();
    float calculateCalibrationQuality() const;
    Eigen::MatrixXf computeMeasurementJacobian(const Eigen::Vector3f& mag, const CalibrationParams& params);
    void initProcessNoiseMatrix();
};

} // namespace SP_Filters 