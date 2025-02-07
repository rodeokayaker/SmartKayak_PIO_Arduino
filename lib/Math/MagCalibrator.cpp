#include "MagCalibrator.h"
#include <ArduinoEigenDense.h>

namespace SP_Filters {

void AdaptiveMagCalibrator::update(
    const Eigen::Vector3f& mag_raw,
    const Eigen::Vector3f& gyro,
    const Eigen::Vector3f& acc
) {
    // Проверяем движение
    float motion = gyro.norm() + (acc.norm() - 9.81f);
    if (motion < config.motion_threshold) {
        return; // Пропускаем статические измерения
    }

    // Добавляем новый образец
    Sample new_sample;
    new_sample.mag = mag_raw;
    new_sample.weight = 1.0f;
    new_sample.is_valid = true;

    // Обновляем веса существующих образцов
    for (auto& sample : samples) {
        updateSampleWeight(sample, gyro);
    }

    // Добавляем новый образец в буфер
    samples.push_back(new_sample);
    sample_count++;

    // Удаляем старые образцы
    removeOldSamples();

    // Проверяем распределение образцов
    if (checkSampleDistribution()) {
        // Обновляем калибровку через фильтр Калмана
        updateKalmanFilter(new_sample);
        
        // Обновляем качество калибровки
        current_params.quality = calculateCalibrationQuality();
        
        is_calibrated = sample_count >= config.min_samples &&
                       current_params.quality > 0.8f;
    }
}

void AdaptiveMagCalibrator::updateSampleWeight(
    Sample& sample,
    const Eigen::Vector3f& gyro
) {
    // Уменьшаем вес старых образцов
    sample.weight *= (1.0f - config.convergence_rate);
    
    // Учитываем скорость движения
    float gyro_magnitude = gyro.norm();
    if (gyro_magnitude > 1.0f) {
        sample.weight *= (1.0f - gyro_magnitude * 0.01f);
    }
    
    // Помечаем образец как недействительный при малом весе
    if (sample.weight < 0.1f) {
        sample.is_valid = false;
    }
}

bool AdaptiveMagCalibrator::checkSampleDistribution() const {
    if (samples.size() < config.min_samples) {
        return false;
    }

    // Проверяем распределение точек в пространстве
    float min_values[3]={1e6f, 1e6f, 1e6f};
    float max_values[3]={-1e6f, -1e6f, -1e6f};
    
    for (const auto& sample : samples) {
        if (!sample.is_valid) continue;
        
        for (int i = 0; i < 3; i++) {
            min_values[i] = std::min(min_values[i], sample.mag[i]);
            max_values[i] = std::max(max_values[i], sample.mag[i]);
        }
    }
    
    // Проверяем диапазон по каждой оси
    float min_range = min({max_values[0]-min_values[0], max_values[1]-min_values[1], max_values[2]-min_values[2]});
    float max_range = max({max_values[0]-min_values[0], max_values[1]-min_values[1], max_values[2]-min_values[2]});

    
    return (min_range > max_range * 0.3f); // Требуем хотя бы 30% покрытия
}

void AdaptiveMagCalibrator::updateKalmanFilter(const Sample& sample) {
    // Состояние: [offset_x, offset_y, offset_z, scale_x, scale_y, scale_z, 
    //             soft_iron_11, soft_iron_12, soft_iron_13, 
    //             soft_iron_21, soft_iron_22, soft_iron_23,
    //             soft_iron_31, soft_iron_32, soft_iron_33]
    
    // Предсказание
    // P = P + Q, где Q - матрица шума процесса
    P = P + Q;
    
    // Измерение
    Eigen::Vector3f mag_corrected = current_params.soft_iron * 
        ((sample.mag - current_params.offset).cwiseProduct(current_params.scale));
    
    float expected_magnitude = 1.0f; // Ожидаемая величина поля
    Eigen::Vector3f innovation = mag_corrected.normalized() - 
                               Eigen::Vector3f(expected_magnitude, 0.0f, 0.0f);
    
    // Вычисляем якобиан измерения H
    Eigen::MatrixXf H = computeMeasurementJacobian(sample.mag, current_params);
    
    // Вычисляем ковариацию инновации
    Eigen::MatrixXf S = H * P * H.transpose() + R;
    
    // Вычисляем усиление Калмана
    Eigen::MatrixXf K = P * H.transpose() * S.inverse();
    
    // Обновляем состояние
    Eigen::VectorXf state_correction = K * innovation;
    
    // Применяем коррекцию к параметрам
    current_params.offset += state_correction.segment<3>(0);
    current_params.scale += state_correction.segment<3>(3);
    
    // Обновляем матрицу soft iron
    Eigen::Map<Eigen::Matrix3f> soft_iron_update(state_correction.segment<9>(6).data());
    current_params.soft_iron += soft_iron_update;
    
    // Обновляем ковариацию
    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(15, 15);
    P = (I - K * H) * P;
    
    // Нормализуем матрицу soft iron для предотвращения дрейфа
    current_params.soft_iron = current_params.soft_iron * 
        std::pow(current_params.soft_iron.determinant(), -1.0f/3.0f);
}

Eigen::MatrixXf AdaptiveMagCalibrator::computeMeasurementJacobian(
    const Eigen::Vector3f& mag,
    const CalibrationParams& params) {
    
    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(3, 15);
    
    // Вычисляем частные производные по offset, scale и soft iron
    Eigen::Vector3f mag_centered = mag - params.offset;
    Eigen::Vector3f mag_scaled = mag_centered.cwiseProduct(params.scale);
    
    // Производные по offset
    H.block<3,3>(0,0) = -params.soft_iron * params.scale.asDiagonal();
    
    // Производные по scale
    H.block<3,3>(0,3) = params.soft_iron * mag_centered.asDiagonal();
    
    // Производные по soft iron
    for(int i = 0; i < 3; i++) {
        H.block<3,3>(0, 6+i*3) = mag_scaled[i] * Eigen::Matrix3f::Identity();
    }
    
    return H;
}

float AdaptiveMagCalibrator::calculateCalibrationQuality() const {
    float total_error = 0.0f;
    int valid_count = 0;
    
    for (const auto& sample : samples) {
        if (!sample.is_valid) continue;
        
        // Применяем текущую калибровку
        Eigen::Vector3f mag_corrected = current_params.soft_iron * 
            ((sample.mag - current_params.offset).cwiseProduct(current_params.scale));
        
        // Вычисляем ошибку величины поля
        float magnitude_error = std::abs(mag_corrected.norm() - 1.0f);
        total_error += magnitude_error * sample.weight;
        valid_count++;
    }
    
    if (valid_count == 0) return 0.0f;
    
    return 1.0f - std::min(1.0f, total_error / valid_count);
}

void AdaptiveMagCalibrator::initProcessNoiseMatrix() {
    // Размерность: 15x15 для [offset(3), scale(3), soft_iron(9)]
    Q = Eigen::MatrixXf::Zero(15, 15);
    
    // Параметры шума процесса для разных компонент
    const float offset_noise = 1e-6f;  // Шум для смещения
    const float scale_noise = 1e-7f;   // Шум для масштаба
    const float soft_iron_noise = 1e-8f; // Шум для soft iron
    
    // Заполняем диагональные блоки
    // Блок для offset (3x3)
    Q.block<3,3>(0,0) = offset_noise * Eigen::Matrix3f::Identity();
    
    // Блок для scale (3x3)
    Q.block<3,3>(3,3) = scale_noise * Eigen::Matrix3f::Identity();
    
    // Блок для soft iron (9x9)
    Q.block<9,9>(6,6) = soft_iron_noise * Eigen::Matrix<float,9,9>::Identity();
    
    // Можно добавить корреляцию между параметрами, заполнив недиагональные элементы
    // Например, корреляция между scale и soft iron:
    float scale_soft_iron_correlation = 1e-8f;
    Q.block<3,9>(3,6) = scale_soft_iron_correlation * Eigen::Matrix<float,3,9>::Ones();
    Q.block<9,3>(6,3) = scale_soft_iron_correlation * Eigen::Matrix<float,9,3>::Ones();
}

} // namespace SP_Filters 