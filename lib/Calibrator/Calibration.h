#ifndef CALIBRATION_H
#define CALIBRATION_H

#ifndef INITIAL_NUM_SAMPLES
#define INITIAL_NUM_SAMPLES 1000
#endif


#include <ArduinoEigenDense.h>

class Calibration {
public:
    Calibration();
    
    void performInitialCalibration(Eigen::Vector3f* magSamples, int numSmpl);
    void performAdaptiveCalibration(const float* magEvent, 
                                    const float* accEvent,
                                    const float* gyrEvent,
                                    const float* tempEvent,
                                    const Eigen::Quaternionf &dmpQuat);
    
    Eigen::Vector3f getCalibratedData(float x, float y, float z);
    bool saveCalibration(char* name);
    bool loadCalibration(char* name);

    inline Eigen::Vector3f getOffset() const {
    return offset;
    }

    inline Eigen::Vector3f getScale() const {
    return scale;
    }

    inline Eigen::Matrix3f getSoftIronMatrix() const {
    return softIronMatrix;
    }

    inline Eigen::Matrix3f getCalibrationMatrix() const {
    return calibrationMatrix;
    }

    void startEKF(const Eigen::Vector3f &magData,
                   const Eigen::Quaternionf &dmpQuat);
                   
    void updateEKF(const Eigen::Vector3f &magData,
                   const Eigen::Quaternionf &dmpQuat);

    void updateCalibrationMatrixFromW();


    void setCalibrationMatrix(const Eigen::Vector3f &offset, const Eigen::Matrix3f &calibrationMatrix);
    void setCalibrationMatrix(float* offset, float* scale, float* SI);

    // Методы для упрощенного фильтра Калмана
    void startSmallEKF(const Eigen::Vector3f &magData, const Eigen::Quaternionf &dmpQuat);
    void updateSmallEKF(const Eigen::Vector3f &magData, const Eigen::Quaternionf &dmpQuat);
    void updateOffsetFromSmallEKF();

private:
    
    Eigen::Vector3f offset;
    Eigen::Vector3f scale;
    Eigen::Matrix3f softIronMatrix;
    Eigen::Matrix3f calibrationMatrix;
    
    Eigen::Matrix3f W;
    Eigen::Vector3f V;
    Eigen::Matrix<float, 12, 1> state;
    Eigen::Matrix<float, 12, 12> errorCovariance;
    Eigen::Matrix<float, 12, 12> processNoise;
    Eigen::Matrix<float, 3, 3> measurementNoise;
    Eigen::Matrix<float, 3, 3> R;

    Eigen::Matrix<float, 6, 1> state_small;
    Eigen::Matrix<float, 6, 6> errorCovariance_small;
    Eigen::Matrix<float, 6, 6> processNoise_small;
    Eigen::Matrix<float, 3, 3> measurementNoise_small;
    
    void performEllipsoidFitting(Eigen::Vector3f* magSamples, int numSmpl);
    void performRANSAC(Eigen::Vector3f* magSamples, int numSmpl, int numIterations=100, float threshold=0.1);
    void performGeometricCalibration(Eigen::Vector3f* magSamples, int numSmpl, int numIterations=100, float threshold=0.1, float tolerance=1e-2);
    void performFullGeometricCalibration(Eigen::Vector3f* magSamples, int numSmpl, int numIterations=100, float threshold=0.1, float tolerance=1e-2);
    void performMinMaxCalibration(Eigen::Vector3f* magSamples, int numSmpl);
    void updateCalibrationMatrix();


    
    bool isValidData(const float* mag, const float* acc, const float* gyr);
    


    /**
     * @brief Выполняет калибровку магнитометра с использованием кватернионов ориентации
     * @param magSamples Массив измерений магнитометра
     * @param quatSamples Массив кватернионов ориентации
     * @param numSmpl Количество измерений
     * @param numIterations Количество итераций оптимизации
     * @param threshold Порог для отбрасывания выбросов
     * @param tolerance Допуск сходимости
     */
    void performQuaternionCalibration(Eigen::Vector3f* magSamples, 
                                    Eigen::Quaternionf* quatSamples, 
                                    int numSmpl, 
                                    int numIterations = 50,
                                    float threshold = 0.2f,
                                    float tolerance = 1e-4f);

    /**
     * @brief Быстрая калибровка магнитометра без смещения (только масштаб и soft iron)
     * @param magSamples Массив измерений магнитометра
     * @param quatSamples Массив кватернионов ориентации
     * @param numSmpl Количество измерений
     * @return true если калибровка успешна
     */
    bool performFastCalibration(Eigen::Vector3f* magSamples, 
                              Eigen::Quaternionf* quatSamples, 
                              int numSmpl);

    /**
     * @brief Калибровка магнитометра на основе параллельности векторов в глобальной системе координат
     * @param magSamples Массив измерений магнитометра
     * @param quatSamples Массив кватернионов ориентации
     * @param numSmpl Количество измерений
     * @param numIterations Количество итераций оптимизации
     * @return true если калибровка успешна
     */
    bool calibrateForParallelVectors(Eigen::Vector3f* magSamples, 
                                   Eigen::Quaternionf* quatSamples, 
                                   int numSmpl,
                                   int numIterations = 50);
};

#endif 