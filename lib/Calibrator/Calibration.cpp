#include "Calibration.h"
#include <vector>
#include "Preferences.h"
#include <Arduino.h>



Calibration::Calibration() 
    : 
      offset(Eigen::Vector3f::Zero()), 
      scale(Eigen::Vector3f::Ones()),
      softIronMatrix(Eigen::Matrix3f::Identity()),
      calibrationMatrix(Eigen::Matrix3f::Identity()),
      state(Eigen::Matrix<float, 12, 1>::Zero()),
      errorCovariance(Eigen::Matrix<float, 12, 12>::Zero()),
      processNoise(Eigen::Matrix<float, 12, 12>::Identity() * 1e-5f),
      measurementNoise(Eigen::Matrix<float, 3, 3>::Identity() * 50.7),
      R(Eigen::Matrix<float, 3, 3>::Identity()) {
          
    // Инициализация состояния
    state << 1.0f,0.0f,0.0f,1.0f,1.0f,1.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f;
    
    // Инициализация матрицы ковариации ошибок
    // Для магнитного поля (первые 3 элемента)
    errorCovariance.block<3,3>(0,0) = Eigen::Matrix3f::Identity() * 10.0f;
    
    // Для диагональных элементов W (следующие 3 элемента)
    errorCovariance.block<3,3>(3,3) = Eigen::Matrix3f::Identity() * 0.1f;
    
    // Для недиагональных элементов W (следующие 3 элемента)
    errorCovariance.block<3,3>(6,6) = Eigen::Matrix3f::Identity() * 0.01f;
    
    // Для вектора смещения V (последние 3 элемента)
    errorCovariance.block<3,3>(9,9) = Eigen::Matrix3f::Identity() * 1.0f;

    // Инициализация шума процесса
    Eigen::Matrix3f defaultDeltaRCov;
    defaultDeltaRCov << 1.2e-8f, 3.0e-3f, 1.4e-3f,
                           3.0e-3f, 4.0e-8f, 2.5e-3f,
                           1.4e-3f, 2.5e-3f, 6.5e-8f;
    defaultDeltaRCov *= 10;
        
    Eigen::Matrix<float, 12, 12> Q = Eigen::Matrix<float, 12, 12>::Zero();
    Q.block<3,3>(0,0) = defaultDeltaRCov;
    
    processNoise = errorCovariance;

    measurementNoise(0,0) = 0.99f; // X
    measurementNoise(1,1) = 0.50f; // Y
    measurementNoise(2,2) = 0.56f; // Z
}

Eigen::Matrix3f Inverse(const Eigen::Matrix3f &matrix) {
    float det = matrix(0,0)*(matrix(1,1)*matrix(2,2) - matrix(1,2)*matrix(2,1)) -
                matrix(0,1)*(matrix(1,0)*matrix(2,2) - matrix(1,2)*matrix(2,0)) +
                matrix(0,2)*(matrix(1,0)*matrix(2,1) - matrix(1,1)*matrix(2,0));
    if (fabs(det) < 1e-6f) {
        return Eigen::Matrix3f::Identity();
    }
    float invDet = 1.0f / det;
    Eigen::Matrix3f invMatrix;
    invMatrix(0,0) = (matrix(1,1)*matrix(2,2) - matrix(1,2)*matrix(2,1)) * invDet;
    invMatrix(0,1) = (matrix(0,2)*matrix(2,1) - matrix(0,1)*matrix(2,2)) * invDet;
    invMatrix(0,2) = (matrix(0,1)*matrix(1,2) - matrix(0,2)*matrix(1,1)) * invDet;
    invMatrix(1,0) = (matrix(1,2)*matrix(2,0) - matrix(1,0)*matrix(2,2)) * invDet;
    invMatrix(1,1) = (matrix(0,0)*matrix(2,2) - matrix(0,2)*matrix(2,0)) * invDet;
    invMatrix(1,2) = (matrix(0,2)*matrix(1,0) - matrix(0,0)*matrix(1,2)) * invDet;
    invMatrix(2,0) = (matrix(1,0)*matrix(2,1) - matrix(1,1)*matrix(2,0)) * invDet;
    invMatrix(2,1) = (matrix(0,1)*matrix(2,0) - matrix(0,0)*matrix(2,1)) * invDet;
    invMatrix(2,2) = (matrix(0,0)*matrix(1,1) - matrix(0,1)*matrix(1,0)) * invDet;
    return invMatrix;
}

void Calibration::performInitialCalibration(Eigen::Vector3f* magSamples, int numSmpl) {

    Serial.println("Performing initial calibration");
    Serial.println(numSmpl);
/*    // Эллипсоидальный фитинг
    performEllipsoidFitting(magSamples, numSmpl);
    Serial.println("Ellipsoid fitting done");
    
    // RANSAC
    performRANSAC(magSamples, numSmpl);
    Serial.println("RANSAC done");

    Serial.printf("Offset: [%.2f, %.2f, %.2f]\n", offset[0], offset[1], offset[2]);
    Serial.printf("Scale: [%.2f, %.2f, %.2f]\n", scale[0], scale[1], scale[2]);
    Serial.printf("Soft iron matrix:\n%.2f %.2f %.2f\n%.2f %.2f %.2f\n%.2f %.2f %.2f\n", 
                  softIronMatrix(0,0), softIronMatrix(0,1), softIronMatrix(0,2),
                  softIronMatrix(1,0), softIronMatrix(1,1), softIronMatrix(1,2),
                  softIronMatrix(2,0), softIronMatrix(2,1), softIronMatrix(2,2));    

//    performMinMaxCalibration(magSamples, numSmpl);
    // Полная геометрическая калибровка
    performFullGeometricCalibration(magSamples, numSmpl, 50);
    Serial.println("Full geometric calibration done");
 */
    performMinMaxCalibration(magSamples, numSmpl);
    // Обновление матрицы калибровки  
    updateCalibrationMatrix();

    Serial.printf("Offset: [%.2f, %.2f, %.2f]\n", offset[0], offset[1], offset[2]);
    Serial.printf("Scale: [%.2f, %.2f, %.2f]\n", scale[0], scale[1], scale[2]);
    Serial.printf("Soft iron matrix:\n%.2f %.2f %.2f\n%.2f %.2f %.2f\n%.2f %.2f %.2f\n", 
                  softIronMatrix(0,0), softIronMatrix(0,1), softIronMatrix(0,2),
                  softIronMatrix(1,0), softIronMatrix(1,1), softIronMatrix(1,2),
                  softIronMatrix(2,0), softIronMatrix(2,1), softIronMatrix(2,2));
    Serial.printf("Calibration matrix:\n%.2f %.2f %.2f\n%.2f %.2f %.2f\n%.2f %.2f %.2f\n", 
                  calibrationMatrix(0,0), calibrationMatrix(0,1), calibrationMatrix(0,2),
                  calibrationMatrix(1,0), calibrationMatrix(1,1), calibrationMatrix(1,2),
                  calibrationMatrix(2,0), calibrationMatrix(2,1), calibrationMatrix(2,2));
    Serial.println("Calibration matrix updated");
}

void Calibration::performAdaptiveCalibration(const float* magEvent, 
                                             const float* accEvent,
                                             const float* gyrEvent,
                                             const float* tempEvent,
                                             const Eigen::Quaternionf &dmpQuat) {
                                                 
    Eigen::Vector3f magData;
    magData << magEvent[0], magEvent[1], magEvent[2];
    
    
    updateEKF(magData, dmpQuat);
}

void Calibration::startEKF(const Eigen::Vector3f &magData,
                           const Eigen::Quaternionf &dmpQuat) {
    R = dmpQuat.toRotationMatrix();
    W = Inverse(calibrationMatrix);
    V = offset;
    state.head<3>() = getCalibratedData(magData[0], magData[1], magData[2]);

    state[3] = W(0,0);
    state[4] = W(1,1);
    state[5] = W(2,2);
    state[6] = W(0,1);
    state[7] = W(0,2);
    state[8] = W(1,2);
    state.tail<3>()=V;

    // Инициализация матрицы ковариации ошибок
    // Для магнитного поля (первые 3 элемента)
    errorCovariance.block<3,3>(0,0) = Eigen::Matrix3f::Identity() * 10.0f;
    
    // Для диагональных элементов W (следующие 3 элемента)
    errorCovariance.block<3,3>(3,3) = Eigen::Matrix3f::Identity() * 1.0f;
    
    // Для недиагональных элементов W (следующие 3 элемента)
    errorCovariance.block<3,3>(6,6) = Eigen::Matrix3f::Identity() * 1.00f;
    
    // Для вектора смещения V (последние 3 элемента)
    errorCovariance.block<3,3>(9,9) = Eigen::Matrix3f::Identity() * 1.0f;

    
}

void Calibration::updateEKF(const Eigen::Vector3f &magData,
                           const Eigen::Quaternionf &dmpQuat) {

    // 1. Прогноз состояния
    // Используем кватернион DMP для предсказания изменения ориентации
    
    // Прогноз магнитного поля с учетом изменения ориентации
//    Serial.printf("State: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n", state[0], state[1], state[2], state[3], state[4], state[5], state[6], state[7], state[8], state[9], state[10], state[11]);
    Eigen::Matrix3f rotationMatrix = dmpQuat.toRotationMatrix();
    Eigen::Matrix3f deltaR =  rotationMatrix.transpose()*R;
    Eigen::Vector3f predictedMag = deltaR * state.head<3>();
 //   Serial.printf("Predicted mag: [%.2f, %.2f, %.2f]\n", predictedMag[0], predictedMag[1], predictedMag[2]);
//    Serial.printf("magData: [%.2f, %.2f, %.2f]\n", magData[0], magData[1], magData[2]);
//    Serial.printf("dmpQuat: [%.2f, %.2f, %.2f, %.2f]\n", dmpQuat.w(), dmpQuat.x(), dmpQuat.y(), dmpQuat.z());
    R = rotationMatrix;

    
    // Прогноз параметров калибровки (считаем их постоянными)
    Eigen::Matrix<float, 12, 1> predictedState(state);
    predictedState.head<3>() = predictedMag;

    // 2. Прогноз ковариации ошибки
    // Матрица перехода состояния
    Eigen::Matrix<float, 12, 12> F = Eigen::Matrix<float, 12, 12>::Identity();
    F.block<3,3>(0,0) = deltaR;
    
    // Прогноз ковариации ошибки
    Eigen::Matrix<float, 12, 12> predictedErrorCovariance = 
        F * errorCovariance * F.transpose() + processNoise;
    

    

    // 3. Матрица Якоби измерений
    Eigen::Matrix<float, 3, 12> H = Eigen::Matrix<float, 3, 12>::Zero();
    H.block<3,3>(0,0) = W; // производная по Bc

    // производные по W (симметричная матрица)
    H(0,3) = state[0]; // d/dW11
    H(1,4) = state[1]; // d/dW22
    H(2,5) = state[2]; // d/dW33

    H(0,6) = state[1]; // d/dW12
    H(1,6) = state[0];

    H(0,7) = state[2]; // d/dW13
    H(2,7) = state[0];

    H(1,8) = state[2]; // d/dW23
    H(2,8) = state[1];

    // производная по V
    H.block<3,3>(0,9) = Eigen::Matrix3f::Identity();
    
    // 4. Ожидаемое измерение магнитометра
    Eigen::Vector3f expectedMag = W * predictedMag + V;


    // 5. Калмановский коэффициент усиления

    Eigen::Matrix3f S=H * predictedErrorCovariance * H.transpose() + measurementNoise;

    Eigen::Matrix<float, 12, 3> K = predictedErrorCovariance * H.transpose() * 
        S.inverse();

/*    Serial.printf("S:\n%.2f %.2f %.2f\n%.2f %.2f %.2f\n%.2f %.2f %.2f\n", 
                  S(0,0), S(0,1), S(0,2),
                  S(1,0), S(1,1), S(1,2),
                  S(2,0), S(2,1), S(2,2));*/
/*    Serial.printf("K:\n%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", 
                  K(0,0), K(1,0), K(2,0), K(3,0), K(4,0), K(5,0), K(6,0), K(7,0), K(8,0), K(9,0), K(10,0), K(11,0), 
                  K(0,1), K(1,1), K(2,1), K(3,1), K(4,1), K(5,1), K(6,1), K(7,1), K(8,1), K(9,1), K(10,1), K(11,1), 
                  K(0,2), K(1,2), K(2,2), K(3,2), K(4,2), K(5,2), K(6,2), K(7,2), K(8,2), K(9,2), K(10,2), K(11,2));
*/

    // 6. Обновление состояния    
    Eigen::Vector3f measurementDelta = magData - expectedMag;
//    Serial.printf("Measurement delta: %.2f\n", measurementDelta.norm());
    state = predictedState + K * measurementDelta;



/*    state[3]=scale[0];
    state[4]=scale[1];
    state[5]=scale[2];
    state[6]=0.0f;
    state[7]=0.0f;
    state[8]=0.0f;*/

    // 7. Обновление ковариации ошибки
    errorCovariance = (Eigen::Matrix<float, 12, 12>::Identity() - K * H) * predictedErrorCovariance;

    // 8. Обновление параметров калибровки
    V = state.tail<3>();
    scale = state.segment<3>(3).cwiseMax(1e-6f); // Избегаем отрицательных масштабов
    W(0,0)=state[3];
    W(1,1)=state[4];
    W(2,2)=state[5];
    W(0,1)=state[6];
    W(1,0)=state[6];
    W(0,2)=state[7];
    W(2,0)=state[7];
    W(1,2)=state[8];
    W(2,1)=state[8];
    
    
    
    // Ортогонализация soft iron матрицы
//    Eigen::JacobiSVD<Eigen::Matrix3f> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
//    softIronMatrix = svd.matrixU() * svd.matrixV().transpose();

}

void Calibration::startSmallEKF(const Eigen::Vector3f &magData, const Eigen::Quaternionf &dmpQuat) {
    R = dmpQuat.toRotationMatrix();
    W = calibrationMatrix.inverse();
    V = offset;
    
    // Инициализация состояния (только смещение)
    state_small.head<3>() = getCalibratedData(magData[0], magData[1], magData[2]);
    state_small.tail<3>() = V;
    
    // Инициализация матрицы ковариации ошибок
    errorCovariance_small = Eigen::Matrix<float, 6, 6>::Identity() * 10.0f;
    
    // Инициализация шумов процесса и измерений
    processNoise_small = Eigen::Matrix<float, 6, 6>::Identity() * 0.001f;
    processNoise_small.block<3,3>(3,3) = Eigen::Matrix3f::Identity() * 0.0f;
    measurementNoise_small = Eigen::Matrix<float, 3, 3>::Identity() * 10.0f;
}

void Calibration::updateSmallEKF(const Eigen::Vector3f &magData, const Eigen::Quaternionf &dmpQuat) {
//    Serial.printf("State: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n", state_small[0], state_small[1], state_small[2], state_small[3], state_small[4], state_small[5]);
    // 1. Прогноз состояния
    Eigen::Matrix3f rotationMatrix = dmpQuat.toRotationMatrix();
    Eigen::Matrix3f deltaR = rotationMatrix.transpose() * R;
    Eigen::Vector3f predictedMag = deltaR * state_small.head<3>();
    R = rotationMatrix;
    
    // Прогноз смещения
    Eigen::Matrix<float, 6, 1> predictedState = state_small;
    predictedState.head<3>() = predictedMag;
    Eigen::Matrix<float, 6, 6> F = Eigen::Matrix<float, 6, 6>::Identity();
    F.block<3,3>(0,3) = deltaR;

    // 2. Прогноз ковариации ошибки
    Eigen::Matrix<float, 6, 6> predictedErrorCovariance = 
        F * errorCovariance_small * F.transpose() + processNoise_small;
    
    // 3. Матрица Якоби измерений
    Eigen::Matrix<float, 3, 6> H = Eigen::Matrix<float, 3, 6>::Zero();
    H.block<3,3>(0,0) = W;
    H.block<3,3>(0,3) = Eigen::Matrix<float, 3, 3>::Identity();
    
    // 4. Ожидаемое измерение магнитометра
    Eigen::Vector3f expectedMag = W * predictedMag + V;
    
    // 5. Калмановский коэффициент усиления
    Eigen::Matrix3f S = H * predictedErrorCovariance * H.transpose() + measurementNoise_small;
    Eigen::Matrix<float, 6, 3> K = predictedErrorCovariance * H.transpose() * S.inverse();
    
    // 6. Обновление состояния
    Eigen::Vector3f measurementDelta = magData - expectedMag;
    Eigen::Vector3f y = magData - H * predictedState;
//    Serial.printf("y: [%.2f, %.2f, %.2f] delta: [%.2f, %.2f, %.2f]\n", y[0], y[1], y[2], measurementDelta[0], measurementDelta[1], measurementDelta[2]);
    state_small = predictedState + K * y;
    
    // 7. Обновление ковариации ошибки
    errorCovariance_small = (Eigen::Matrix<float, 6, 6>::Identity() - K * H) * predictedErrorCovariance;

    V=state_small.tail<3>();
}

void Calibration::updateOffsetFromSmallEKF() {
    offset = state_small.tail<3>();
}

void Calibration::updateCalibrationMatrixFromW() {

    offset = V;
    scale[0]=W(0,0);
    scale[1]=W(1,1);
    scale[2]=W(2,2);
//    Serial.println("Calibration matrix updated");

    calibrationMatrix = Inverse(W);
    softIronMatrix = W;

/*    Serial.printf("Offset: [%.2f, %.2f, %.2f]\n", offset[0], offset[1], offset[2]);
    Serial.printf("Scale: [%.2f, %.2f, %.2f]\n", scale[0], scale[1], scale[2]);
    Serial.printf("Soft iron matrix:\n%.2f %.2f %.2f\n%.2f %.2f %.2f\n%.2f %.2f %.2f\n", 
                  softIronMatrix(0,0), softIronMatrix(0,1), softIronMatrix(0,2),
                  softIronMatrix(1,0), softIronMatrix(1,1), softIronMatrix(1,2),
                  softIronMatrix(2,0), softIronMatrix(2,1), softIronMatrix(2,2));
    Serial.printf("Calibration matrix:\n%.2f %.2f %.2f\n%.2f %.2f %.2f\n%.2f %.2f %.2f\n", 
                  calibrationMatrix(0,0), calibrationMatrix(0,1), calibrationMatrix(0,2),
                  calibrationMatrix(1,0), calibrationMatrix(1,1), calibrationMatrix(1,2),
                  calibrationMatrix(2,0), calibrationMatrix(2,1), calibrationMatrix(2,2));
*/
}
/*
void Calibration::updateEKF(const Eigen::Vector3f &magData,
                          const Eigen::Vector3f &accData,
                          const Eigen::Vector3f &gyrData,
                          float temperature,
                          const Eigen::Quaternionf &dmpQuat) {
    // Проверка валидности данных
    if (!isValidData(magData.data(), accData.data(), gyrData.data())) {
        return;
    }

    // 1. Прогноз состояния
    // Используем кватернион DMP для предсказания изменения ориентации
    Eigen::Matrix3f rotationMatrix = dmpQuat.toRotationMatrix();
    
    // Прогноз магнитного поля с учетом изменения ориентации
    Eigen::Vector3f predictedMag = rotationMatrix * state.head<3>();
    
    // Прогноз параметров калибровки (считаем их постоянными)
    Eigen::Matrix<float, 9, 1> predictedState;
    predictedState.head<3>() = predictedMag;
    predictedState.segment<3>(3) = state.segment<3>(3); // scale
    predictedState.tail<3>() = state.tail<3>(); // offset

    // 2. Прогноз ковариации ошибки
    // Матрица перехода состояния
    Eigen::Matrix<float, 9, 9> F = Eigen::Matrix<float, 9, 9>::Identity();
    F.block<3,3>(0,0) = rotationMatrix;
    
    // Прогноз ковариации ошибки
    Eigen::Matrix<float, 9, 9> predictedErrorCovariance = 
        F * errorCovariance * F.transpose() + processNoise;

    // 3. Обновление измерения
    // Ожидаемое измерение магнитометра
    Eigen::Vector3f expectedMag = predictedState.head<3>();
    
    // Ожидаемое измерение акселерометра (гравитация)
    Eigen::Vector3f expectedAcc = dmpQuat * Eigen::Vector3f(0, 0, 1);
    
    // Объединяем ожидаемые измерения
    Eigen::Matrix<float, 6, 1> expectedMeasurement;
    expectedMeasurement.head<3>() = expectedMag;
    expectedMeasurement.tail<3>() = expectedAcc;

    // 4. Матрица Якоби измерений
    Eigen::Matrix<float, 6, 9> H = Eigen::Matrix<float, 6, 9>::Zero();
    H.block<3,3>(0,0) = Eigen::Matrix3f::Identity(); // Магнитометр
    H.block<3,3>(3,3) = Eigen::Matrix3f::Identity(); // Акселерометр

    // 5. Калмановский коэффициент усиления
    Eigen::Matrix<float, 9, 6> K = predictedErrorCovariance * H.transpose() * 
        (H * predictedErrorCovariance * H.transpose() + measurementNoise).inverse();

    // 6. Обновление состояния
    Eigen::Matrix<float, 6, 1> measurement;
    measurement.head<3>() = magData;
    measurement.tail<3>() = accData;
    
    state = predictedState + K * (measurement - expectedMeasurement);

    // 7. Обновление ковариации ошибки
    errorCovariance = (Eigen::Matrix<float, 9, 9>::Identity() - K * H) * predictedErrorCovariance;

    // 8. Обновление параметров калибровки
    offset = state.tail<3>();
    scale = state.segment<3>(3).cwiseMax(1e-6f); // Избегаем отрицательных масштабов
    
    // Обновление soft iron матрицы
    Eigen::Matrix3f newSoftIron;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            newSoftIron(i,j) = state(6 + i*3 + j);
        }
    }
    
    // Ортогонализация soft iron матрицы
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(newSoftIron, Eigen::ComputeFullU | Eigen::ComputeFullV);
    softIronMatrix = svd.matrixU() * svd.matrixV().transpose();

    updateCalibrationMatrix();
}

*/

Eigen::Vector3f Calibration::getCalibratedData(float x, float y, float z) {
    Eigen::Vector3f raw;
    raw << x, y, z;
    Eigen::Vector3f calibrated = calibrationMatrix * (raw - offset);
//    Serial.printf("Calibration matrix: [%.2f, %.2f, %.2f]\n", calibrationMatrix(0,0), calibrationMatrix(1,1), calibrationMatrix(2,2));
//    Serial.printf("Offset: [%.2f, %.2f, %.2f]\n", offset[0], offset[1], offset[2]);
//    Serial.printf("Raw Data: [%.2f, %.2f, %.2f] Calibrated data: [%.2f, %.2f, %.2f]\n", raw[0], raw[1], raw[2], calibrated[0], calibrated[1], calibrated[2]);
    return calibrated;  
}

void Calibration::performEllipsoidFitting(Eigen::Vector3f* magSamples, int numSmpl) {
    // Вычисляем центр масс данных магнитометра
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    for (int i = 0; i < numSmpl; ++i) {
        center += magSamples[i];
    }
    center /= numSmpl;
    
    // Центрируем данные 
    Eigen::Matrix3Xf centeredData(3, numSmpl);
    for (int i = 0; i < numSmpl; ++i) {
        centeredData.col(i) = magSamples[i] - center;
    }
    
    // Вычисляем ковариационную матрицу
    Eigen::Matrix3f cov = centeredData * centeredData.transpose() / numSmpl;
    
    // Выполняем сингулярное разложение (SVD)
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    // Получаем матрицу вращения и масштабы
    Eigen::Matrix3f rotation = svd.matrixU() * svd.matrixV().transpose();
    Eigen::Vector3f scales = svd.singularValues();
    
    // Обновляем параметры
    offset = center;
    scale = scales;
    softIronMatrix = rotation;
}

void Calibration::performRANSAC(Eigen::Vector3f* magSamples, int numSmpl, int numIterations, float threshold) {
    int bestInliers = 0;
    Eigen::Vector3f bestOffset = offset; // Используем смещение из эллипсоидального фиттинга
    Eigen::Vector3f bestScale = scale;   // Используем масштаб из эллипсоидального фиттинга
    Eigen::Matrix3f bestSoftIron = softIronMatrix; // Используем матрицу soft iron из эллипсоидального фиттинга
    
    if (numSmpl < 6) {
        Serial.println("Not enough samples for RANSAC");
        return;
    }

    // Подсчет inliers для начальной модели
    for (int j = 0; j < numSmpl; j++) {
        Eigen::Vector3f point = magSamples[j];
        Eigen::Vector3f calibratedPoint = bestSoftIron.inverse() * bestScale.asDiagonal().inverse() * (point - bestOffset);
        if (fabs(calibratedPoint.norm() - 1.0f) < threshold) {
            bestInliers++;
        }
    }
    
    for (int i = 0; i < numIterations; i++) {
        // Случайно выбираем 6 точек для оценки модели
//        Serial.printf("Selecting 6 random points. Iteration: %d\n", i);
 

        // Создаем матрицу из 6 случайных точек
        Eigen::Matrix3Xf samples(3, 6);
        for (int col=0;col <6; col++) {
            samples.col(col) = magSamples[Eigen::internal::random(0, numSmpl)];
        }
        
        // Выполняем эллипсоидальный фитинг на выбранных точках
        Eigen::Vector3f sampleCenter = samples.rowwise().mean();
        Eigen::Matrix3Xf centeredSamples = samples.colwise() - sampleCenter;
        Eigen::Matrix3f sampleCov = centeredSamples * centeredSamples.transpose() / 6;
        Eigen::JacobiSVD<Eigen::Matrix3f> sampleSvd(sampleCov, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3f sampleRotation = sampleSvd.matrixU() * sampleSvd.matrixV().transpose();
        Eigen::Vector3f sampleScales = sampleSvd.singularValues();
        
        // Считаем количество inliers
        int inliers = 0;
        for (int j = 0; j < numSmpl; j++) {
            Eigen::Vector3f point = magSamples[j];
            Eigen::Vector3f calibratedPoint = sampleRotation.inverse() * sampleScales.asDiagonal().inverse() * (point - sampleCenter);
            if (fabs(calibratedPoint.norm() - 1.0f) < threshold) {
                inliers++;
            }
        }
        
        // Обновляем лучшую модель, если нашли больше inliers
        if (inliers > bestInliers) {
            bestInliers = inliers;
            bestOffset = sampleCenter;
            bestScale = sampleScales;
            bestSoftIron = sampleRotation;
        }
    }

    Serial.printf("RANSAC best inliers: %d\n", bestInliers);
    
    // Обновляем параметры лучшей моделью
    offset = bestOffset;
    scale = bestScale;
    softIronMatrix = bestSoftIron;
}

void Calibration::updateCalibrationMatrix() {
    // Обновляем матрицу калибровки на основе текущих параметров
    Eigen::Matrix3f scaleInversed;
    scaleInversed << 1.0f/scale(0), 0, 0,
                     0, 1.0f/scale(1), 0,
                     0, 0, 1.0f/scale(2);

    
    calibrationMatrix = softIronMatrix.inverse() * scaleInversed;
    
}

bool Calibration::isValidData(const float* mag, const float* acc, const float* gyr) {
    // Проверяем, что модуль ускорения близок к 1g
    float accNorm = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (fabs(accNorm - 9.81f) > 0.5f) { // Увеличили допуск до 0.5g
        return false;
    }
    
    // Проверяем, что угловая скорость мала
    float gyrNorm = sqrt(gyr[0] * gyr[0] + gyr[1] * gyr[1] + gyr[2] * gyr[2]);
    if (gyrNorm > 0.2f) { // Увеличили порог до 0.2 рад/с
        return false;
    }
    
    // Проверяем, что магнитное поле не слишком мало
    float magNorm = sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
    if (magNorm < 1e-6f) {
        return false;
    }
    
    return true;
} 

void Calibration::performGeometricCalibration(Eigen::Vector3f* magSamples, int numSmpl, int numIterations, float threshold, float tolerance) {
   

    // Инициализация параметров эллипсоида
    Eigen::Vector3f center = offset;
    Eigen::Vector3f radii = scale;
    Eigen::Matrix3f rotation = softIronMatrix;

    for (int iter = 0; iter < numIterations; iter++) {
        int validPoints = 0;
        Eigen::Vector3f dCenter = Eigen::Vector3f::Zero();
        Eigen::Vector3f dRadii = Eigen::Vector3f::Zero();
        Eigen::Matrix3f dRotation = Eigen::Matrix3f::Zero();

        for (int i = 0; i < numSmpl; i++) {
            Eigen::Vector3f point = magSamples[i];
            Eigen::Vector3f transformedPoint = rotation.inverse() * radii.asDiagonal().inverse() * (point - center);

            float distance = transformedPoint.norm();
            if (distance < 1e-6) continue;  // Избегаем деления на очень маленькие числа

            float error = distance - 1.0f;
            if (fabs(error) > threshold) continue;  // Игнорируем выбросы

            validPoints++;

            Eigen::Vector3f normalizedPoint = transformedPoint / distance;
            
            // Градиенты для центра и радиусов
            dCenter += error * normalizedPoint;
            dRadii += error * (transformedPoint.array().square() / radii.array().cube()).matrix();
            
            // Градиент для матрицы вращения
            Eigen::Matrix3f J = Eigen::Matrix3f::Zero();
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    Eigen::Matrix3f dR = Eigen::Matrix3f::Zero();
                    dR(j, k) = 1.0f;
                    Eigen::Vector3f dPoint = (rotation.inverse() * dR * rotation.inverse()) * 
                                            radii.asDiagonal().inverse() * (point - center);
                    J(j, k) = -error * normalizedPoint.dot(dPoint);
                }
            }
            dRotation += J;
        }

        if (validPoints < 6) break;  // Минимум 6 точек нужно для определения эллипсоида

        float sc = 1.0f / validPoints;
        
        // Обновляем параметры с меньшим шагом для стабильности
        center += dCenter * sc * 0.5f;
        radii += dRadii * sc * 0.5f;
        
        // Обновляем матрицу вращения с помощью экспоненциального отображения
        Eigen::Matrix3f dR = dRotation * sc * 0.1f;
        Eigen::Matrix3f skewSymmetric = 0.5f * (dR - dR.transpose());
        Eigen::Matrix3f expMap = Eigen::Matrix3f::Identity() + skewSymmetric + 0.5f * skewSymmetric * skewSymmetric;
        rotation = expMap * rotation;
        
        // Ортогонализация матрицы вращения
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
        rotation = svd.matrixU() * svd.matrixV().transpose();

        // Проверяем, что радиусы остаются положительными
        radii = radii.cwiseMax(0.1f);

        // Проверка сходимости
        if (dCenter.norm() * sc < tolerance && 
            dRadii.norm() * sc < tolerance && 
            dRotation.norm() * sc < tolerance) {
            break;
        }
    }

    // Обновляем параметры калибровки
    offset = center;
    scale = radii;
    softIronMatrix = rotation;
}

void Calibration::performFullGeometricCalibration(Eigen::Vector3f* magSamples, int numSmpl, int numIterations, float threshold, float tolerance) {
    
    // Инициализация параметров эллипсоида
    Eigen::Vector3f center = offset;
    Eigen::Vector3f radii = scale;
    Eigen::Matrix3f rotation = softIronMatrix;
    
    for (int iter = 0; iter < numIterations; iter++) {
        Serial.printf("Full geometric calibration iteration: %d\n", iter);
        int validPoints = 0;
        Eigen::Vector3f dCenter = Eigen::Vector3f::Zero();
        Eigen::Vector3f dRadii = Eigen::Vector3f::Zero();
        Eigen::Matrix3f dRotation = Eigen::Matrix3f::Zero();
        
        for (int i = 0; i < numSmpl; i++) {
            Eigen::Vector3f point = magSamples[i];
            Eigen::Vector3f transformedPoint = rotation.inverse() * radii.asDiagonal().inverse() * (point - center);
            
            float distance = transformedPoint.norm();
            if (distance < 1e-6) continue;  // Избегаем деления на очень маленькие числа
            
            float error = distance - 1.0f;
            if (fabs(error) > threshold) continue;  // Игнорируем выбросы
            
            validPoints++;
            
            Eigen::Vector3f normalizedPoint = transformedPoint / distance;
            
            // Градиенты для центра и радиусов
            dCenter += error * normalizedPoint;
            dRadii += error * (transformedPoint.array().square() / radii.array().cube()).matrix();
            
            // Градиент для матрицы вращения
            Eigen::Matrix3f J = Eigen::Matrix3f::Zero();
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    Eigen::Matrix3f dR = Eigen::Matrix3f::Zero();
                    dR(j, k) = 1.0f;
                    Eigen::Vector3f dPoint = (rotation.inverse() * dR * rotation.inverse()) * 
                                            radii.asDiagonal().inverse() * (point - center);
                    J(j, k) = -error * normalizedPoint.dot(dPoint);
                }
            }
            dRotation += J;
        }
        
        if (validPoints < 10) break;  // Минимум 10 точек для полной калибровки
        
        float sc = 1.0f / validPoints;
        
        // Обновляем параметры с меньшим шагом для стабильности
        center += dCenter * sc * 0.5f;
        radii += dRadii * sc * 0.5f;
        
        // Обновляем матрицу вращения с помощью экспоненциального отображения
        Eigen::Matrix3f dR = dRotation * sc * 0.1f;
        Eigen::Matrix3f skewSymmetric = 0.5f * (dR - dR.transpose());
        Eigen::Matrix3f expMap = Eigen::Matrix3f::Identity() + skewSymmetric + 0.5f * skewSymmetric * skewSymmetric;
        rotation = expMap * rotation;
        
        // Ортогонализация матрицы вращения
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
        rotation = svd.matrixU() * svd.matrixV().transpose();

        // Проверяем, что радиусы остаются положительными
        radii = radii.cwiseMax(0.1f);

        // Проверка сходимости
        if (dCenter.norm() * sc < tolerance && 
            dRadii.norm() * sc < tolerance && 
            dRotation.norm() * sc < tolerance) {
            break;
        }
    }

    // Обновляем параметры калибровки
    offset = center;
    scale = radii;
    softIronMatrix = rotation;
}

bool Calibration::saveCalibration(char* name) {
    Preferences preferences;
    if (!preferences.begin(name, false)) {
        return false;
    }
    preferences.putFloat("offset_x", offset[0]);
    preferences.putFloat("offset_y", offset[1]);
    preferences.putFloat("offset_z", offset[2]);
    preferences.putFloat("scale_x", scale[0]);
    preferences.putFloat("scale_y", scale[1]);
    preferences.putFloat("scale_z", scale[2]);
    preferences.putFloat("softIron_xx", softIronMatrix(0,0));
    preferences.putFloat("softIron_xy", softIronMatrix(0,1));
    preferences.putFloat("softIron_xz", softIronMatrix(0,2));
    preferences.putFloat("softIron_yx", softIronMatrix(1,0));
    preferences.putFloat("softIron_yy", softIronMatrix(1,1));
    preferences.putFloat("softIron_yz", softIronMatrix(1,2));
    preferences.putFloat("softIron_zx", softIronMatrix(2,0));
    preferences.putFloat("softIron_zy", softIronMatrix(2,1));
    preferences.putFloat("softIron_zz", softIronMatrix(2,2));
    preferences.end();
    return true;
}

bool Calibration::loadCalibration(char* name) {
    Preferences preferences;
    if (!preferences.begin(name, false)) {
        return false;
    }
    offset[0] = preferences.getFloat("offset_x");
    offset[1] = preferences.getFloat("offset_y");
    offset[2] = preferences.getFloat("offset_z");
    scale[0] = preferences.getFloat("scale_x");
    scale[1] = preferences.getFloat("scale_y");
    scale[2] = preferences.getFloat("scale_z");
    softIronMatrix(0,0) = preferences.getFloat("softIron_xx");
    softIronMatrix(0,1) = preferences.getFloat("softIron_xy");
    softIronMatrix(0,2) = preferences.getFloat("softIron_xz");
    softIronMatrix(1,0) = preferences.getFloat("softIron_yx");
    softIronMatrix(1,1) = preferences.getFloat("softIron_yy");
    softIronMatrix(1,2) = preferences.getFloat("softIron_yz");
    softIronMatrix(2,0) = preferences.getFloat("softIron_zx");
    softIronMatrix(2,1) = preferences.getFloat("softIron_zy");
    softIronMatrix(2,2) = preferences.getFloat("softIron_zz");
    preferences.end();
    return true;
}

void Calibration::performMinMaxCalibration(Eigen::Vector3f* magSamples, int numSmpl) {
    if (numSmpl < 2) {
        Serial.println("Not enough samples for Min-Max calibration");
        return;
    }

    // Инициализация минимальных и максимальных значений
    Eigen::Vector3f minValues = magSamples[0];
    Eigen::Vector3f maxValues = magSamples[0];

    // Находим минимальные и максимальные значения по каждой оси
    for (int i = 1; i < numSmpl; i++) {
        minValues = minValues.cwiseMin(magSamples[i]);
        maxValues = maxValues.cwiseMax(magSamples[i]);
    }

    // Вычисляем смещение как среднее между минимальными и максимальными значениями
    offset = (minValues + maxValues) / 2.0f;

    // Вычисляем масштаб как половину разницы между максимальными и минимальными значениями
    scale = (maxValues - minValues) / 2.0f;

    // Устанавливаем матрицу soft iron как единичную матрицу
    softIronMatrix = Eigen::Matrix3f::Identity();

    // Обновляем матрицу калибровки
    updateCalibrationMatrix();
}

void Calibration::performQuaternionCalibration(Eigen::Vector3f* magSamples, 
                                             Eigen::Quaternionf* quatSamples, 
                                             int numSmpl, 
                                             int numIterations,
                                             float threshold,
                                             float tolerance) {
    Serial.println("Performing quaternion-based calibration");
    
    // Инициализация параметров эллипсоида
    Eigen::Vector3f center = offset;
    Eigen::Vector3f radii = scale;
    Eigen::Matrix3f rotation = softIronMatrix;
    
    // Начальное приближение для угла к северу
    float northAngle = 0.0f;
    
    for (int iter = 0; iter < numIterations; iter++) {
        int validPoints = 0;
        Eigen::Vector3f dCenter = Eigen::Vector3f::Zero();
        Eigen::Vector3f dRadii = Eigen::Vector3f::Zero();
        Eigen::Matrix3f dRotation = Eigen::Matrix3f::Zero();
        float dNorthAngle = 0.0f;
        
        // Создаем кватернион для поворота к северу
        Eigen::Quaternionf northQuat(Eigen::AngleAxisf(northAngle, Eigen::Vector3f::UnitZ()));
        
        for (int i = 0; i < numSmpl; i++) {
            // Преобразуем измерение в глобальную систему координат
            Eigen::Vector3f globalMag = quatSamples[i].conjugate() * magSamples[i];
            
            // Применяем калибровку
            Eigen::Vector3f centeredMag = globalMag - center;
            Eigen::Vector3f scaledMag = radii.asDiagonal().inverse() * centeredMag;
            Eigen::Vector3f transformedMag = rotation.inverse() * scaledMag;
            
            // Поворачиваем к северу
            Eigen::Vector3f northAlignedMag = northQuat * transformedMag;
            
            float distance = northAlignedMag.norm();
            if (distance < 1e-6) continue;
            
            float error = distance - 1.0f;
            if (fabs(error) > threshold) continue;
            
            validPoints++;
            
            Eigen::Vector3f normalizedMag = northAlignedMag / distance;
            
            // Градиенты для центра
            dCenter += error * normalizedMag;
            
            // Градиенты для радиусов
            dRadii += error * (transformedMag.array().square() / radii.array().cube()).matrix();
            
            // Градиент для матрицы вращения
            Eigen::Matrix3f J = Eigen::Matrix3f::Zero();
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    Eigen::Matrix3f dR = Eigen::Matrix3f::Zero();
                    dR(j, k) = 1.0f;
                    Eigen::Vector3f dPoint = (rotation.inverse() * dR * rotation.inverse()) * 
                                            radii.asDiagonal().inverse() * centeredMag;
                    J(j, k) = -error * normalizedMag.dot(dPoint);
                }
            }
            dRotation += J;
            
            // Градиент для угла к северу
            Eigen::Vector3f dNorth = Eigen::Vector3f(-northAlignedMag.y(), northAlignedMag.x(), 0.0f);
            dNorthAngle += error * normalizedMag.dot(dNorth);
        }
        
        if (validPoints < 10) break;
        
        float sc = 1.0f / validPoints;
        
        // Обновляем параметры с меньшим шагом для стабильности
        center += dCenter * sc * 0.5f;
        radii += dRadii * sc * 0.5f;
        northAngle += dNorthAngle * sc * 0.1f;
        
        // Обновляем матрицу вращения
        Eigen::Matrix3f dR = dRotation * sc * 0.1f;
        Eigen::Matrix3f skewSymmetric = 0.5f * (dR - dR.transpose());
        Eigen::Matrix3f expMap = Eigen::Matrix3f::Identity() + skewSymmetric + 0.5f * skewSymmetric * skewSymmetric;
        rotation = expMap * rotation;
        
        // Ортогонализация матрицы вращения
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
        rotation = svd.matrixU() * svd.matrixV().transpose();
        
        // Проверяем, что радиусы остаются положительными
        radii = radii.cwiseMax(0.1f);
        
        // Проверка сходимости
        if (dCenter.norm() * sc < tolerance && 
            dRadii.norm() * sc < tolerance && 
            dRotation.norm() * sc < tolerance &&
            fabs(dNorthAngle * sc) < tolerance) {
            break;
        }
    }
    
    // Обновляем параметры калибровки
    offset = center;
    scale = radii;
    softIronMatrix = rotation;
    
    // Обновляем матрицу калибровки
    updateCalibrationMatrix();
    
    Serial.printf("Quaternion calibration done. North angle: %.2f degrees\n", northAngle * 180.0f / M_PI);
}

bool Calibration::performFastCalibration(Eigen::Vector3f* magSamples, 
                                       Eigen::Quaternionf* quatSamples, 
                                       int numSmpl) {
    if (numSmpl < 9) { // Нужно минимум 9 точек для оценки всех параметров
        Serial.println("Недостаточно измерений для калибровки");
        return false;
    }

    // Создаем матрицу для метода наименьших квадратов
    Eigen::MatrixXf A(numSmpl, 9);
    Eigen::VectorXf b(numSmpl);

    // Заполняем матрицу A и вектор b
    for (int i = 0; i < numSmpl; i++) {
        // Преобразуем измерение в глобальную систему координат
        Eigen::Vector3f globalMag = quatSamples[i].conjugate() * magSamples[i];
        
        // Заполняем строку матрицы A
        A(i, 0) = globalMag.x() * globalMag.x();
        A(i, 1) = globalMag.y() * globalMag.y();
        A(i, 2) = globalMag.z() * globalMag.z();
        A(i, 3) = 2.0f * globalMag.x() * globalMag.y();
        A(i, 4) = 2.0f * globalMag.x() * globalMag.z();
        A(i, 5) = 2.0f * globalMag.y() * globalMag.z();
        A(i, 6) = 2.0f * globalMag.x();
        A(i, 7) = 2.0f * globalMag.y();
        A(i, 8) = 2.0f * globalMag.z();
        
        // Заполняем вектор b
        b(i) = 1.0f; // Нормализованное магнитное поле
    }

    // Решаем систему методом наименьших квадратов
    Eigen::VectorXf x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    // Извлекаем параметры эллипсоида
    Eigen::Matrix3f Q;
    Q << x(0), x(3), x(4),
         x(3), x(1), x(5),
         x(4), x(5), x(2);

    // Вычисляем смещение
    Eigen::Vector3f center;
    center << -x(6), -x(7), -x(8);
    center = Q.inverse() * center;

    // Выполняем сингулярное разложение для Q
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(Q, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    // Получаем матрицу вращения и масштабы
    Eigen::Matrix3f rotation = svd.matrixU() * svd.matrixV().transpose();
    Eigen::Vector3f scales = svd.singularValues().cwiseSqrt().cwiseInverse();

    // Обновляем параметры калибровки
    offset = center;
    scale = scales;
    softIronMatrix = rotation;

    // Обновляем матрицу калибровки
    updateCalibrationMatrix();

    // Проверяем качество калибровки
    float avgError = 0.0f;
    for (int i = 0; i < numSmpl; i++) {
        Eigen::Vector3f calibrated = getCalibratedData(magSamples[i].x(), magSamples[i].y(), magSamples[i].z());
        avgError += fabs(calibrated.norm() - 1.0f);
    }
    avgError /= numSmpl;

    Serial.printf("Быстрая калибровка завершена. Средняя ошибка: %.4f\n", avgError);
    Serial.printf("Смещение: [%.2f, %.2f, %.2f]\n", offset[0], offset[1], offset[2]);
    
    return avgError < 0.1f; // Возвращаем true, если средняя ошибка меньше 10%
}

/*
void Calibration::performQuaternionCalibration(Eigen::Vector3f* magSamples, 
                                                    Eigen::Quaternionf* quatSamples, 
                                                    int numSmpl,
                                                    int numIterations ,
                                                    float threshold ,
                                                    float tolerance ) {
    if (numSmpl < 9) {
        Serial.println("Недостаточно измерений для калибровки");
        return false;
    }

    // Сохраняем лучшие параметры
    Eigen::Vector3f bestOffset = offset;
    Eigen::Vector3f bestScale = scale;
    Eigen::Matrix3f bestSoftIron = softIronMatrix;
    int bestInliers = 0;
    float bestError = 1e6f;

    // RANSAC цикл
    for (int iter = 0; iter < numIterations; iter++) {
        // Случайно выбираем 9 точек
        std::vector<int> indices;
        for (int i = 0; i < 9; i++) {
            int idx;
            do {
                idx = random(numSmpl);
            } while (std::find(indices.begin(), indices.end(), idx) != indices.end());
            indices.push_back(idx);
        }

        // Создаем матрицу для метода наименьших квадратов
        Eigen::MatrixXf A(9, 9);
        Eigen::VectorXf b(9);

        // Заполняем матрицу A и вектор b
        for (int i = 0; i < 9; i++) {
            int idx = indices[i];
            Eigen::Vector3f globalMag = quatSamples[idx].conjugate() * magSamples[idx];
            
            A(i, 0) = globalMag.x() * globalMag.x();
            A(i, 1) = globalMag.y() * globalMag.y();
            A(i, 2) = globalMag.z() * globalMag.z();
            A(i, 3) = 2.0f * globalMag.x() * globalMag.y();
            A(i, 4) = 2.0f * globalMag.x() * globalMag.z();
            A(i, 5) = 2.0f * globalMag.y() * globalMag.z();
            A(i, 6) = 2.0f * globalMag.x();
            A(i, 7) = 2.0f * globalMag.y();
            A(i, 8) = 2.0f * globalMag.z();
            
            b(i) = 1.0f;
        }

        // Решаем систему
        Eigen::VectorXf x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

        // Извлекаем параметры эллипсоида
        Eigen::Matrix3f Q;
        Q << x(0), x(3), x(4),
             x(3), x(1), x(5),
             x(4), x(5), x(2);

        // Вычисляем смещение
        Eigen::Vector3f center;
        center << -x(6), -x(7), -x(8);
        center = Q.inverse() * center;

        // Выполняем сингулярное разложение
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(Q, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3f rotation = svd.matrixU() * svd.matrixV().transpose();
        Eigen::Vector3f scales = svd.singularValues().cwiseSqrt().cwiseInverse();

        // Временно обновляем параметры
        offset = center;
        scale = scales;
        softIronMatrix = rotation;
        updateCalibrationMatrix();

        // Подсчитываем количество inliers
        int inliers = 0;
        float totalError = 0.0f;
        for (int i = 0; i < numSmpl; i++) {
            Eigen::Vector3f calibrated = getCalibratedData(magSamples[i].x(), magSamples[i].y(), magSamples[i].z());
            float error = fabs(calibrated.norm() - 1.0f);
            if (error < threshold) {
                inliers++;
                totalError += error;
            }
        }

        // Обновляем лучшие параметры
        if (inliers > bestInliers || (inliers == bestInliers && totalError < bestError)) {
            bestInliers = inliers;
            bestError = totalError;
            bestOffset = offset;
            bestScale = scale;
            bestSoftIron = softIronMatrix;
        }
    }

    // Применяем лучшие параметры
    offset = bestOffset;
    scale = bestScale;
    softIronMatrix = bestSoftIron;
    updateCalibrationMatrix();

    // Проверяем качество калибровки
    float avgError = 0.0f;
    int validPoints = 0;
    for (int i = 0; i < numSmpl; i++) {
        Eigen::Vector3f calibrated = getCalibratedData(magSamples[i].x(), magSamples[i].y(), magSamples[i].z());
        float error = fabs(calibrated.norm() - 1.0f);
        if (error < threshold) {
            avgError += error;
            validPoints++;
        }
    }
    
    if (validPoints > 0) {
        avgError /= validPoints;
    }

    Serial.printf("Начальная калибровка завершена. Inliers: %d/%d, Средняя ошибка: %.4f\n", 
                 bestInliers, numSmpl, avgError);
    Serial.printf("Смещение: [%.2f, %.2f, %.2f]\n", offset[0], offset[1], offset[2]);
    
    return bestInliers >= numSmpl / 2 && avgError < 0.1f;
}
*/

bool Calibration::calibrateForParallelVectors(Eigen::Vector3f* magSamples, 
                                            Eigen::Quaternionf* quatSamples, 
                                            int numSmpl,
                                            int numIterations) {
    if (numSmpl < 3) {
        Serial.println("Недостаточно измерений для калибровки");
        return false;
    }

    // Инициализация параметров
    offset = Eigen::Vector3f::Zero();
    scale = Eigen::Vector3f::Ones();
    softIronMatrix = Eigen::Matrix3f::Identity();
    
    // Начальное приближение для направления магнитного поля
    Eigen::Vector3f targetDirection = Eigen::Vector3f::UnitX();
    
    // Градиентный спуск
    for (int iter = 0; iter < numIterations; iter++) {
        Eigen::Vector3f dOffset = Eigen::Vector3f::Zero();
        Eigen::Vector3f dScale = Eigen::Vector3f::Zero();
        Eigen::Matrix3f dSoftIron = Eigen::Matrix3f::Zero();
        int validPoints = 0;
        
        for (int i = 0; i < numSmpl; i++) {
            // Применяем текущую калибровку
            Eigen::Vector3f calibrated = getCalibratedData(magSamples[i].x(), magSamples[i].y(), magSamples[i].z());
            
            // Преобразуем в глобальную систему координат
            Eigen::Vector3f globalMag = quatSamples[i].conjugate() * calibrated;
            
            // Нормализуем
            float norm = globalMag.norm();
            if (norm < 1e-6) continue;
            globalMag /= norm;
            
            // Вычисляем ошибку как угол между векторами
            float cosAngle = globalMag.dot(targetDirection);
            if (cosAngle < -1.0f) cosAngle = -1.0f;
            if (cosAngle > 1.0f) cosAngle = 1.0f;
            float angle = acosf(cosAngle);
            
            // Пропускаем выбросы
            if (angle > M_PI/4) continue;
            
            validPoints++;
            
            // Вычисляем градиенты
            Eigen::Vector3f error = globalMag - targetDirection;
            
            // Градиент для смещения
            dOffset += error;
            
            // Градиент для масштаба
            dScale += error.cwiseProduct(calibrated);
            
            // Градиент для soft iron матрицы
            dSoftIron += error * calibrated.transpose();
        }
        
        if (validPoints == 0) continue;
        
        // Нормализуем градиенты
        float sc = 0.1f / validPoints;
        offset -= dOffset * sc;
        scale -= dScale * sc;
        softIronMatrix -= dSoftIron * sc;
        
        // Ограничиваем масштабы
        scale = scale.cwiseMax(0.1f);
        
        // Ортогонализация soft iron матрицы
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(softIronMatrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
        softIronMatrix = svd.matrixU() * svd.matrixV().transpose();
        
        // Обновляем матрицу калибровки
        updateCalibrationMatrix();
        
        // Обновляем целевое направление как среднее всех векторов
        targetDirection = Eigen::Vector3f::Zero();
        for (int i = 0; i < numSmpl; i++) {
            Eigen::Vector3f calibrated = getCalibratedData(magSamples[i].x(), magSamples[i].y(), magSamples[i].z());
            Eigen::Vector3f globalMag = quatSamples[i].conjugate() * calibrated;
            float norm = globalMag.norm();
            if (norm > 1e-6) {
                targetDirection += globalMag / norm;
            }
        }
        targetDirection.normalize();
    }
    
    // Проверяем качество калибровки
    float maxAngle = 0.0f;
    float avgAngle = 0.0f;
    int validPoints = 0;
    
    for (int i = 0; i < numSmpl; i++) {
        Eigen::Vector3f calibrated = getCalibratedData(magSamples[i].x(), magSamples[i].y(), magSamples[i].z());
        Eigen::Vector3f globalMag = quatSamples[i].conjugate() * calibrated;
        float norm = globalMag.norm();
        if (norm < 1e-6) continue;
        
        globalMag /= norm;
        float cosAngle = globalMag.dot(targetDirection);
        if (cosAngle < -1.0f) cosAngle = -1.0f;
        if (cosAngle > 1.0f) cosAngle = 1.0f;
        float angle = acosf(cosAngle);
        
        if (angle < M_PI/4) {
            maxAngle = fmaxf(maxAngle, angle);
            avgAngle += angle;
            validPoints++;
        }
    }
    
    if (validPoints > 0) {
        avgAngle /= validPoints;
    }
    
    Serial.printf("Калибровка завершена. Средний угол: %.2f°, Максимальный угол: %.2f°\n", 
                 avgAngle * 180.0f / M_PI, maxAngle * 180.0f / M_PI);
    Serial.printf("Смещение: [%.2f, %.2f, %.2f]\n", offset[0], offset[1], offset[2]);
    
    return validPoints >= numSmpl / 2 && avgAngle < M_PI/12; // Успех, если средний угол меньше 15°
}

// Вычисление матрицы шума измерений R из статических данных магнитометра
Eigen::Matrix3f calculateMeasurementNoise(const std::vector<Eigen::Vector3f>& staticMagData) {
     
    // Вычисляем среднее значение
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const auto& data : staticMagData) {
        mean += data;
    }
    mean /= staticMagData.size();
    
    // Вычисляем ковариационную матрицу
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (const auto& data : staticMagData) {
        Eigen::Vector3f diff = data - mean;
        covariance += diff * diff.transpose();
    }
    covariance /= (staticMagData.size() - 1);
    
    // В статье используется только дисперсия, поэтому берем диагональ
    return covariance.diagonal().asDiagonal();
}

// Вычисление матрицы шума процесса Q из статических данных гироскопа и акселерометра
Eigen::Matrix<float, 12, 12> calculateProcessNoise(const std::vector<Eigen::Matrix3f>& deltaRotations) {
    if (deltaRotations.empty()) {
        // Значения по умолчанию из статьи
        Eigen::Matrix3f defaultDeltaRCov;
        defaultDeltaRCov << 1.2e-8f, 3.0e-3f, 1.4e-3f,
                           3.0e-3f, 4.0e-8f, 2.5e-3f,
                           1.4e-3f, 2.5e-3f, 6.5e-8f;
        
        Eigen::Matrix<float, 12, 12> Q = Eigen::Matrix<float, 12, 12>::Zero();
        Q.block<3,3>(0,0) = defaultDeltaRCov;
        return Q;
    }
    
    // Вычисляем среднее значение матрицы поворота
    Eigen::Matrix3f meanRotation = Eigen::Matrix3f::Zero();
    for (const auto& deltaR : deltaRotations) {
        meanRotation += deltaR;
    }
    meanRotation /= deltaRotations.size();
    
    // Вычисляем ковариацию для матрицы поворота
    Eigen::Matrix3f rotationCovariance = Eigen::Matrix3f::Zero();
    for (const auto& deltaR : deltaRotations) {
        Eigen::Matrix3f diff = deltaR - meanRotation;
        rotationCovariance += diff * diff.transpose();
    }
    rotationCovariance /= (deltaRotations.size() - 1);
    
    // Формируем полную матрицу Q
    Eigen::Matrix<float, 12, 12> Q = Eigen::Matrix<float, 12, 12>::Zero();
    Q.block<3,3>(0,0) = rotationCovariance;
    
    return Q;
}

// Функция для сбора статических данных
void collectStaticData(int durationSeconds, int samplingRate,
                      std::vector<Eigen::Vector3f>& magData,
                      std::vector<Eigen::Matrix3f>& deltaRotations) {
    magData.clear();
    deltaRotations.clear();
    
    int totalSamples = durationSeconds * samplingRate;
    Eigen::Matrix3f lastRotation = Eigen::Matrix3f::Identity();
    
    for (int i = 0; i < totalSamples; i++) {
        // Здесь должно быть получение данных с сенсоров
        // magData.push_back(getCurrentMagData());
        // Eigen::Matrix3f currentRotation = getCurrentRotationMatrix();
        // deltaRotations.push_back(currentRotation * lastRotation.transpose());
        // lastRotation = currentRotation;
        
        // Задержка для соблюдения частоты дискретизации

    }
}

void Calibration::setCalibrationMatrix(const Eigen::Vector3f &offset, const Eigen::Matrix3f &calibrationMatrix) {
    this->offset = offset;
    this->calibrationMatrix = calibrationMatrix;
    this->scale = calibrationMatrix.diagonal().cwiseSqrt();
}

void Calibration::setCalibrationMatrix(float* offset, float* scale, float* SI) {
    this->offset[0] = offset[0];
    this->offset[1] = offset[1];
    this->offset[2] = offset[2];
    this->scale[0] = scale[0];
    this->scale[1] = scale[1];
    this->scale[2] = scale[2];
    calibrationMatrix(0,0)=scale[0];
    calibrationMatrix(1,1)=scale[1];
    calibrationMatrix(2,2)=scale[2];
    calibrationMatrix(0,1)=SI[0];
    calibrationMatrix(0,2)=SI[1];
    calibrationMatrix(1,2)=SI[2];
    calibrationMatrix(1,0)=SI[0];
    calibrationMatrix(2,0)=SI[1];
    calibrationMatrix(2,1)=SI[2];
}



    






