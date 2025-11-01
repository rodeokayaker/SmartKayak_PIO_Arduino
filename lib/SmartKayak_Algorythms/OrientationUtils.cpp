#include "OrientationUtils.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

void getPaddleAngles(const SP_Math::Quaternion& relativePaddleQ, 
    float& shaftRotationAngle,  // поворот вокруг оси Z каяка
    float& shaftTiltAngle,      // наклон вокруг оси X каяка
    float& bladeRotationAngle)  // поворот вокруг оси Y весла
{
    SP_Math::Vector paddleY(0, 1, 0);  // ось шафта
    SP_Math::Vector globalZ(0, 0, 1);  // вертикальный вектор

    // Поворачиваем векторы с использованием относительной ориентации
    SP_Math::Vector currentShaftDir = relativePaddleQ.rotate(paddleY);
    SP_Math::Vector currentZinPaddle = relativePaddleQ.conjugate().rotate(globalZ); // Вертикальный вектор в системе весла

    // Вычисляем углы поворота шафта
    // Проекция оси шафта на плоскость XY каяка
    SP_Math::Vector shaftProjectionXY(currentShaftDir.x(), currentShaftDir.y(), 0);
    shaftProjectionXY.normalize();

    // Угол поворота шафта вокруг Z (в плоскости XY)
    SP_Math::Vector sideDir(0,1,0);

    // Вычисляем угол между осью Y и проекцией шафта на плоскость XY
    float cosAngle = sideDir.dot(shaftProjectionXY);
    float crossZ = sideDir.x() * shaftProjectionXY.y() - sideDir.y() * shaftProjectionXY.x();    

    shaftRotationAngle = atan2(crossZ, cosAngle) * RAD_TO_DEG;

    // Угол наклона шафта относительно плоскости XY
    shaftTiltAngle = asin(currentShaftDir.z()) * RAD_TO_DEG;

    // Угол поворота лопасти вокруг оси шафта
    bladeRotationAngle = atan2(currentZinPaddle.x(),currentZinPaddle.z());
    bladeRotationAngle = bladeRotationAngle * RAD_TO_DEG;
}

BladeSideType getLowerBladeSide(const SP_Math::Quaternion& paddleQ, int Y_axis_sign) {
    if (Y_axis_sign == 0) {
    return BladeSideType::ALL_BLADES;
    }
    // Получаем текущее направление оси весла направленного на правую лопатку в глобальной системе
    SP_Math::Vector paddleYAxis(0, Y_axis_sign, 0);
    SP_Math::Vector globalYAxis = paddleQ.rotate(paddleYAxis);

    if (globalYAxis.z() < 0) {
    return BladeSideType::RIGHT_BLADE;
    } else {
    return BladeSideType::LEFT_BLADE;
    }

}

SP_Math::Quaternion getRelativeOrientation(
    const SP_Math::Quaternion& paddleQuat,
    const SP_Math::Quaternion& kayakQuat
) 
{
    // Получаем направление оси X каяка в мировой системе координат
    SP_Math::Vector x_k = kayakQuat.rotate(SP_Math::Vector(1, 0, 0));
    
    // Проецируем на горизонтальную плоскость (убираем наклон каяка)
    float norm_sq = x_k[0] * x_k[0] + x_k[1] * x_k[1];
    
    SP_Math::Vector X_new(0, 0, 0);
    if (norm_sq < 1e-10f) {
        X_new[0] = 1;
        X_new[1] = 0;
        X_new[2] = 0;
    } else {
        float inv_norm = 1.0f / std::sqrt(norm_sq);
        X_new[0] = x_k[0] * inv_norm;
        X_new[1] = x_k[1] * inv_norm;
        X_new[2] = 0;
    }
    
    // Строим кватернион поворота от [1,0,0] к X_new
    float cos_half_theta = std::sqrt(0.5f * (1 + X_new[0]));
    float sin_half_theta = (X_new[1] >= 0 ? 1 : -1) * std::sqrt(0.5f * (1 - X_new[0]));
    SP_Math::Quaternion q_new(cos_half_theta, 0, 0, sin_half_theta);
    
    // Вычисляем ориентацию весла относительно горизонтального направления каяка
    SP_Math::Quaternion q_relative = q_new.conjugate() * paddleQuat;
    
    return q_relative.normalize();
}
