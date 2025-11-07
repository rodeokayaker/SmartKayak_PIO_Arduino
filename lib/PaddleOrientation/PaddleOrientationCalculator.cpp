#include "PaddleOrientationCalculator.h"

#include "OrientationUtils.h"

#define RAD_TO_DEG 57.295779513082320876798154814105


BladeSideType PaddleOrientationCalculator::getLowerBladeSide() const{
    // Получаем текущее направление оси весла направленного на правую лопатку в глобальной системе
    SP_Math::Vector globalShaftVector = paddleOrientationQuat.rotate(paddleShaftVector);
    if (globalShaftVector.z() < 0) {
        return BladeSideType::RIGHT_BLADE;
    } else {
        return BladeSideType::LEFT_BLADE;
    }
}

double PaddleOrientationCalculator::shaftVectorZ() const{
    SP_Math::Vector globalShaftVector = paddleOrientationQuat.rotate(paddleShaftVector);
    return globalShaftVector.z();
}

void PaddleOrientationCalculator::updateRelativeOrientation() {

    // Получаем направление оси X каяка в мировой системе координат

    SP_Math::Vector arrow = kayakOrientationQuat.rotate(kayakDirectionVector);
    
    // Проецируем на горизонтальную плоскость (убираем наклон каяка)
    float norm_sq = arrow[0] * arrow[0] + arrow[1] * arrow[1];
    
    SP_Math::Vector X_new(0, 0, 0);
    if (norm_sq < 1e-10f) {
        X_new = kayakDirectionVector;
    } else {
        float inv_norm = 1.0f / std::sqrt(norm_sq);
        X_new[0] = arrow[0] * inv_norm;
        X_new[1] = arrow[1] * inv_norm;
        X_new[2] = 0;
    }
    
    // Строим кватернион поворота от [1,0,0] к X_new
    float cos_half_theta = std::sqrt(0.5f * (1 + X_new[0]));
    float sin_half_theta = (X_new[1] >= 0 ? 1 : -1) * std::sqrt(0.5f * (1 - X_new[0]));
    paddleModificationQuat[0] = cos_half_theta;
    paddleModificationQuat[1] = 0;
    paddleModificationQuat[2] = 0;
    paddleModificationQuat[3] = - sin_half_theta;  //because we use conjugate to get relative orientation
    
    // Вычисляем ориентацию весла относительно горизонтального направления каяка
    paddleRelativeQuat = paddleModificationQuat * paddleOrientationQuat;
    
    calculationState |= PO_ORIENTATION_VALID|PO_MODIFICATION_VALID;
}

void PaddleOrientationCalculator::getPaddleAngles(const SP_Math::Quaternion& paddleQuat, float& rotation, float& tilt, float& blade) {

        SP_Math::Vector globalZ(0, 0, 1);  // вертикальный вектор

        // Поворачиваем векторы с использованием относительной ориентации
        SP_Math::Vector currentShaftDir = paddleQuat.rotate(paddleShaftVector);
        SP_Math::Vector currentZinPaddle = paddleQuat.conjugate().rotate(globalZ); // Вертикальный вектор в системе весла

        // Вычисляем углы поворота шафта
        // Проекция оси шафта на плоскость XY каяка
        SP_Math::Vector shaftProjectionXY(currentShaftDir.x(), currentShaftDir.y(), 0);
        shaftProjectionXY.normalize();

        // Угол поворота шафта вокруг Z (в плоскости XY)
        SP_Math::Vector sideDir(0,1,0);

        // Вычисляем угол между осью Y и проекцией шафта на плоскость XY
        float cosAngle = sideDir.dot(shaftProjectionXY);
        float crossZ = sideDir.x() * shaftProjectionXY.y() - sideDir.y() * shaftProjectionXY.x();    

        rotation = atan2(crossZ, cosAngle) * RAD_TO_DEG;

        // Угол наклона шафта относительно плоскости XY
        tilt = asin(currentShaftDir.z()) * RAD_TO_DEG;

        // Угол поворота лопасти вокруг оси шафта
        if (paddleShaftVector.y() != 0) {
            blade = atan2(currentZinPaddle.x(),currentZinPaddle.z()) * RAD_TO_DEG;
        } else {
            if (paddleShaftVector.x() != 0) {
                blade = atan2(currentZinPaddle.y(),currentZinPaddle.z()) * RAD_TO_DEG;
            } else {
                blade = atan2(currentZinPaddle.z(),currentZinPaddle.y()) * RAD_TO_DEG;
            }
        }


}

void PaddleOrientationCalculator::getPaddleAngles(float& rotation, float& tilt, float& blade) {

    if (!(calculationState & PO_ANGLES_VALID)) 
    {
        if (!(calculationState & PO_ORIENTATION_VALID)) updateRelativeOrientation();

        getPaddleAngles(paddleRelativeQuat, paddleShaftRotationAngle, paddleShaftTiltAngle, paddleBladeRotationAngle);


        calculationState |= PO_ANGLES_VALID;
    }
    rotation = paddleShaftRotationAngle;
    tilt = paddleShaftTiltAngle;
    blade = paddleBladeRotationAngle;
}


void PaddleOrientationCalculator::setKayakAxisDirection(AxisDirection kaDir) {
    kayakAxisDirection = kaDir;
    switch (kaDir) {
        case Y_AXIS_FORWARD:
            kayakDirectionVector = SP_Math::Vector(0, 1, 0);
            break;
        case X_AXIS_FORWARD:
            kayakDirectionVector = SP_Math::Vector(1, 0, 0);
            break;
        case Y_AXIS_BACKWARD:
            kayakDirectionVector = SP_Math::Vector(0, -1, 0);
            break;
        case X_AXIS_BACKWARD:
            kayakDirectionVector = SP_Math::Vector(-1, 0, 0);
            break;
        default:
            kayakDirectionVector = SP_Math::Vector(1, 0, 0);
            break;
    }
}

void PaddleOrientationCalculator::setPaddleAxisDirection(AxisDirection paDir) {
    paddleAxisDirection = paDir;
    switch (paDir) {
        case Y_AXIS_RIGHT:
            paddleShaftVector = SP_Math::Vector(0, 1, 0);
            break;
        case X_AXIS_RIGHT:
            paddleShaftVector = SP_Math::Vector(1, 0, 0);
            break;
        case Z_AXIS_RIGHT:
            paddleShaftVector = SP_Math::Vector(0, 0, 1);
            break;
        case Y_AXIS_LEFT:
            paddleShaftVector = SP_Math::Vector(0, -1, 0);
            break;
        case X_AXIS_LEFT:
            paddleShaftVector = SP_Math::Vector(-1, 0, 0);
            break;
    }
}