#ifndef ORIENTATION_UTILS_H
#define ORIENTATION_UTILS_H

#include "../Core/Types.h"
#include "SP_Math.h"

// Объявления функций
void getPaddleAngles(const SP_Math::Quaternion& relativePaddleQ, 
    float& shaftRotationAngle,  // поворот вокруг оси Z каяка
    float& shaftTiltAngle,      // наклон вокруг оси X каяка
    float& bladeRotationAngle); // поворот вокруг оси Y весла

BladeSideType getLowerBladeSide(const SP_Math::Quaternion& paddleQ, int Y_axis_sign);

SP_Math::Quaternion getRelativeOrientation(
    const SP_Math::Quaternion& paddleQuat,
    const SP_Math::Quaternion& kayakQuat);

#endif