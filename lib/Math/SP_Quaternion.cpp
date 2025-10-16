#include "SP_Quaternion.h"
#include <cmath>

namespace SP_Math
{
    Quaternion::Quaternion() : q{1, 0, 0, 0} {}
    Quaternion::~Quaternion() {}
    Quaternion::Quaternion(const Quaternion& other) : q{other.q[0], other.q[1], other.q[2], other.q[3]} {}
    Quaternion::Quaternion(const Vector& axis, float angle) {
        float halfAngle = angle / 2;
        float sinHalfAngle = sin(halfAngle);
        q[0] = cos(halfAngle);          // w
        q[1] = axis.x() * sinHalfAngle; // x
        q[2] = axis.y() * sinHalfAngle; // y
        q[3] = axis.z() * sinHalfAngle; // z
    }
    Quaternion::Quaternion(float w, float x, float y, float z) : q{w, x, y, z} {}
    Quaternion& Quaternion::operator=(const Quaternion& other) {
        q[0] = other.q[0];
        q[1] = other.q[1];
        q[2] = other.q[2];
        q[3] = other.q[3];
        return *this;
    }
    Quaternion Quaternion::operator*(const Quaternion& other) const {
        return Quaternion(
            q[0]*other.q[0] - q[1]*other.q[1] - q[2]*other.q[2] - q[3]*other.q[3],  // w
            q[0]*other.q[1] + q[1]*other.q[0] + q[2]*other.q[3] - q[3]*other.q[2],  // x
            q[0]*other.q[2] - q[1]*other.q[3] + q[2]*other.q[0] + q[3]*other.q[1],  // y
            q[0]*other.q[3] + q[1]*other.q[2] - q[2]*other.q[1] + q[3]*other.q[0]   // z
        );
    }
    Quaternion Quaternion::operator+(const Quaternion& other) const {
        return Quaternion(q[0]+other.q[0], q[1]+other.q[1], q[2]+other.q[2], q[3]+other.q[3]);
    }
    Quaternion Quaternion::operator-(const Quaternion& other) const {
        return Quaternion(q[0]-other.q[0], q[1]-other.q[1], q[2]-other.q[2], q[3]-other.q[3]);
    }
    Quaternion Quaternion::operator/(const Quaternion& other) const {
        return *this * other.inverse();
    }
    Quaternion Quaternion::operator*(float scalar) const {
        return Quaternion(q[0]*scalar, q[1]*scalar, q[2]*scalar, q[3]*scalar);
    }
    Quaternion Quaternion::operator/(float scalar) const {
        return Quaternion(q[0]/scalar, q[1]/scalar, q[2]/scalar, q[3]/scalar);
    }
    Quaternion Quaternion::conjugate() const {
        return Quaternion(q[0], -q[1], -q[2], -q[3]);
    }
    Quaternion Quaternion::inverse() const {
        float len = length();
        if(len < 1e-6f) {  // Защита от деления на очень маленькое число
            return Quaternion(1, 0, 0, 0);  // Возвращаем единичный кватернион
        }
        return conjugate() / len;
    }

    Vector Quaternion::rotate(const Vector& vector) const {
        Quaternion v(0.0f, vector.x(), vector.y(), vector.z());
        Quaternion conjugate = this->conjugate();
        Quaternion result = *this * v * conjugate;
        return Vector(result.x(), result.y(), result.z());
    }
    Vector Quaternion::operator*(const Vector& vector) const {
        return rotate(vector);
    }
    Quaternion& Quaternion::operator+=(const Quaternion& other) {
        q[0] += other.q[0];
        q[1] += other.q[1];
        q[2] += other.q[2];
        q[3] += other.q[3];
        return *this;
    }
    Quaternion& Quaternion::operator-=(const Quaternion& other) {
        q[0] -= other.q[0];
        q[1] -= other.q[1];
        q[2] -= other.q[2];
        q[3] -= other.q[3];
        return *this;
    }
    Quaternion& Quaternion::operator*=(float scalar) {
        q[0] *= scalar;
        q[1] *= scalar;
        q[2] *= scalar;
        q[3] *= scalar;
        return *this;
    }
    Quaternion& Quaternion::operator/=(float scalar) {
        q[0] /= scalar;
        q[1] /= scalar;
        q[2] /= scalar;
        q[3] /= scalar;
        return *this;
    }
    Quaternion& Quaternion::operator*=(const Quaternion& other) {
        *this = *this * other;
        return *this;
    }
    Quaternion& Quaternion::operator/=(const Quaternion& other) {
        *this = *this / other;
        return *this;
    }
    float Quaternion::length() const {
        return sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    }
    float Quaternion::dot(const Quaternion& other) const {
        return q[0]*other.q[0] + q[1]*other.q[1] + q[2]*other.q[2] + q[3]*other.q[3];
    }
    Quaternion Quaternion::normalize() const {
        float len = length();
        if(len < 1e-6f) {  // Защита от деления на очень маленькое число
            return Quaternion(1, 0, 0, 0);  // Возвращаем единичный кватернион
        }
        return Quaternion(q[0]/len, q[1]/len, q[2]/len, q[3]/len);
    }
    Quaternion Quaternion::slerp(const Quaternion& other, float t) const {
        float dotProduct = this->dot(other);
        
        // Обеспечиваем кратчайший путь
        float scale0, scale1;
        if (dotProduct < 0.0f) {
            dotProduct = -dotProduct;
            scale1 = -1.0f;
        } else {
            scale1 = 1.0f;
        }
        
        // Если кватернионы очень близки, используем линейную интерполяцию
        if (dotProduct > 0.9995f) {
            scale0 = 1.0f - t;
            scale1 *= t;
        } else {
            float theta = acos(dotProduct);
            float sinTheta = sin(theta);
            
            // Защита от деления на ноль
            if (sinTheta < 1e-6f) {
                return *this;  // или можно использовать nlerp
            }
            
            scale0 = sin((1.0f - t) * theta) / sinTheta;
            scale1 *= sin(t * theta) / sinTheta;
        }
        
        return scale0 * *this + scale1 * other;
    }
    Quaternion Quaternion::nlerp(const Quaternion& other, float t) const {
        return (1-t) * *this + t * other;
    }
    float Quaternion::yaw() const {
        // Поворот вокруг оси Z (yaw)
        return atan2(2 * (q[0]*q[3] + q[1]*q[2]), 
                    1 - 2 * (q[2]*q[2] + q[3]*q[3]));
    }
    float Quaternion::pitch() const {
        // Поворот вокруг оси Y (pitch)
        float sinp = 2 * (q[0]*q[2] - q[3]*q[1]);
        // Проверяем на граничные случаи (±90 градусов)
        if (fabs(sinp) >= 1)
            return copysign(M_PI / 2, sinp);
        else
            return asin(sinp);
    }
    float Quaternion::roll() const {
        // Поворот вокруг оси X (roll)
        return atan2(2 * (q[0]*q[1] + q[2]*q[3]),
                    1 - 2 * (q[1]*q[1] + q[2]*q[2]));
    }   
    Quaternion Quaternion::fromYPR(float yaw, float pitch, float roll) {
        float cy = cos(yaw * 0.5f);
        float sy = sin(yaw * 0.5f);
        float cp = cos(pitch * 0.5f);
        float sp = sin(pitch * 0.5f);
        float cr = cos(roll * 0.5f);
        float sr = sin(roll * 0.5f);
        
        return Quaternion(
            cr*cp*cy + sr*sp*sy,  // w
            sr*cp*cy - cr*sp*sy,  // x
            cr*sp*cy + sr*cp*sy,  // y
            cr*cp*sy - sr*sp*cy   // z
        );
    }
    Vector Quaternion::toYPR() const {
        return Vector(yaw(), pitch(), roll());
    }
    
    void Quaternion::toEuler(float& roll, float& pitch, float& yaw) const {
        roll = this->roll();
        pitch = this->pitch();
        yaw = this->yaw();
    }
    
    Quaternion Quaternion::fromEuler(float roll, float pitch, float yaw) {
        return fromYPR(yaw, pitch, roll);
    }
    
    Quaternion Quaternion::fromAxisAngle(const Vector& axis, float angle) {
        float halfAngle = angle / 2;
        float sinHalfAngle = sin(halfAngle);
        return Quaternion(
            cos(halfAngle),                 // w
            axis.x() * sinHalfAngle,        // x
            axis.y() * sinHalfAngle,        // y
            axis.z() * sinHalfAngle         // z
        );
    }


}