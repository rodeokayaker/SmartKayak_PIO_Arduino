#include "SP_Vector.h"
#include "SP_Quaternion.h"
#include <cmath>

namespace SP_Math
{
    Vector::Vector() : v{0, 0, 0} {}
    Vector::~Vector() {}
    Vector::Vector(float x, float y, float z) : v{x, y, z} {}
    Vector::Vector(const Vector& other) : v{other.v[0], other.v[1], other.v[2]} {}
    Vector::Vector(const float* other) : v{other[0], other[1], other[2]} {}
    Vector& Vector::operator=(const Vector& other) {
        v[0] = other.v[0];
        v[1] = other.v[1];
        v[2] = other.v[2];
        return *this;
    }
    Vector Vector::operator+(const Vector& other) const {
        return Vector(v[0] + other.v[0], v[1] + other.v[1], v[2] + other.v[2]);
    }
    Vector Vector::operator-(const Vector& other) const {
        return Vector(v[0] - other.v[0], v[1] - other.v[1], v[2] - other.v[2]);
    }
    Vector Vector::operator*(float scalar) const {
        return Vector(v[0] * scalar, v[1] * scalar, v[2] * scalar);
    }
    Vector Vector::operator/(float scalar) const {
        return Vector(v[0] / scalar, v[1] / scalar, v[2] / scalar);
    }
    float Vector::operator*(const Vector& other) const {
        return v[0]*other.v[0] + v[1]*other.v[1] + v[2]*other.v[2];
    }
    float Vector::length() const {
        return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    }
    float Vector::dot(const Vector& other) const {
        return v[0]*other.v[0] + v[1]*other.v[1] + v[2]*other.v[2];
    }
    Vector Vector::cross(const Vector& other) const {
        return Vector(v[1]*other.v[2] - v[2]*other.v[1], v[2]*other.v[0] - v[0]*other.v[2], v[0]*other.v[1] - v[1]*other.v[0]);
    }
    Vector Vector::normalize() const {
        float len = length();
        return Vector(v[0] / len, v[1] / len, v[2] / len);
    }
    Vector Vector::rotate(const Vector& axis, float angle) const {
        Quaternion rotation(axis, angle);
        return rotation * (*this);
    }
    Vector Vector::operator+=(const Vector& other) {
        v[0] += other.v[0];
        v[1] += other.v[1];
        v[2] += other.v[2];
        return *this;
    }
    Vector Vector::operator-=(const Vector& other) {
        v[0] -= other.v[0];
        v[1] -= other.v[1];
        v[2] -= other.v[2];
        return *this;
    }
    Vector Vector::operator*=(float scalar) {
        v[0] *= scalar;
        v[1] *= scalar;
        v[2] *= scalar;
        return *this;
    }
    Vector Vector::operator/=(float scalar) {
        v[0] /= scalar;
        v[1] /= scalar;
        v[2] /= scalar;
        return *this;
    }
    float Vector::lengthSquared() const {
        return v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
    }
    Vector Vector::hadamard(const Vector& other) const {
        return Vector(v[0]*other.v[0], v[1]*other.v[1], v[2]*other.v[2]);
    }
    Vector operator*(float scalar, const Vector& vector) {
        return Vector(scalar*vector[0], scalar*vector[1], scalar*vector[2]);
    }

}
