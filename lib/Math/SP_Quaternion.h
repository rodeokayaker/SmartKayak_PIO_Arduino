#ifndef SP_QUATERNION_H
#define SP_QUATERNION_H

#include "SP_Vector.h"

#define RADIANS_TO_DEGREES 57.295779513082320876798154814105
#define DEGREES_TO_RADIANS 0.01745329251994329576921542398385

namespace SP_Math
{

    class Quaternion
    {
        float q[4];
    public:
        Quaternion();
        ~Quaternion();
        Quaternion(const Quaternion& other);
        Quaternion(const Vector& axis, float angle);
        Quaternion(float w, float x, float y, float z);
        Quaternion& operator=(const Quaternion& other);
        Quaternion operator*(const Quaternion& other) const;
        Quaternion operator/(const Quaternion& other) const;
        Quaternion operator+(const Quaternion& other) const;
        Quaternion operator-(const Quaternion& other) const;
        Quaternion operator*(float scalar) const;
        Quaternion operator/(float scalar) const;
        Quaternion conjugate() const;
        Quaternion inverse() const;
        inline float& operator[](int index){return q[index];};
        inline const float& operator[](int index) const {return q[index];};
        inline float& w(){return q[0];};
        inline float& x(){return q[1];};
        inline float& y(){return q[2];};
        inline float& z(){return q[3];};
        inline const float& w() const {return q[0];};
        inline const float& x() const {return q[1];};
        inline const float& y() const {return q[2];};
        inline const float& z() const {return q[3];};
        inline float& a(){return q[0];};
        inline float& b(){return q[1];};
        inline float& c(){return q[2];};
        inline float& d(){return q[3];};
        inline const float& a() const {return q[0];};
        inline const float& b() const {return q[1];};
        inline const float& c() const {return q[2];};
        inline const float& d() const {return q[3];};

        Vector rotate(const Vector& vector) const;
        Vector operator*(const Vector& vector) const;
        Quaternion& operator+=(const Quaternion& other);
        Quaternion& operator-=(const Quaternion& other);
        Quaternion& operator*=(float scalar);
        Quaternion& operator/=(float scalar);
        Quaternion& operator*=(const Quaternion& other);
        Quaternion& operator/=(const Quaternion& other);
        float length() const;
        float dot(const Quaternion& other) const;
        Quaternion normalize() const;
        Quaternion slerp(const Quaternion& other, float t) const;
        Quaternion nlerp(const Quaternion& other, float t) const;
        float yaw() const;
        float pitch() const;
        float roll() const;
        Quaternion fromYPR(float yaw, float pitch, float roll);
        Vector toYPR() const;
        void toEuler(float& roll, float& pitch, float& yaw) const;
        static Quaternion fromEuler(float roll, float pitch, float yaw);
        static Quaternion fromAxisAngle(const Vector& axis, float angle);
        
    };

    // Объявление внешнего оператора
    inline Quaternion operator*(float scalar, const Quaternion& q) {
        return q * scalar;  // Используем уже определенный оператор умножения справа
    }    
}

#endif