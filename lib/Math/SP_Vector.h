#ifndef SP_VECTOR_H
#define SP_VECTOR_H

namespace SP_Math
{
    class Vector
    {
        float v[3];
    public:
        Vector();
        ~Vector();
        Vector(float x, float y, float z);
        Vector(const Vector& other);
        Vector(const float* other);
        Vector& operator=(const Vector& other);
        Vector operator+(const Vector& other) const;
        Vector operator-(const Vector& other) const;
        Vector operator*(float scalar) const;
        Vector operator/(float scalar) const;
        Vector operator+=(const Vector& other);
        Vector operator-=(const Vector& other);
        Vector operator*=(float scalar);
        Vector operator/=(float scalar);
        float operator*(const Vector& other) const;
        float length() const;
        float lengthSquared() const;
        float dot(const Vector& other) const;
        Vector cross(const Vector& other) const;
        Vector normalize() const;
        Vector rotate(const Vector& axis, float angle) const;
        inline float& operator[](int index){return v[index];};
        inline const float& operator[](int index) const {return v[index];};
        inline float& x(){return v[0];};
        inline float& y(){return v[1];};
        inline float& z(){return v[2];};
        inline const float& x() const {return v[0];};
        inline const float& y() const {return v[1];};
        inline const float& z() const {return v[2];};
        inline float& a(){return v[0];};
        inline float& b(){return v[1];};
        inline float& c(){return v[2];};
        inline const float& a() const {return v[0];};
        inline const float& b() const {return v[1];};
        inline const float& c() const {return v[2];};
        Vector hadamard(const Vector& other) const;
    };

    SP_Math::Vector operator*(float scalar, const SP_Math::Vector& vector);
}

#endif
