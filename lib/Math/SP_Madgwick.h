#ifndef SP_Madgwick_h
#define SP_Madgwick_h
#include "SP_Quaternion.h"

namespace SP_Math
{

class Madgwick{
private:
    static float invSqrt(float x);
    float beta;				
    float q0, q1, q2, q3;
    float invSampleFreq;
    float roll;
    float pitch;
    float yaw;
    char anglesComputed;
    void computeAngles();

public:
    Madgwick(void);
    void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    float getRoll() {
        if (!anglesComputed) computeAngles();
        return roll * RADIANS_TO_DEGREES;
    }
    float getPitch() {
        if (!anglesComputed) computeAngles();
        return pitch * RADIANS_TO_DEGREES;
    }
    float getYaw() {
        if (!anglesComputed) computeAngles();
        return yaw * RADIANS_TO_DEGREES + 180.0f;
    }
    float getRollRadians() {
        if (!anglesComputed) computeAngles();
        return roll;
    }
    float getPitchRadians() {
        if (!anglesComputed) computeAngles();
        return pitch;
    }
    float getYawRadians() {
        if (!anglesComputed) computeAngles();
        return yaw;
    }
    inline void getQuaternion(float *q) {
        q[0] = q0;
        q[1] = q1;
        q[2] = q2;
        q[3] = q3;
    }

    inline Quaternion getQuaternion() {
        return Quaternion(q0, q1, q2, q3);
    }

    inline float* getQuaternionArray() {
        return &q0;
    }

    inline float getBeta() { return beta; }
    inline void setBeta(float b) { beta = b; }

};

}
#endif
