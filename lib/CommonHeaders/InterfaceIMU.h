#ifndef INTERFACE_IMU_H
#define INTERFACE_IMU_H

#include <Arduino.h>

struct IMUCalibData {
    // Калибровочные данные акселерометра
    int16_t accelOffset[3];  // Смещения нуля по трем осям
    float accelScale[3];     // Масштабирующие коэффициенты для приведения к м/с²
    
    // Калибровочные данные гироскопа
    int16_t gyroOffset[3];   // Смещения нуля по трем осям
    float gyroScale[3];      // Масштабирующие коэффициенты для приведения к рад/с
    
    // Калибровочные данные магнитометра
    float magOffset[3];    // Смещения нуля по трем осям
    float magScale[3];       // Масштабирующие коэффициенты для нормализации
    float magSI[3];        // Soft iron matrix ({0,1} {0,2} {1,2})
};

// IMU data structure
struct IMUData {
    float ax, ay, az;    // Acceleration calibrated data
    float gx, gy, gz;      // Angular velocity calibrated data
    float mx, my, mz;         // Magnetic field calibrated data
    int16_t mag_x, mag_y, mag_z; // Magnetic field raw data
    float q0, q1, q2, q3;    // Quaternion if available
    uint32_t timestamp;               // Timestamp in milliseconds
};

struct OrientationData {
    float q0, q1, q2, q3;           // Madgwick quaternion
    uint32_t timestamp;               // Timestamp in milliseconds
};

class IIMU {
public:
    virtual bool readData() = 0;
    virtual OrientationData getOrientation() = 0; // from readed Data
    virtual IMUData getData() = 0;
    virtual OrientationData updateOrientation() = 0;
    virtual uint16_t getFrequency() = 0;
    virtual bool DMPEnabled() = 0;
    virtual int8_t interruptPIN() = 0;
    virtual uint16_t magnetometerFrequency() = 0;
    virtual void magnetometerUpdate() = 0;
    virtual void calibrate() = 0;
    virtual void calibrateCompass() = 0;
    virtual bool isCalibrationValid() = 0;
    virtual void setLogStream(Stream* stream = &Serial) = 0;
    virtual IMUCalibData getCalibrationData() = 0;
    virtual void setCalibrationData(const IMUCalibData data, bool save = false) = 0;
    virtual bool DMPValid() =0;
    virtual int8_t getIntStatus() = 0;
    virtual void setFrequency(uint16_t frequency) {};
    virtual void begin() {};
    virtual ~IIMU() = default;
};


#endif