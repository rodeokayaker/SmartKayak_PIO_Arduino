/**
 * @file Types.h
 * @brief Central location for all common data types used across SmartKayak/SmartPaddle project
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#ifndef SMARTKAYAK_CORE_TYPES_H
#define SMARTKAYAK_CORE_TYPES_H

#ifdef ARDUINO
#include <Arduino.h>
#else
#define String std::string
#define uint32_t unsigned long
#define int16_t short
#define int32_t long
#define uint16_t unsigned short
#endif


// ============================================================================
// PADDLE TYPES
// ============================================================================

enum PaddleType {
    ONE_BLADE,
    TWO_BLADES
};

enum BladeSideType {
    RIGHT_BLADE,
    LEFT_BLADE,
    ALL_BLADES
};

enum AxisDirection {
    X_AXIS_RIGHT,    
    Y_AXIS_RIGHT,
    Z_AXIS_RIGHT,
    X_AXIS_LEFT,
    Y_AXIS_LEFT,
    Z_AXIS_LEFT,
    X_AXIS_FORWARD,
    Y_AXIS_FORWARD,
    X_AXIS_BACKWARD,
    Y_AXIS_BACKWARD,
};

// ============================================================================
// IMU DATA STRUCTURES
// ============================================================================

/**
 * @struct IMUData
 * @brief Complete IMU sensor data including accelerometer, gyroscope, magnetometer, and quaternion
 */
struct IMUData {
    // Accelerometer data (m/s²)
    float ax, ay, az;
    
    // Gyroscope data (rad/s)
    float gx, gy, gz;
    
    // Magnetometer data (µT)
    float mx, my, mz;
    
    // Raw magnetometer data (LSB) for compatibility
    int16_t mag_x, mag_y, mag_z;
    
    // Quaternion orientation (w, x, y, z) - from DMP if available
    float q0, q1, q2, q3;
    
    // Timestamp
    uint32_t timestamp;
};

/**
 * @struct OrientationData
 * @brief Orientation quaternion data (typically from sensor fusion)
 */
struct OrientationData {
    float q0, q1, q2, q3;
    uint32_t timestamp;
};

/**
 * @struct IMUCalibData
 * @brief IMU calibration data for all sensors
 */
struct IMUCalibData {
    // Accelerometer calibration
    int16_t accelOffset[3];
    float accelScale[3];
    
    // Gyroscope calibration
    int16_t gyroOffset[3];
    float gyroScale[3];
    
    // Magnetometer calibration
    float magOffset[3];
    float magScale[3];
    float magSI[3];  // Soft iron matrix
};

// ============================================================================
// LOAD CELL DATA STRUCTURES
// ============================================================================

/**
 * @struct loadData
 * @brief Load cell data for both left and right blades
 */
struct loadData {
    float forceR;
    float forceL;
    int32_t forceR_raw;
    int32_t forceL_raw;
    uint32_t timestamp;
};

/**
 * @struct loadCellSetCalibrationData
 * @brief Calibration data for load cell set
 */
struct loadCellSetCalibrationData {
    float scale[2];   // scale for right and left load cells
    float offset[2];  // offset for right and left load cells
};

// ============================================================================
// MOTOR TYPES
// ============================================================================

enum MotorDirection {
    REVERSE = -1,
    BRAKE = 0,
    FORWARD = 1
};

enum MotorPowerMode {
    MOTOR_OFF = 0,
    MOTOR_LOW_POWER = 1,
    MOTOR_MEDIUM_POWER = 2,
    MOTOR_HIGH_POWER = 3,
    MOTOR_DEBUG = 4
};

// ============================================================================
// LOGGING TYPES
// ============================================================================

enum LogMode {
    LOG_MODE_OFF = 0,
    LOG_MODE_PADDLE_MAG = 1,
    LOG_MODE_KAYAK_MAG = 2,
    LOG_MODE_DEBUG = 3,
    LOG_MODE_ALL = 4
};

static const int nLogModes = 5;

static const char* logModeNames[] = {
    "OFF",
    "PMAG",
    "KMAG",
    "DBG",
    "ALL"
};

// ============================================================================
// PADDLE STRUCTURES
// ============================================================================

struct BladeData {
    BladeSideType bladeSide;
    float quaternion[4];
    float force;
    uint32_t timestamp;
};

struct PaddleSpecs {
    String paddleID;
    PaddleType paddleType;
    float length;
    float imuDistance;
    float bladeWeight;
    float bladeCenter;
    float bladeMomentInertia;
    uint16_t firmwareVersion;
    String paddleModel;
    bool hasLeftBlade;
    bool hasRightBlade;
    uint16_t imuFrequency;
    AxisDirection axisDirection;
    int8_t axisDirectionSign;
};

struct BladeOrientation {
//    signed char YAxisDirection;
    float rightBladeAngle;
    float rightBladeVector[3];
    float leftBladeAngle;
    float leftBladeVector[3];
};

struct PaddleStatus {
    int8_t batteryLevel;
    int8_t temperature;
};

struct KayakSpecs {
    AxisDirection axisDirection;
};


#endif // SMARTKAYAK_CORE_TYPES_H

