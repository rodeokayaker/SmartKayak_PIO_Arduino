/**
 * @file SP_Types.h
 * @brief Базовые типы и структуры данных SmartPaddle
 */
#pragma once
#include <Arduino.h>

enum PaddleType {
    ONE_BLADE,
    TWO_BLADES
};

enum BladeSideType {
    RIGHT_BLADE,
    LEFT_BLADE,
    ALL_BLADES
};

struct BladeData {
    BladeSideType bladeSide;
    float quaternion[4];
    float force;
    uint32_t timestamp;
};

enum AxisDirection {
    X_AXIS_RIGHT,
    Y_AXIS_RIGHT,
    Z_AXIS_RIGHT,
    X_AXIS_LEFT,
    Y_AXIS_LEFT,
    Z_AXIS_LEFT,
};

struct PaddleSpecs {
 
    String paddleID;                // Paddle ID
    PaddleType paddleType;          // Blade type

    float length;                   // in meters
    float imuDistance;              // расстояние от центра весла до IMU в метрах

    float bladeWeight;        // вес лопасти в кг
    float bladeCenter;        // расстояние от конца весла до центра масс лопасти в метрах
    float bladeMomentInertia; // момент инерции лопасти кг*м^2

    uint16_t firmwareVersion;
    String paddleModel;
    bool hasLeftBlade;
    bool hasRightBlade;
    uint16_t imuFrequency;

    AxisDirection axisDirection; //What axis and where it is directed
};

struct BladeOrientation {
    signed char YAxisDirection;     // 1 if Y axis directs right, -1 if Y axis directs left
    float rightBladeAngle;         // in radians
    float rightBladeVector[3];     // normal vector in paddleOrientation
    float leftBladeAngle;          // in radians
    float leftBladeVector[3];      // normal vector in paddleOrientation
};

struct PaddleStatus {
    int8_t batteryLevel;
    int8_t temperature;
}; 