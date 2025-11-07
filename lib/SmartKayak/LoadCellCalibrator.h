#pragma once
#include "../Core/Interfaces/IIMUSensor.h"
#include "../Math/SP_Vector.h"

class LoadCellCalibrator {
private:
    struct TareData {
        double sum;
        int samples;
        double average;
        
        TareData() : sum(0), samples(0), average(0) {}
    };

    TareData leftTare;
    TareData rightTare;
    
    // Параметры весла
    float paddleLength;       // длина весла в метрах
    float imuDistance;        // расстояние от датчика до середины весла в метрах (справа положительно, слева отрицательно)
    float bladeWeight;        // вес лопасти в кг
    float bladeCenter;        // расстояние от конца весла до центра масс лопасти в метрах
    float bladeMomentInertia; // момент инерции лопасти кг*м^2
    
    static constexpr int SAMPLES_THRESHOLD = 10;
    static constexpr double ALPHA_LEFT = 0.02;   
    static constexpr double ALPHA_RIGHT = 0.02;  

    // Вычисление инерционных эффектов
    float calculateInertialEffects(const IMUData& imuData, const SP_Math::Vector& angularAcceleration,
                                 const BladeOrientation& bladeOrientation, 
                                 bool isRightBlade) const;

    // Вычисление гироскопического эффекта
    float calculateGyroscopicEffect(const IMUData& imuData, const SP_Math::Vector& angularAcceleration,
                                  const BladeOrientation& bladeOrientation,
                                  bool isRightBlade) const;

public:
    LoadCellCalibrator(float paddleLength = 2.2f, 
                      float imuDistance = 0.0f,
                      float bladeWeight = 0.3f, 
                      float bladeCenter = 0.20f,
                      float bladeMomentInertia = 0.01f);
    
    void setPaddleParameters(float length, float imuDist, float weight, float center, float momentInertia);
    
    void updateTare(bool isLeftBlade, const loadData& loadData,
                   const IMUData& imuData, const SP_Math::Vector& angularAcceleration, const BladeOrientation& bladeOrientation);
    
    double getCalibratedForce(bool isRightBlade, double rawForce, 
                            const IMUData& imuData, const SP_Math::Vector& angularAcceleration,
                            const BladeOrientation& bladeOrientation) const;

    double getCalibratedForce(bool isRightBlade, double rawForce) const;
    
    double getLeftTare() const { return leftTare.average; }
    double getRightTare() const { return rightTare.average; }
    void resetTare() { leftTare.average = 0; rightTare.average = 0; leftTare.samples = 0; rightTare.samples = 0; leftTare.sum = 0; rightTare.sum = 0; }

    // Расчет момента инерции для прямоугольной лопасти
    static float calculateSimpleBladeMomentInertia(
        float bladeWidth,    // ширина лопасти в метрах
        float bladeLength,   // длина лопасти в метрах
        float bladeMass      // масса лопасти в кг
    ) {
        return (1.0f/12.0f) * bladeMass * (bladeWidth*bladeWidth + bladeLength*bladeLength);
    }
    
    // Расчет момента инерции по периоду колебаний
    static float calculateMomentInertiaFromOscillation(
        float mass,          // масса лопасти в кг
        float length,        // расстояние от оси вращения до центра масс в метрах
        float period         // период колебаний в секундах
    ) {
        const float g = 9.81f;
        return (mass * g * length * period * period) / (4 * PI * PI);
    }
    
    // Установка момента инерции напрямую (например, из CAD)
    void setBladeMomentInertia(float momentInertia) {
        bladeMomentInertia = momentInertia;
    }
}; 