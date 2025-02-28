#pragma once
#include <Arduino.h>

// Базовый интерфейс для отображения
class IDisplay {
public:
    virtual void begin() = 0;
    virtual void clear() = 0;
    virtual void update() = 0;
    virtual ~IDisplay() = default;
};

// Интерфейс для текстового дисплея
class ITextDisplay : public IDisplay {
public:
    virtual void setCursor(int x, int y) = 0;
    virtual void print(const char* text) = 0;
    virtual void printf(const char* format, ...) = 0;
    virtual int getColumns() const = 0;
    virtual int getRows() const = 0;
};

// Структура для хранения данных каяка
struct KayakDisplayData {
    // Углы
    float shaftRotationAngle;
    float shaftTiltAngle;
    float bladeRotationAngle;
    
    // Силы
    float leftForce;
    float rightForce;
    float leftTare;
    float rightTare;
    
    // Состояние весла
    bool isRightBlade;
    
    // Углы лопастей
    float leftBladeAngle;
    float rightBladeAngle;
    
    // Мотор
    int motorForce;
};

// Базовый класс для отображения данных каяка
class KayakDisplay {
protected:
    KayakDisplayData currentData;
    uint32_t updateInterval;
    uint32_t lastUpdate;

public:
    explicit KayakDisplay(uint32_t interval = 100) 
        : updateInterval(interval), lastUpdate(0) {}
    
    virtual void begin() = 0;
    virtual void update(const KayakDisplayData& data) {
        if (millis() - lastUpdate < updateInterval) {
            return;
        }
        currentData = data;
        updateDisplay();
        lastUpdate = millis();
    }
    
protected:
    virtual void updateDisplay() = 0;
}; 