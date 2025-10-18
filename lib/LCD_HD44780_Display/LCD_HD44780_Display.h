#pragma once
#include "DisplayInterface.h"
#include <hd44780.h>

// Реализация для двух LCD HD44780
class DualLCDDisplay : public KayakDisplay {
private:
    hd44780* lcd1;
    hd44780* lcd2;
    
public:
    DualLCDDisplay(hd44780* display1, hd44780* display2, uint32_t interval = 100)
        : KayakDisplay(interval), lcd1(display1), lcd2(display2) {}
        
    void begin() override {
        if (lcd1) lcd1->begin(20, 4);  // Предполагаем 20x4 дисплей
        if (lcd2) lcd2->begin(20, 2);  // Предполагаем 20x2 дисплей
    }
    
protected:
    void updateDisplay() override {
        updateMainDisplay();
        updateSecondaryDisplay();
    }
    
private:
    void updateMainDisplay() {
        if (!lcd1) return;
        
        char buf[22];
        
        // Первая строка - значения тары
        lcd1->setCursor(0, 0);
        snprintf(buf, sizeof(buf), "%6d  TARE  %6d", 
            static_cast<int>(currentData.leftTare), 
            static_cast<int>(currentData.rightTare));
        lcd1->print(buf);
        
        // Вторая строка - текущие нагрузки
        lcd1->setCursor(0, 1);
        snprintf(buf, sizeof(buf), "%6d  LOAD  %6d",
            static_cast<int>(currentData.leftForce - currentData.leftTare),
            static_cast<int>(currentData.rightForce - currentData.rightTare));
        lcd1->print(buf);
        
        // Третья строка - углы и индикатор стороны
        float cosL = cos(currentData.leftBladeAngle);
        float cosR = cos(currentData.rightBladeAngle);
        lcd1->setCursor(0, 2);
        snprintf(buf, sizeof(buf), "%6.3f %sPR%s %6.3f",
            cosL,
            currentData.isRightBlade ? "  " : "<-",
            currentData.isRightBlade ? "->" : "  ",
            cosR);
        lcd1->print(buf);
        
        // Четвертая строка - сила мотора
        lcd1->setCursor(0, 3);
        snprintf(buf, sizeof(buf), "%6d Force %6d",
            currentData.motorForce,
            currentData.motorForce);
        lcd1->print(buf);
    }
    
    void updateSecondaryDisplay() {
        if (!lcd2) return;
        
        char buf[22];
        
        // Первая строка - углы
        lcd2->setCursor(0, 0);
        snprintf(buf, sizeof(buf), "%4d %4d %4d",
            static_cast<int>(currentData.shaftRotationAngle),
            static_cast<int>(currentData.shaftTiltAngle),
            static_cast<int>(currentData.bladeRotationAngle));
        lcd2->print(buf);
        
        // Вторая строка - углы лопастей
        int leftAngle = static_cast<int>(-currentData.leftBladeAngle * RAD_TO_DEG + 
                                       currentData.bladeRotationAngle + 90);
        int rightAngle = static_cast<int>(-currentData.rightBladeAngle * RAD_TO_DEG + 
                                        currentData.bladeRotationAngle + 90);
        
        // Нормализация углов
        leftAngle = normalizeAngle(leftAngle);
        rightAngle = normalizeAngle(rightAngle);
        
        lcd2->setCursor(0, 1);
        snprintf(buf, sizeof(buf), "%4d %s %4d",
            leftAngle,
            currentData.isRightBlade ? "right>" : "<left ",
            rightAngle);
        lcd2->print(buf);
    }
    
    int normalizeAngle(int angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}; 