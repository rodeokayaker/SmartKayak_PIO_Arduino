#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include "ChinaMotor.h"
#include "Peripherals.h"

// Пины
#define BUTTON1_PIN 17
#define BUTTON2_PIN 18
#define MOTOR_PIN 32
#define LOW_LED_PIN 23
#define MED_LED_PIN 25
#define HIGH_LED_PIN 26
#define LOG_LED_PIN 27

// I2C пины
#define I2C_SDA 21
#define I2C_SCL 22

// LCD параметры
#define LCD_ADDR 0x27
#define LCD_COLS 20
#define LCD_ROWS 4

// Глобальные объекты
hd44780_I2Cexp lcd(LCD_ADDR);
ChinaMotor motor(MOTOR_PIN);
LEDDriver lowLed(LOW_LED_PIN);
LEDDriver medLed(MED_LED_PIN);
LEDDriver hiLed(HIGH_LED_PIN);
LEDDriver logLed(LOG_LED_PIN);

// Текущее значение
volatile int currentForce = 1500;

class DebugButton : public ButtonDriver {
private:
    int increment;
public:
    DebugButton(int pin, int inc) : ButtonDriver(pin), increment(inc) {}
    
    void onPress() override {
        int newForce = currentForce + increment;
        
        // Проверка границ
        if (newForce >= 1000 && newForce <= 2000) {
            currentForce = newForce;
        }
    }
    
    void onRelease() override {}
    void onLongPress() override {}
};

DebugButton button1(BUTTON1_PIN, 50);  // Увеличение
DebugButton button2(BUTTON2_PIN, -50); // Уменьшение

// Задача обновления мотора
void motorTask(void *pvParameters) {
    while(1) {
        motor.runRaw(currentForce);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Задача обновления дисплея
void displayTask(void *pvParameters) {
    while(1) {
        lcd.setCursor(0, 0);
        lcd.print("Motor Debug Mode    ");
        
        lcd.setCursor(0, 1);
        char buf[21];
        snprintf(buf, sizeof(buf), "Force: %4d         ", currentForce);
        lcd.print(buf);
        
        // Обновление светодиодов
        if (currentForce == 1500) {
            lowLed.off();
            medLed.on();
            hiLed.off();
        } else if (currentForce < 1500) {
            lowLed.on();
            medLed.off();
            hiLed.off();
        } else {
            lowLed.off();
            medLed.off();
            hiLed.on();
        }
        
        // Предельные значения
        if (currentForce <= 1000 || currentForce >= 2000) {
            logLed.on();
        } else {
            logLed.off();
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup() {
    Serial.begin(115200);
    
    // Инициализация I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    
    // Инициализация LCD
    int status = lcd.begin(LCD_COLS, LCD_ROWS);
    if(status) {
        Serial.printf("LCD initialization failed: %d\n", status);
    }
    lcd.clear();
    
    // Инициализация компонентов
    motor.begin();
    button1.begin();
    button2.begin();
    lowLed.begin();
    medLed.begin();
    hiLed.begin();
    logLed.begin();
    
    // Создание задач
    xTaskCreatePinnedToCore(
        motorTask,
        "Motor",
        2048,
        NULL,
        2,
        NULL,
        1
    );
    
    xTaskCreatePinnedToCore(
        displayTask,
        "Display",
        2048,
        NULL,
        1,
        NULL,
        1
    );
    
    Serial.println("Motor Debug System Ready!");
}

void loop() {
    vTaskDelete(NULL);
} 