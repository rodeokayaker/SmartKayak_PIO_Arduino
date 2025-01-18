#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <Arduino.h>

// Определения для LED
enum LED_MODE {
    LED_OFF,
    LED_ON,
    LED_CYCLE
};

class LEDDriver {
protected:
    const int pin;
    bool is_on;
    bool in_cycle;
    uint32_t cycle_period_on;
    uint32_t cycle_period_off;
    uint32_t stop_cycle;
    LED_MODE cycle_finish_mode;
    TaskHandle_t ledTaskHandle;
    TimerHandle_t ledTimer;
    
    // Обработка события таймера (вызывается из задачи)
    void handleTimerEvent();
    
    // Статический метод для задачи FreeRTOS
    static void ledTask(void* parameter);
    
    // Статический колбэк для таймера
    static void timerCallback(TimerHandle_t timer);

public:
    LEDDriver(int pin);
    void begin();
    virtual void on();
    virtual void off();
    void Blink(uint32_t period_on_ms, uint32_t period_off_ms, 
               uint32_t duration_ms = 0, LED_MODE finish_mode = LED_CYCLE);
    bool isOn() { return is_on; }
    LED_MODE getMode();
    ~LEDDriver();
};

// Определения для кнопки
enum BUTTON_STATE {
    BUTTON_PRESSED,
    BUTTON_RELEASED
};

class ButtonDriver {
protected:
    int pin;
    BUTTON_STATE state;
    int main_frequency;
    TimerHandle_t longPressTimer;  // Таймер для длинного нажатия
    TimerHandle_t debounceTimer;   // Таймер для защиты от дребезга
    bool long_press_fired;      // Флаг срабатывания длительного нажатия
    uint8_t last_pin_state;     // Последнее состояние пина
    static void longPressTimerCallback(TimerHandle_t timer);
    static void debounceTimerCallback(TimerHandle_t timer);

public:
    ButtonDriver(int pin);
    void begin(int frequency=10);
    bool isPressed() { return state == BUTTON_PRESSED; }
    BUTTON_STATE getState() { return state; }
    int getPin() { return pin; }
    int getFrequency() { return main_frequency; }
    bool isLongPressed() { return long_press_fired; }
    
    // Виртуальные функции для обработки событий
    virtual void onPress() =0;
    virtual void onRelease() =0;
    virtual void onLongPress() =0;
    
    friend void IRAM_ATTR buttonISR(void* arg);
    virtual ~ButtonDriver() = default;
    

};

// Обработчик прерывания для кнопки
void IRAM_ATTR buttonISR(void* arg);

#endif