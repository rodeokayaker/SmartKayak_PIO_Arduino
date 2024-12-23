#include "Peripherals.h"

// Реализация LEDDriver
LEDDriver::LEDDriver(int pin) : 
    pin(pin), 
    is_on(false), 
    in_cycle(false),
    ledTaskHandle(nullptr),
    ledTimer(nullptr) {}

void LEDDriver::begin() {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    is_on = false;
    
    // Создаем задачу
    xTaskCreate(
        ledTask,
        "LED_Task",
        2048,
        this,
        1,
        &ledTaskHandle
    );
    
    // Создаем программный таймер
    ledTimer = xTimerCreate(
        "LED_Timer",
        pdMS_TO_TICKS(100), // Начальный период не важен
        pdFALSE,  // Одноразовый таймер
        this,     // ID таймера - указатель на объект
        timerCallback
    );
}

void LEDDriver::on() {
    if(ledTimer) {
        xTimerStop(ledTimer, 0);
    }
    digitalWrite(pin, HIGH);
    is_on = true;
    in_cycle = false;
}

void LEDDriver::off() {
    if(ledTimer) {
        xTimerStop(ledTimer, 0);
    }
    digitalWrite(pin, LOW);
    is_on = false;
    in_cycle = false;
}

void LEDDriver::Blink(uint32_t period_on_ms, uint32_t period_off_ms, 
                     uint32_t duration_ms, LED_MODE finish_mode) {
    if(!ledTimer) return;
    
    cycle_period_on = period_on_ms;
    cycle_period_off = period_off_ms;
    cycle_finish_mode = finish_mode;
    in_cycle = true;
    
    if(duration_ms > 0) {
        stop_cycle = millis() + duration_ms;
    } else {
        stop_cycle = 0;
    }
    
    // Запуск с включенного состояния
    digitalWrite(pin, HIGH);
    is_on = true;
    
    xTimerChangePeriod(ledTimer, pdMS_TO_TICKS(period_on_ms), 0);
    xTimerStart(ledTimer, 0);
}

LED_MODE LEDDriver::getMode() { 
    if(in_cycle) return LED_CYCLE;
    return is_on ? LED_ON : LED_OFF;
}

void LEDDriver::handleTimerEvent() {
    if(in_cycle) {
        if(stop_cycle > 0 && millis() >= stop_cycle) {
            in_cycle = false;
            xTimerStop(ledTimer, 0);
            if(cycle_finish_mode == LED_ON) {
                on();
            } else {
                off();
            }
            return;
        }
        
        // Переключение состояния
        is_on = !is_on;
        digitalWrite(pin, is_on ? HIGH : LOW);
        
        // Установка следующего периода
        uint32_t next_period = is_on ? cycle_period_on : cycle_period_off;
        xTimerChangePeriod(ledTimer, pdMS_TO_TICKS(next_period), 0);
    }
}

void LEDDriver::ledTask(void* parameter) {
    LEDDriver* led = static_cast<LEDDriver*>(parameter);
    while(1) {
        uint32_t notification;
        // Ждем уведомления от таймера
        if(xTaskNotifyWait(0, ULONG_MAX, &notification, portMAX_DELAY)) {
            led->handleTimerEvent();
        }
    }
}

void LEDDriver::timerCallback(TimerHandle_t timer) {
    LEDDriver* led = static_cast<LEDDriver*>(pvTimerGetTimerID(timer));
    // Отправляем уведомление задаче
    if(led->ledTaskHandle != nullptr) {
        xTaskNotifyFromISR(led->ledTaskHandle, 1, eSetValueWithOverwrite, nullptr);
    }
}

LEDDriver::~LEDDriver() {
    if(ledTimer) {
        xTimerDelete(ledTimer, 0);
    }
    if(ledTaskHandle) {
        vTaskDelete(ledTaskHandle);
    }
}

// Реализация ButtonDriver
ButtonDriver::ButtonDriver(int pin) : 
    pin(pin), 
    state(BUTTON_RELEASED), 
    last_state_change(0) {}

void ButtonDriver::begin(int frequency) {
    main_frequency = frequency;
    pinMode(pin, INPUT_PULLUP);
        
    // астройка прерывания
    attachInterruptArg(
        digitalPinToInterrupt(pin),
        buttonISR,
        this,
        CHANGE
    );
}

void ButtonDriver::update() {
    // В этой реализации основная обработка происходит в прерывании
}

void IRAM_ATTR buttonISR(void* arg) {
    ButtonDriver* button = (ButtonDriver*)arg;
    uint32_t now = millis();
    
    // Защита от дребезга
    if (now - button->last_state_change >= button->main_frequency) {
        button->last_state_change = now;
        
        if (digitalRead(button->getPin()) == LOW) {
            // Кнопка нажата
            button->state = BUTTON_PRESSED;
            button->press_start_time = now;  // Запоминаем время начала нажатия
            button->long_press_fired = false; // Сбрасываем флаг

            button->onPress();
        } else {
            // Кнопка отпущена
            // Проверка длительного нажатия
            if (button->state == BUTTON_PRESSED && !button->long_press_fired) {
                if (now - button->press_start_time >= 2000) { // 3 секунды
                    button->long_press_fired = true;
                    button->onLongPress();
                }
            }
            button->state = BUTTON_RELEASED;
            button->press_start_time = 0;
            button->onRelease();
        }
    }
    

}
