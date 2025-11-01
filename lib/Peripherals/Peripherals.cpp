#include "Peripherals.h"

// Реализация LEDDriver
LEDDriver::LEDDriver(int pin) : 
    pin(pin), 
    is_on(false), 
    in_cycle(false),
    ledTaskHandle(nullptr),
    ledTimer(nullptr) {}

void LEDDriver::begin() {
    if (pin == -1) return;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    is_on = false;
    
    // Создаем задачу
    xTaskCreate(
        ledTask,
        "LED_Task",
        1024,
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
    if (pin == -1) return;
    if(ledTimer) {
        xTimerStop(ledTimer, 0);
    }
    digitalWrite(pin, HIGH);
    is_on = true;
    in_cycle = false;
}

void LEDDriver::off() {
    if (pin == -1) return;
    if(ledTimer) {
        xTimerStop(ledTimer, 0);
    }
    digitalWrite(pin, LOW);
    is_on = false;
    in_cycle = false;
}

void LEDDriver::Blink(uint32_t period_on_ms, uint32_t period_off_ms, 
                     uint32_t duration_ms, LED_MODE finish_mode) {
    if (pin == -1) return;
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
    if (pin == -1) return LED_OFF;
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
    if (timer == nullptr) return;
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
    longPressTimer(nullptr),
    debounceTimer(nullptr),
    long_press_fired(false),
    last_pin_state(HIGH) {}

void ButtonDriver::begin(int frequency) {
    if (pin == -1) return;
    main_frequency = frequency;
    pinMode(pin, INPUT_PULLUP);
    last_pin_state = digitalRead(pin);
    
    // Создаем таймер для длинного нажатия
    longPressTimer = xTimerCreate(
        "LongPress_Timer",
        pdMS_TO_TICKS(BUTTON_LONG_PRESS_TIME), 
        pdFALSE,  // Одноразовый таймер
        this,     // ID таймера - указатель на объект
        longPressTimerCallback
    );

    // Создаем таймер для защиты от дребезга
    debounceTimer = xTimerCreate(
        "Debounce_Timer",
        pdMS_TO_TICKS(main_frequency),
        pdFALSE,  // Одноразовый таймер
        this,     // ID таймера - указатель на объект
        debounceTimerCallback
    );
        
    // Настройка прерывания
    attachInterruptArg(
        digitalPinToInterrupt(pin),
        buttonISR,
        this,
        CHANGE
    );
}

void IRAM_ATTR buttonISR(void* arg) {
    ButtonDriver* button = (ButtonDriver*)arg;
    
    // Только планируем проверку состояния через таймер
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (button->debounceTimer) {
        xTimerStartFromISR(button->debounceTimer, &xHigherPriorityTaskWoken);
    }
}

void ButtonDriver::debounceTimerCallback(TimerHandle_t timer) {
    ButtonDriver* button = static_cast<ButtonDriver*>(pvTimerGetTimerID(timer));
    uint8_t current_pin_state = digitalRead(button->pin);
    
    // Проверяем, действительно ли изменилось состояние
    if (current_pin_state != button->last_pin_state) {
        button->last_pin_state = current_pin_state;
        
        if (current_pin_state == LOW) {
            // Кнопка нажата
            button->state = BUTTON_PRESSED;
            button->long_press_fired = false;
            button->onPress();
            
            // Запускаем таймер длинного нажатия
            if (button->longPressTimer) {
                xTimerStart(button->longPressTimer, 0);
            }
        } else {
            // Кнопка отпущена
            button->state = BUTTON_RELEASED;
            button->onRelease();
            
            // Останавливаем таймер длинного нажатия
            if (button->longPressTimer) {
                xTimerStop(button->longPressTimer, 0);
            }
        }
    }
}

void ButtonDriver::longPressTimerCallback(TimerHandle_t timer) {
    ButtonDriver* button = static_cast<ButtonDriver*>(pvTimerGetTimerID(timer));
    if (button->state == BUTTON_PRESSED && !button->long_press_fired) {
        button->long_press_fired = true;
        button->onLongPress();
    }
}
