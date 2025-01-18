#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Настройка пинов UART для GPS
#define GPS_RX_PIN 16  // Пин RX ESP32 подключается к TX GPS модуля
#define GPS_TX_PIN 15  // Пин TX ESP32 подключается к RX GPS модуля
#define GPS_BAUD 9600  // Стандартная скорость NEO-M8N

// Создаем объекты для работы с GPS
TinyGPSPlus gps;
HardwareSerial GPSSerial(2); // Используем UART1

// Структура для хранения GPS данных
struct GPSData {
    double latitude;
    double longitude;
    double altitude;
    int satellites;
    int hour;
    int minute;
    int second;
    int day;
    int month;
    int year;
};

// Очередь для передачи данных между задачами
QueueHandle_t gpsQueue;

// Задача чтения данных с GPS
void gpsReadTask(void *parameter) {
    while (1) {
        while (GPSSerial.available() > 0) {
//            Serial.println("GPS data available");
            char c=GPSSerial.read();
            Serial.print(c);
            if (gps.encode(c)) {
                if (gps.location.isValid() && gps.altitude.isValid() && gps.time.isValid()) {
                    GPSData data = {
                        .latitude = gps.location.lat(),
                        .longitude = gps.location.lng(),
                        .altitude = gps.altitude.meters(),
                        .satellites = gps.satellites.value(),
                        .hour = gps.time.hour(),
                        .minute = gps.time.minute(),
                        .second = gps.time.second(),
                        .day = gps.date.day(),
                        .month = gps.date.month(),
                        .year = gps.date.year()
                    };
                    
                    // Отправляем данные в очередь
                    xQueueSend(gpsQueue, &data, 0);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Небольшая задержка
    }
}

// Задача обработки и вывода данных
void gpsProcessTask(void *parameter) {
    GPSData data;
    
    while (1) {
        if (xQueueReceive(gpsQueue, &data, portMAX_DELAY) == pdTRUE) {
            // Форматируем и выводим данные
            Serial.println("GPS Data:");
            Serial.printf("DateTime: %02d-%02d-%02d %02d:%02d:%02d\n",
                data.year, data.month, data.day,
                data.hour, data.minute, data.second);
            Serial.printf("Lat: %.6f, Lon: %.6f\n", data.latitude, data.longitude);
            Serial.printf("Alt: %.2fm, Satellites: %d\n", data.altitude, data.satellites);
            Serial.println("------------------------");
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Обновляем раз в секунду
    }
}

void setup() {
    // Инициализация последовательных портов
    Serial.begin(115200);  // Отладочный порт
    GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); // GPS порт
    
    // Создаем очередь для GPS данных
    gpsQueue = xQueueCreate(10, sizeof(GPSData));
    
    if (gpsQueue == NULL) {
        Serial.println("Error creating GPS queue");
        return;
    }

    // Создаем задачи
    xTaskCreate(
        gpsReadTask,    // Функция задачи
        "GPS Read",     // Имя задачи
        4096,           // Размер стека
        NULL,           // Параметры
        2,              // Приоритет
        NULL           // Хендл задачи
    );

    xTaskCreate(
        gpsProcessTask, // Функция задачи
        "GPS Process",  // Имя задачи
        4096,           // Размер стека
        NULL,           // Параметры
        1,              // Приоритет
        NULL           // Хендл задачи
    );

    Serial.println("GPS Tasks started");
}

void loop() {
    // В основном цикле ничего не делаем, так как используем RTOS
    vTaskDelay(pdMS_TO_TICKS(1000));
} 