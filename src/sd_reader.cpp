#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Определение пинов для SD карты Amperka
#define SD_CS_PIN 5    // CS пин
#define SD_SCK_PIN 18  // SCK пин
#define SD_MISO_PIN 19 // MISO пин
#define SD_MOSI_PIN 23 // MOSI пин

// Структура для хранения параметров задачи
typedef struct {
    const char* filename;
    uint32_t interval;
} SDTaskParams;

// Задача записи на SD карту
void sdWriteTask(void* parameters) {
    SDTaskParams* params = (SDTaskParams*)parameters;
    
    while (1) {
        File dataFile = SD.open(params->filename, FILE_APPEND);
        if (dataFile) {
            // Записываем временную метку и какие-то данные
            dataFile.printf("%lu,Data\n", millis());
            dataFile.close();
            Serial.println("Data written to SD card");
        } else {
            Serial.println("Error opening file for writing");
        }
        
        vTaskDelay(pdMS_TO_TICKS(params->interval));
    }
}

// Задача чтения с SD карты
void sdReadTask(void* parameters) {
    const char* filename = (const char*)parameters;
    
    while (1) {
        File dataFile = SD.open(filename);
        if (dataFile) {
            while (dataFile.available()) {
                String line = dataFile.readStringUntil('\n');
                Serial.println(line);
            }
            dataFile.close();
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // Читаем каждые 5 секунд
    }
}

void setup() {
    Serial.begin(115200);

   // Настройка пинов
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH); // Отключаем карту по умолчанию
    Serial.println("SD Card initialization started");

    SPISettings spiSettings(10000000, MSBFIRST, SPI_MODE0); // 10MHz, стандартный режим

    Serial.println("SPI initialization started");
    // Инициализация SPI для SD карты
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN);
    Serial.println("SPI initialization finished");

//    SPI.beginTransaction(spiSettings);
    Serial.println("SPI transaction started");
    // Инициализация SD карты
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD Card initialization failed!");
        return;
    }
    Serial.println("SD Card initialized successfully");

    // Параметры для задачи записи
    static SDTaskParams writeParams = {
        .filename = "/data.txt",
        .interval = 1000 // Интервал записи 1 секунда
    };

    // Создание задач FreeRTOS
    xTaskCreate(
        sdWriteTask,    // Функция задачи
        "SD Write",     // Имя задачи
        4096,           // Размер стека
        &writeParams,   // Параметры
        1,              // Приоритет
        NULL           // Хендл задачи
    );

    xTaskCreate(
        sdReadTask,     // Функция задачи
        "SD Read",      // Имя задачи
        4096,           // Размер стека
        (void*)"/data.txt", // Параметры
        1,              // Приоритет
        NULL           // Хендл задачи
    );
}

void loop() {
    // В основном цикле ничего не делаем, так как используем RTOS
    vTaskDelay(pdMS_TO_TICKS(1000));
} 