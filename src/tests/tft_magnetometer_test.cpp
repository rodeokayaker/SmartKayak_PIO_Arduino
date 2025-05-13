/**
 * Тестовый сценарий для TFT дисплея и магнитометров
 * 
 * Отображает данные с трех магнитометров на TFT экране:
 * - MPU-9250/6500 (через интерфейс MPU)
 * - GY-85 (используя HMC5883L)
 * - BMM150
 * 
 * TFT экран подключен через SPI интерфейс, магнитометры через I2C
 */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <MPU6050.h>
#include <HMC5883L.h>
#include <bmm150.h>
#include <bmm150_defs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


// Константы для TFT дисплея
#define TFT_CS    5
#define TFT_DC    21
#define TFT_MOSI  23
#define TFT_CLK   18
#define TFT_RST   22
#define TFT_MISO  19

// Константы для сенсорного экрана
#define TOUCH_CS  4  // T_CS
#define TOUCH_IRQ 36 // T_IRQ
// Аналоговые пины для TouchScreen
#define YP A2  // T_DIN
#define XP A1  // T_CLK
#define YM A3  // T_DO
#define XM A4  // для совместимости с API

// Параметры калибровки сенсорного экрана
#define TS_MINX 150
#define TS_MINY 120
#define TS_MAXX 920
#define TS_MAXY 940
#define MINPRESSURE 10
#define MAXPRESSURE 1000

// I2C пины
#define SDA_PIN 16
#define SCL_PIN 17

#include <TFT_eSPI.h>


// Создаем объекты
TFT_eSPI tft = TFT_eSPI();

// Объекты магнитометров
MPU6050 mpu;
HMC5883L hmc;
BMM150 bmm150;

// Структура для хранения данных магнитометров
struct MagData {
    float x, y, z;
    char name[10];
};

// Глобальные переменные для обмена данными между задачами
MagData mpuMagData = {0, 0, 0, "MPU9250"};
MagData hmcMagData = {0, 0, 0, "GY-85"};
MagData bmmMagData = {0, 0, 0, "BMM150"};

// Мьютексы для безопасного доступа к данным
SemaphoreHandle_t mpuMutex;
SemaphoreHandle_t hmcMutex;
SemaphoreHandle_t bmmMutex;

// Задача чтения данных с MPU9250
void mpuTask(void *pvParameters) {
    // Инициализация MPU
    mpu.initialize();
    
    // Проверка соединения
    if (!mpu.testConnection()) {
        ESP_LOGE("MPU", "MPU соединение не установлено");
        vTaskDelete(NULL);
    }
    
    ESP_LOGI("MPU", "MPU инициализирован успешно");
    
    while (true) {
        // Чтение данных с магнитометра будет через поллинг
        // поскольку встроенный магнитометр в MPU9250 доступен через AK8963
        // для простоты используем случайные значения для теста
        
        // Захватываем мьютекс перед изменением общих данных
        if (xSemaphoreTake(mpuMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            mpuMagData.x = random(-100, 100) / 10.0;
            mpuMagData.y = random(-100, 100) / 10.0;
            mpuMagData.z = random(-100, 100) / 10.0;
            
            // Освобождаем мьютекс
            xSemaphoreGive(mpuMutex);
        }
        
        // Задержка перед следующим чтением
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Задача чтения данных с HMC5883L (GY-85)
void hmcTask(void *pvParameters) {
    // Инициализация HMC5883L
    hmc.initialize();
    
    // Проверка соединения
    if (!hmc.testConnection()) {
        ESP_LOGE("HMC", "HMC5883L соединение не установлено");
        vTaskDelete(NULL);
    }
    
    ESP_LOGI("HMC", "HMC5883L инициализирован успешно");
    
    while (true) {
        // Чтение данных с магнитометра HMC5883L
        int16_t rawX, rawY, rawZ;
        
        hmc.getHeading(&rawX, &rawY, &rawZ);
        
        // Захватываем мьютекс перед изменением общих данных
        if (xSemaphoreTake(hmcMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            hmcMagData.x = rawX / 16.0; // Нормализация значений
            hmcMagData.y = rawY / 16.0;
            hmcMagData.z = rawZ / 16.0;
            
            // Освобождаем мьютекс
            xSemaphoreGive(hmcMutex);
        }
        
        // Задержка перед следующим чтением
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Задача чтения данных с BMM150
void bmmTask(void *pvParameters) {
    // Инициализация BME680 (в этом примере используем как заглушку для BMM150)
    // В реальном коде нужно использовать правильную библиотеку BMM150
    bmm150.initialize(); // Типичный адрес BME680
    
    ESP_LOGI("BMM", "BMM150 инициализирован (симуляция)");
    
    while (true) {
        bmm150.read_mag_data();
        
        // Захватываем мьютекс перед изменением общих данных
        if (xSemaphoreTake(bmmMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            bmmMagData.x = bmm150.raw_mag_data.raw_datax;
            bmmMagData.y = bmm150.raw_mag_data.raw_datay;
            bmmMagData.z = bmm150.raw_mag_data.raw_dataz;
            
            // Освобождаем мьютекс
            xSemaphoreGive(bmmMutex);
        }
        
        // Задержка перед следующим чтением
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Задача обновления дисплея
void displayTask(void *pvParameters) {
    // Инициализация дисплея с использованием TFT_eSPI
    tft.init();
    tft.setRotation(2); // Устанавливаем ориентацию дисплея
    
    // Очистка дисплея
    tft.fillScreen(TFT_BLACK);
    
    // Заголовок
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(20, 10);
    tft.println("Тест магнитометров");
    
    // Разделительная линия
    tft.drawLine(0, 30, 240, 30, TFT_WHITE);
    
    ESP_LOGI("TFT", "Дисплей инициализирован");
    
    // Локальные копии данных для отображения
    MagData localMpuData, localHmcData, localBmmData;
    
    while (true) {
        // Получаем данные через мьютексы
        if (xSemaphoreTake(mpuMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memcpy(&localMpuData, &mpuMagData, sizeof(MagData));
            xSemaphoreGive(mpuMutex);
        }
        
        if (xSemaphoreTake(hmcMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memcpy(&localHmcData, &hmcMagData, sizeof(MagData));
            xSemaphoreGive(hmcMutex);
        }
        
        if (xSemaphoreTake(bmmMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memcpy(&localBmmData, &bmmMagData, sizeof(MagData));
            xSemaphoreGive(bmmMutex);
        }
        
        // Очищаем область данных
        tft.fillRect(0, 40, 240, 200, TFT_BLACK);
        
        // Отображаем данные MPU
        tft.setTextColor(TFT_YELLOW);
        tft.setTextSize(1);
        tft.setCursor(10, 50);
        tft.print(localMpuData.name);
        
        tft.setTextColor(TFT_WHITE);
        tft.setCursor(10, 65);
        tft.print("X: ");
        tft.print(localMpuData.x, 2);
        tft.setCursor(10, 80);
        tft.print("Y: ");
        tft.print(localMpuData.y, 2);
        tft.setCursor(10, 95);
        tft.print("Z: ");
        tft.print(localMpuData.z, 2);
        
        // Отображаем данные HMC
        tft.setTextColor(TFT_GREEN);
        tft.setCursor(10, 120);
        tft.print(localHmcData.name);
        
        tft.setTextColor(TFT_WHITE);
        tft.setCursor(10, 135);
        tft.print("X: ");
        tft.print(localHmcData.x, 2);
        tft.setCursor(10, 150);
        tft.print("Y: ");
        tft.print(localHmcData.y, 2);
        tft.setCursor(10, 165);
        tft.print("Z: ");
        tft.print(localHmcData.z, 2);
        
        // Отображаем данные BMM
        tft.setTextColor(TFT_CYAN);
        tft.setCursor(10, 190);
        tft.print(localBmmData.name);
        
        tft.setTextColor(TFT_WHITE);
        tft.setCursor(10, 205);
        tft.print("X: ");
        tft.print(localBmmData.x, 2);
        tft.setCursor(10, 220);
        tft.print("Y: ");
        tft.print(localBmmData.y, 2);
        tft.setCursor(10, 235);
        tft.print("Z: ");
        tft.print(localBmmData.z, 2);
        
        // Проверяем сенсорный экран, если он включен
        uint16_t touchX = 0, touchY = 0;
        bool touched = tft.getTouch(&touchX, &touchY);
        
        if (touched) {
            // Отображаем точку касания
            tft.fillCircle(touchX, touchY, 5, TFT_RED);
            ESP_LOGI("TOUCH", "Нажатие на экран: x=%d, y=%d", touchX, touchY);
        }
        
        // Задержка перед следующим обновлением экрана
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void setup() {
    // Инициализация последовательного порта
    Serial.begin(115200);
    delay(1000);
    ESP_LOGI("MAIN", "Начало инициализации");
    
    // Инициализация I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000); // 400kHz
    
    // Инициализация мьютексов
    mpuMutex = xSemaphoreCreateMutex();
    hmcMutex = xSemaphoreCreateMutex();
    bmmMutex = xSemaphoreCreateMutex();
    
    // Создание задач FreeRTOS
    xTaskCreate(mpuTask, "MPU Task", 2048, NULL, 2, NULL);
    xTaskCreate(hmcTask, "HMC Task", 2048, NULL, 2, NULL);
    xTaskCreate(bmmTask, "BMM Task", 2048, NULL, 2, NULL);
    xTaskCreate(displayTask, "Display Task", 4096, NULL, 1, NULL);
    
    ESP_LOGI("MAIN", "Все задачи запущены");
}

void loop() {
    // Основной цикл Arduino не используется
    vTaskDelay(portMAX_DELAY);
} 