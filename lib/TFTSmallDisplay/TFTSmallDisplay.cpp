#include "TFTSmallDisplay.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "../Core/Interfaces/ILogger.h"

TFTSmallDisplay::TFTSmallDisplay(int frequency) : 
    KayakDisplay(1000/frequency), 
    tft(), 
    mode(0), 
    updateTFTTaskHandle(nullptr), 
    firstShow(true),
    debugScreen(false),
    debugData{0, 0, true, false}
{
    tftMutex = xSemaphoreCreateMutex();
    if (tftMutex == NULL) {
        Serial.println("Failed to create TFT mutex");
    }
}

void TFTSmallDisplay::begin() {
    Serial.println("TFTSmallDisplay::begin");
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    Serial.println("TFTSmallDisplay::begin: tft.init()");
    Serial.println("TFTSmallDisplay::begin: task created");

    firstShow = true;
}

void TFTSmallDisplay::update(const KayakDisplayData& data) {
    currentData = data;  // Сохраняем данные
}

void TFTSmallDisplay::clear() {
    if (xSemaphoreTake(tftMutex, portMAX_DELAY) == pdTRUE) {
        tft.fillScreen(TFT_BLACK);
        xSemaphoreGive(tftMutex);
    }
}

void TFTSmallDisplay::end() {
    if (updateTFTTaskHandle != nullptr) {
        vTaskDelete(updateTFTTaskHandle);
        updateTFTTaskHandle = nullptr;
    }
    if (tftMutex != nullptr) {
        vSemaphoreDelete(tftMutex);
        tftMutex = nullptr;
    }
}

TFTSmallDisplay::~TFTSmallDisplay() {
    end();
}

void TFTSmallDisplay::updateTFTTask(void* pvParameters) {
    TFTSmallDisplay* display = (TFTSmallDisplay*)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(display->updateInterval);
    
    while (true) {
        display->updateDisplay();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TFTSmallDisplay::showMotorForceScreen() {
    if (xSemaphoreTake(tftMutex, portMAX_DELAY) == pdTRUE) {
        tft.setCursor(0, 50);
        tft.setTextColor(TFT_WHITE);
        tft.setTextFont(2);
        tft.fillRect(0, 50, 160, 50, TFT_BLACK);
        tft.printf("Left force: %f\n", currentData.leftForce);
        tft.printf("Right force: %f\n", currentData.rightForce);
        tft.setTextFont(1);
        xSemaphoreGive(tftMutex);
    }
}

void TFTSmallDisplay::showOrientationScreen() { 
    if (xSemaphoreTake(tftMutex, portMAX_DELAY) == pdTRUE) {
        
        // Рабочая область экрана (с учетом статусных строк)
        int centerX = 80;  // 160/2
        int centerY = 64;  // 128/2
        int workHeight = 88; // 128 - 20 (верх) - 20 (низ)
        int workTop = 20;
        int radius = 30;
        

        if (firstShow) {
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_WHITE);
            tft.setTextFont(2);
            tft.drawString("ORIENTATION", 5, 5);
            
            // Рисуем каяк (статичные элементы)
            tft.drawLine(centerX, workTop + 10, centerX, workTop + workHeight - 10, TFT_BLUE);
            tft.drawLine(centerX - 5, workTop + 15, centerX, workTop + 10, TFT_BLUE);
            tft.drawLine(centerX + 5, workTop + 15, centerX, workTop + 10, TFT_BLUE);
            
            // Рисуем окружность для показа ориентации
            tft.drawCircle(centerX, centerY, radius, TFT_DARKGREY);
            
            // Рисуем компас (статичные элементы)
            tft.setTextFont(1);
            tft.setTextColor(TFT_CYAN);
            tft.drawString("N", centerX - 3, workTop + 2);
            tft.drawString("S", centerX - 3, workTop + workHeight - 10);
            tft.drawString("E", centerX + radius + 5, centerY - 3);
            tft.drawString("W", centerX - radius - 12, centerY - 3);
            
            firstShow = false;
        }
        
        // Получаем углы из currentData
        float shaftRotation = currentData.shaftRotationAngle;
        float shaftTilt = currentData.shaftTiltAngle;
        float bladeRotation = currentData.bladeRotationAngle;
        bool isRightBlade = currentData.isRightBlade;
        

        
        if (bladeRotation != lastData.bladeRotationAngle) {
            // Стираем старое положение весла
            if (!firstShow && (lastData.shaftRotationAngle != 0 || lastData.shaftTiltAngle != 0)) {
                // Стираем старую линию и точку
                tft.drawLine(oldLineCoord[0], oldLineCoord[1], oldLineCoord[2], oldLineCoord[3], TFT_BLACK);
//                tft.fillCircle(oldPaddleX, oldPaddleY, 3, TFT_BLACK);
            }
            
            // Рисуем новое положение весла

            float paddleEndX = centerX + radius * cos((shaftRotation) * M_PI / 180.0f);
            float paddleEndY = centerY - radius * sin((shaftRotation) * M_PI / 180.0f);
            float paddleStartX = centerX - radius * cos((shaftRotation) * M_PI / 180.0f);
            float paddleStartY = centerY + radius * sin((shaftRotation) * M_PI / 180.0f);

            oldLineCoord[0] = paddleStartX;
            oldLineCoord[1] = paddleStartY;
            oldLineCoord[2] = paddleEndX;
            oldLineCoord[3] = paddleEndY;
            
            uint16_t paddleColor = TFT_GREEN;
            
            // Рисуем новую линию весла и точку
            tft.drawLine(paddleStartX, paddleStartY, paddleEndX, paddleEndY, paddleColor);
        }
        
        // Обновляем отображение углов только если изменились
        if ((int)shaftRotation != (int)lastData.shaftRotationAngle || 
            (int)shaftTilt != (int)lastData.shaftTiltAngle) {
            
            // Стираем старые значения
            tft.fillRect(0, 106, 160, 10, TFT_BLACK);
            
            // Рисуем новые значения
            tft.setTextFont(1);
            tft.setTextColor(TFT_WHITE);
            tft.drawString("Rot: " + String((int)bladeRotation) + "", 5, 106);
            tft.drawString("Tilt: " + String((int)shaftTilt) + "", 85, 106);
        }
        

        if ((lastData.isRightBlade != isRightBlade)) {
            tft.fillRect(centerX - 30, workTop + workHeight - 20, 70, 12, TFT_BLACK);
            String direction = (isRightBlade) ? "RIGHT" : "LEFT";
            tft.setTextColor(TFT_YELLOW);
            tft.drawString(direction, centerX - 20, workTop + workHeight - 15);
        }

                // Отображение значений loadcell
        tft.fillRect(0, 75, 50, 25, TFT_BLACK);  // Очищаем область для loadcell
        tft.fillRect(110, 75, 50, 25, TFT_BLACK);  // Очищаем область для loadcell

        tft.setTextFont(1);
        tft.setTextColor(TFT_CYAN);
        
        // Левый loadcell
        tft.drawString(String((int)currentData.leftTare), 5, 80);
        tft.drawString(String((int)currentData.leftForce), 5, 90);
        
        // Правый loadcell  
        tft.drawString(String((int)currentData.rightTare), 115, 80);
        tft.drawString(String((int)currentData.rightForce), 115, 90);

        xSemaphoreGive(tftMutex);
    }
}

void TFTSmallDisplay::updateDisplay() {


    //showMotorForceScreen();
    if (debugScreen) {
        showDebugScreen();
    } else {
        showOrientationScreen();
    }
    showStatusLines();
    showModeLines();
    lastData = currentData;
}


void TFTSmallDisplay::showStatusLines() {
    if (xSemaphoreTake(tftMutex, portMAX_DELAY) == pdTRUE) {
        tft.fillRect(0, 0, 160, 20, TFT_BLACK);
        tft.setCursor(0, 0);
        tft.setTextColor(TFT_WHITE);
        tft.printf("Motor: %9d  %8d\n", motorDriver?motorDriver->getForce():0, millis());
        tft.setCursor(0, 10);
        tft.printf("Paddle: %s\n", currentData.isPaddleConnected?"Connected":"Disconnected");
        xSemaphoreGive(tftMutex);
    }
}

void TFTSmallDisplay::showModeLines() {
    if (xSemaphoreTake(tftMutex, portMAX_DELAY) == pdTRUE) {
//        tft.fillRect(0, 118, 160, 128, TFT_BLACK);
        tft.setCursor(0, 118);
        tft.setTextColor(TFT_WHITE);
        tft.printf("Mode: ");
        switch (motorSwitch?motorSwitch->getMode():MOTOR_OFF) {
            case MOTOR_OFF:
                tft.fillRect(30, 116, 50, 20, TFT_BLACK);
                tft.setTextColor(TFT_WHITE);
                tft.printf("OFF\n");
                break;
            case MOTOR_LOW_POWER:
                tft.fillRect(30, 116, 50, 20, TFT_GREEN);
                tft.setTextColor(TFT_BLACK);
                tft.printf("LOW\n");
                break;
            case MOTOR_MEDIUM_POWER:
                tft.fillRect(30, 116, 50, 20, TFT_YELLOW);
                tft.setTextColor(TFT_BLACK);
                tft.printf("MED\n");
                break;
            case MOTOR_HIGH_POWER:
                tft.fillRect(30, 116, 50, 20, TFT_RED);
                tft.setTextColor(TFT_WHITE);
                tft.printf("HIGH\n");
                break;
            case MOTOR_DEBUG:
                tft.fillRect(30, 116, 50, 20, TFT_WHITE);
                tft.setTextColor(TFT_BLACK);
                tft.printf("DEBUG\n");
                break;
        }
        tft.setCursor(100, 118);
        if (logSwitch&&logSwitch->getLogStarted()) {
            tft.fillRect(126, 116, 30, 20, TFT_BLUE);
        } else {
            tft.fillRect(126, 116, 30, 20, TFT_BLACK);
        }
        tft.setTextColor(TFT_WHITE);
        tft.printf("Log: %s\n", logModeNames[logSwitch?logSwitch->getLogMode():LOG_MODE_OFF]);
        xSemaphoreGive(tftMutex);
    }
}

void TFTSmallDisplay::startTasks() {
    xTaskCreate(updateTFTTask, "TFT Task", 2048, this, 5, &updateTFTTaskHandle);
}

void TFTSmallDisplay::setDebugData(int force, int load, bool scn) {
    debugData.force = force;
    debugData.load = load;
    debugData.scn = scn;
}

void TFTSmallDisplay::switchDebugScreen(bool on) {
    debugData.firstShow = true;
    debugScreen = on;
    firstShow = true;
}

void TFTSmallDisplay::showDebugScreen() {
    if (xSemaphoreTake(tftMutex, portMAX_DELAY) == pdTRUE) {
        if (debugData.firstShow) {
            tft.fillScreen(TFT_BLACK);
            debugData.firstShow = false;
        }
        tft.fillRect(45, 56, 160, 32, TFT_BLACK);
        tft.setCursor(0, 40);
        if (debugData.scn) {
            tft.setTextColor(TFT_YELLOW);
        } else {
            tft.setTextColor(TFT_WHITE);
        }
        tft.setTextFont(2);
        tft.printf("DEBUG\nFORCE: %d\nLOAD:  %d\n", debugData.force, debugData.load);

        tft.setTextFont(1);
        tft.setTextColor(TFT_WHITE);
        xSemaphoreGive(tftMutex);
    }
}
