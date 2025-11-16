#include "TFTSmallDisplay.h"
#include "SmartKayak.h"
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
    debugData{0, 0, true, false},
    kayak(nullptr)
{
    tftMutex = xSemaphoreCreateMutex();
    if (tftMutex == NULL) {
        Serial.println("Failed to create TFT mutex");
    }
}

void TFTSmallDisplay::begin() {
    Serial.println("TFTSmallDisplay::begin");
    #if defined(TFT_BL)
    pinMode(TFT_BL, OUTPUT);
  #if defined(TFT_BACKLIGHT_ON)
    digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);
  #else
    digitalWrite(TFT_BL, HIGH);
  #endif
#endif

#if defined(TFT_SCLK) && defined(TFT_MOSI) && defined(TFT_CS)
    // На S3 у ST7735 чаще всего нет MISO
  #if defined(TFT_MISO)
    SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  #else
    SPI.begin(TFT_SCLK, -1,        TFT_MOSI, TFT_CS);
  #endif
#endif
    tft.init();
    Serial.println("TFTSmallDisplay::begin: tft.init()");
    tft.setRotation(1);
    Serial.println("TFTSmallDisplay::begin: tft.setRotation(1)");
    tft.fillScreen(TFT_BLACK);
    Serial.println("TFTSmallDisplay::begin: tft.fillScreen(TFT_BLACK)");
    firstShow = true;
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

        if (predictedPaddle[0]) {
            tft.printf("Left force: %f\n", predictedPaddle[0]->getLoadData().forceL);
            tft.printf("Right force: %f\n", predictedPaddle[0]->getLoadData().forceR);
        }
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
            tft.drawString("F", centerX - 3, workTop + 2);
            tft.drawString("B", centerX - 3, workTop + workHeight - 10);
            tft.drawString("L", centerX + radius + 5, centerY - 3);
            tft.drawString("R", centerX - radius - 12, centerY - 3);
            
        }
        
        // Получаем углы из currentData
        int shaftRotation = 0;
        int shaftTilt = 0;
        int bladeRotation = 0;
        bool isRightBlade = true;

        if (predictedPaddle[0]){
            float tmpShaftRotation = 0;
            float tmpShaftTilt = 0;
            float tmpBladeRotation = 0;

            predictedPaddle[0]->getRelativeAngles(tmpShaftRotation, tmpShaftTilt, tmpBladeRotation, isRightBlade);
            shaftRotation = (int)tmpShaftRotation;
            shaftTilt = (int)tmpShaftTilt;
            bladeRotation = (int)tmpBladeRotation;

            
            

            
            if (shaftRotation != ppaddleData[0].lastShaftRotation) {
                // Стираем старое положение весла
                if (!firstShow) {
                    // Стираем старую линию и точку
                    tft.drawLine(ppaddleData[0].oldLineCoord[0], ppaddleData[0].oldLineCoord[1], ppaddleData[0].oldLineCoord[2], ppaddleData[0].oldLineCoord[3], TFT_BLACK);
    //                tft.fillCircle(oldPaddleX, oldPaddleY, 3, TFT_BLACK);
                }
                
                // Рисуем новое положение весла

                float paddleEndX = centerX + radius * cos((shaftRotation) * M_PI / 180.0f);
                float paddleEndY = centerY - radius * sin((shaftRotation) * M_PI / 180.0f);
                float paddleStartX = centerX - radius * cos((shaftRotation) * M_PI / 180.0f);
                float paddleStartY = centerY + radius * sin((shaftRotation) * M_PI / 180.0f);

                ppaddleData[0].oldLineCoord[0] = paddleStartX;
                ppaddleData[0].oldLineCoord[1] = paddleStartY;
                ppaddleData[0].oldLineCoord[2] = paddleEndX;
                ppaddleData[0].oldLineCoord[3] = paddleEndY;
                
                uint16_t paddleColor = TFT_GREEN;
                
                // Рисуем новую линию весла и точку
                tft.drawLine(paddleStartX, paddleStartY, paddleEndX, paddleEndY, paddleColor);
            }
            
            // Обновляем отображение углов только если изменились
            if ((int)bladeRotation != (int)ppaddleData[0].lastBladeRotation || 
                (int)shaftTilt != (int)ppaddleData[0].lastShaftTilt) {
                
                // Стираем старые значения
                tft.fillRect(0, 106, 160, 10, TFT_BLACK);
                
                // Рисуем новые значения
                tft.setTextFont(1);
                tft.setTextColor(TFT_WHITE);
                tft.drawString("Rot: " + String((int)bladeRotation) + "", 5, 106);
                tft.drawString("Tilt: " + String((int)shaftTilt) + "", 85, 106);
            }
            

            if ((ppaddleData[0].lastIsRightBlade != isRightBlade)) {
                tft.fillRect(centerX - 30, workTop + workHeight - 20, 70, 12, TFT_BLACK);
                String direction = (isRightBlade) ? "RIGHT" : "LEFT";
                tft.setTextColor(TFT_YELLOW);
                tft.drawString(direction, centerX - 20, workTop + workHeight - 15);
            }


            tft.setTextFont(1);
            tft.setTextColor(TFT_CYAN);
            
                    // Отображение значений loadcell

            // Левый loadcell
            if (ppaddleData[0].lastLeftTare != predictedPaddle[0]->getLeftTare()) {
                tft.fillRect(0, 80, 50, 10, TFT_BLACK);  
                tft.drawString(String((int)predictedPaddle[0]->getLeftTare()), 5, 80);
            }
            if (ppaddleData[0].lastLeftForce != predictedPaddle[0]->getLeftForce()) {
                tft.fillRect(0, 90, 50, 10, TFT_BLACK);  
                tft.drawString(String((int)predictedPaddle[0]->getLeftForce()), 5, 90);
            }
            if (ppaddleData[0].lastRightTare != predictedPaddle[0]->getRightTare()) {
                tft.fillRect(110, 80, 50, 10, TFT_BLACK);  
                tft.drawString(String((int)predictedPaddle[0]->getRightTare()), 115, 80);
            }
            if (ppaddleData[0].lastRightForce != predictedPaddle[0]->getRightForce()) {
                tft.fillRect(110, 90, 50, 10, TFT_BLACK);  
                tft.drawString(String((int)predictedPaddle[0]->getRightForce()), 115, 90);
            }

            ppaddleData[0].lastShaftRotation = shaftRotation;
            ppaddleData[0].lastShaftTilt = shaftTilt;
            ppaddleData[0].lastBladeRotation = bladeRotation;
            ppaddleData[0].lastIsRightBlade = isRightBlade;
            ppaddleData[0].lastLeftTare = predictedPaddle[0]->getLeftTare();
            ppaddleData[0].lastRightTare = predictedPaddle[0]->getRightTare();
            ppaddleData[0].lastLeftForce = predictedPaddle[0]->getLeftForce();
            ppaddleData[0].lastRightForce = predictedPaddle[0]->getRightForce();
        }
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
    firstShow = false;
}


void TFTSmallDisplay::showStatusLines() {
    if (xSemaphoreTake(tftMutex, portMAX_DELAY) == pdTRUE) {
        tft.setTextColor(TFT_WHITE);
        tft.fillRect(0, 0, 160, 10, TFT_BLACK);
        tft.setCursor(0, 0);
        tft.printf("Motor: %9d  %8d\n", motorDriver?motorDriver->getForce():0, millis());
        if (firstShow) {

            tft.setCursor(0, 10);
            tft.printf("Paddle: %s\n", ppaddleData[0].lastStatus==PADDLE_STATUS_CONNECTED?"Connected":ppaddleData[0].lastStatus==PADDLE_STATUS_PAIRING?"Pairing":"Disconnected");
        }
        if (predictedPaddle[0]){
            if (ppaddleData[0].lastStatus != predictedPaddle[0]->status()) {
                ppaddleData[0].lastStatus = predictedPaddle[0]->status();
                tft.fillRect(0, 10, 160, 10, TFT_BLACK);
                tft.setCursor(0, 10);
                tft.printf("Paddle: %s\n", predictedPaddle[0]->status()==PADDLE_STATUS_CONNECTED?"Connected":predictedPaddle[0]->status()==PADDLE_STATUS_PAIRING?"Pairing":"Disconnected");
            }
        }
        xSemaphoreGive(tftMutex);
    }
}

void TFTSmallDisplay::showModeLines() {
    if (xSemaphoreTake(tftMutex, portMAX_DELAY) == pdTRUE) {
//        tft.fillRect(0, 118, 160, 128, TFT_BLACK);
        // Predictor mode line
        if (kayak) {
            if (kayak->getPredictorMode()==1) {
                tft.fillRect(70, 116, 20, 20, TFT_ORANGE);
                if (kayak->isUsingPredictedForce()) {
                    tft.fillRect(80, 116, 10, 20, TFT_RED);
                }
            } else {
                tft.fillRect(70, 116, 20, 20, TFT_BLACK);
            }
        }        

        tft.setCursor(0, 118);
        tft.setTextColor(TFT_WHITE);
        tft.printf("Mode: ");
        switch (motorSwitch?motorSwitch->getMode():MOTOR_OFF) {
            case MOTOR_OFF:
                tft.fillRect(30, 116, 40, 20, TFT_BLACK);
                tft.setTextColor(TFT_WHITE);
                tft.printf("OFF\n");
                break;
            case MOTOR_LOW_POWER:
                tft.fillRect(30, 116, 40, 20, TFT_GREEN);
                tft.setTextColor(TFT_BLACK);
                tft.printf("LOW\n");
                break;
            case MOTOR_MEDIUM_POWER:
                tft.fillRect(30, 116, 40, 20, TFT_YELLOW);
                tft.setTextColor(TFT_BLACK);
                tft.printf("MED\n");
                break;
            case MOTOR_HIGH_POWER:
                tft.fillRect(30, 116, 40, 20, TFT_RED);
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
    xTaskCreate(updateTFTTask, "TFT Task", 4096, this, 5, &updateTFTTaskHandle);
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
