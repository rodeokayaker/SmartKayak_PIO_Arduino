#include "GFXKayakDisplay.h"
#include "SmartKayak.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "../Core/Interfaces/ILogger.h"
#include <Arduino_GFX_Library.h>

GFXKayakDisplay::GFXKayakDisplay(Arduino_GFX* _gfx, int _ledPin, int frequency) : 
    KayakDisplay(1000/frequency), 
    gfx(_gfx), 
    mode(0), 
    updateGFXTaskHandle(nullptr), 
    firstShow(true),
    debugScreen(false),
    debugData{0, 0, true, false},
    motorData{0, 0.0f},
    kayak(nullptr),
    initialized(false),
    ledPin(_ledPin)
{
    gfxMutex = xSemaphoreCreateMutex();
    if (gfxMutex == NULL) {
        Serial.println("Failed to create GFX mutex");
    }
}

void GFXKayakDisplay::begin() {
    Serial.println("GFXKayakDisplay::begin");

    if (!gfx->begin()) {
        Serial.println("Failed to initialize GFX");
        return;
    }
    if (ledPin != -1) {
        pinMode(ledPin, OUTPUT);
        digitalWrite(ledPin, HIGH);
    }
    initialized = true;
    width = gfx->width();
    height = gfx->height();
    landscape = width > height;
    firstShow = true;
}

void GFXKayakDisplay::setRotation(int rotation) {
    if (rotation < 0 || rotation > 3) {
        Serial.println("Invalid rotation");
        return;
    }
    gfx->setRotation(rotation);
    if (rotation %2)landscape = !landscape;
}

void GFXKayakDisplay::clear() {
    if (xSemaphoreTake(gfxMutex, portMAX_DELAY) == pdTRUE) {
        gfx->fillScreen(BLACK);
        xSemaphoreGive(gfxMutex);
    }
}

void GFXKayakDisplay::end() {
    if (!initialized) return;
    if (updateGFXTaskHandle != nullptr) {
        vTaskDelete(updateGFXTaskHandle);
        updateGFXTaskHandle = nullptr;
    }
    if (gfxMutex != nullptr) {
        vSemaphoreDelete(gfxMutex);
        gfxMutex = nullptr;
    }
}

GFXKayakDisplay::~GFXKayakDisplay() {
    end();
}

void GFXKayakDisplay::updateGFXTask(void* pvParameters) {
    GFXKayakDisplay* display = (GFXKayakDisplay*)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(display->updateInterval);
    
    while (true) {
        if (display->initialized) {
            display->updateDisplay();
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void GFXKayakDisplay::showMotorForceScreen() {
    if (xSemaphoreTake(gfxMutex, portMAX_DELAY) == pdTRUE) {
        gfx->setCursor(0, 50);
        gfx->setTextColor(WHITE);
        gfx->setTextSize(2);
        gfx->fillRect(0, 50, 160, 50, BLACK);

        if (predictedPaddle[0]) {
            gfx->printf("Left force: %f\n", predictedPaddle[0]->getLoadData().forceL);
            gfx->printf("Right force: %f\n", predictedPaddle[0]->getLoadData().forceR);
        }
        xSemaphoreGive(gfxMutex);
    }
}

void GFXKayakDisplay::showOrientationScreen() { 
    if (xSemaphoreTake(gfxMutex, portMAX_DELAY) == pdTRUE) {
        int fontsSize = 1;
        if (width > 200) {
            fontsSize = 2;
        }
        // Рабочая область экрана (с учетом статусных строк)
        int centerX = width/2;  // 160/2
        int centerY = height/2;  // 128/2
        int workHeight = height - fontsSize*10*3; // 128 - 20 (верх) - 20 (низ)
        int workTop = fontsSize*20;
        int radius = 25*fontsSize;
        

        if (firstShow) {
            gfx->fillScreen(BLACK);
            gfx->setTextColor(WHITE);


            // Рисуем каяк (статичные элементы)
            gfx->drawLine(centerX, workTop + 10*fontsSize, centerX, workTop + workHeight - 10*fontsSize, BLUE);
            gfx->drawLine(centerX - 5*fontsSize, workTop + 15*fontsSize, centerX, workTop + 10*fontsSize, BLUE);
            gfx->drawLine(centerX + 5*fontsSize, workTop + 15*fontsSize, centerX, workTop + 10*fontsSize, BLUE);
            gfx->drawLine(centerX + 5*fontsSize, workTop + 15*fontsSize, centerX, workTop + 10*fontsSize, BLUE);
            
            // Рисуем окружность для показа ориентации
            gfx->drawCircle(centerX, centerY, radius, DARKGREY);
            gfx->drawCircle(centerX, centerY, radius+fontsSize-1, DARKGREY);
            
            
            // Рисуем компас (статичные элементы)
            gfx->setTextSize(fontsSize);
            gfx->setTextColor(CYAN);
            gfx->setCursor(centerX - 3*fontsSize, workTop + 2*fontsSize);
            gfx->printf("F");
            gfx->setCursor(centerX - 3*fontsSize, workTop + workHeight - 10*fontsSize);
            gfx->printf("B");
            gfx->setCursor(centerX + radius + 5*fontsSize, centerY - 3*fontsSize);
            gfx->printf("R");
            gfx->setCursor(centerX - radius - 12*fontsSize, centerY - 3*fontsSize);
            gfx->printf("L");
            
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
                    gfx->drawLine(ppaddleData[0].oldLineCoord[0], ppaddleData[0].oldLineCoord[1], ppaddleData[0].oldLineCoord[2], ppaddleData[0].oldLineCoord[3], BLACK);
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
                
                uint16_t paddleColor = GREEN;
                
                // Рисуем новую линию весла и точку
                gfx->drawLine(paddleStartX, paddleStartY, paddleEndX, paddleEndY, paddleColor);
            }
            
            // Обновляем отображение углов только если изменились
            if ((int)bladeRotation != (int)ppaddleData[0].lastBladeRotation || 
                (int)shaftTilt != (int)ppaddleData[0].lastShaftTilt) {
                
                // Стираем старые значения
                gfx->fillRect(0, workTop + workHeight - 10*fontsSize, width, fontsSize*10, BLACK);
                
                // Рисуем новые значения
                gfx->setTextSize(fontsSize);
                gfx->setTextColor(WHITE);
                gfx->setCursor(6*fontsSize, workTop + workHeight - 10*fontsSize);
                gfx->printf("Rot: %d\n", (int)bladeRotation);
                gfx->setCursor(width- 10*6*fontsSize, workTop + workHeight - 10*fontsSize);
                gfx->printf("Tilt: %3d\n", (int)shaftTilt);
            }
            

            if ((ppaddleData[0].lastIsRightBlade != isRightBlade)) {
                gfx->fillRect(centerX - 15*fontsSize, workTop + workHeight - 20*fontsSize, 6*5*fontsSize, 10*fontsSize, BLACK);
                String direction = (isRightBlade) ? "RIGHT" : "LEFT";
                gfx->setTextColor(YELLOW);
                gfx->setCursor(centerX - 15*fontsSize, workTop + workHeight - 20*fontsSize);
                gfx->printf("%s\n", direction.c_str());
            }


            gfx->setTextSize(fontsSize);
            gfx->setTextColor(CYAN);
            
                    // Отображение значений loadcell

            // Левый loadcell
            if (ppaddleData[0].lastLeftTare != predictedPaddle[0]->getLeftTare()||ppaddleData[0].lastLeftForce != predictedPaddle[0]->getLeftForce()) {
                gfx->setTextSize(1);
                gfx->fillRect(5, workTop + 10, 60, 20, BLACK);  
                gfx->setCursor(5, workTop + 10);
                gfx->printf("%d\n", (int)predictedPaddle[0]->getLeftTare());
                gfx->setCursor(5, workTop + 20);
                gfx->printf("%d\n", (int)predictedPaddle[0]->getLoadData().forceL);
                gfx->setTextSize(fontsSize);
            }
            if (ppaddleData[0].lastLeftForce != predictedPaddle[0]->getLeftForce()) {
                gfx->fillRect(5*fontsSize, workTop + workHeight - 30*fontsSize, 8*6*fontsSize, 10*fontsSize, BLACK);  
                gfx->setCursor(5*fontsSize, workTop + workHeight - 30*fontsSize);
                gfx->printf("%d\n", (int)predictedPaddle[0]->getLeftForce());
            }
            if (ppaddleData[0].lastRightTare != predictedPaddle[0]->getRightTare()||ppaddleData[0].lastRightForce != predictedPaddle[0]->getRightForce()) {
                gfx->setTextSize(1);
                gfx->fillRect(width-5-6*10, workTop + 10, 60, 20, BLACK);  
                gfx->setCursor(width-5-6*10, workTop + 10);
                gfx->printf("%d\n", (int)predictedPaddle[0]->getRightTare());
                gfx->setCursor(width-5-6*10, workTop + 20);
                gfx->printf("%d\n", (int)predictedPaddle[0]->getLoadData().forceR);
                gfx->setTextSize(fontsSize);
            }
            if (ppaddleData[0].lastRightForce != predictedPaddle[0]->getRightForce()) {
                gfx->fillRect(width-8*fontsSize*6-5*fontsSize, workTop + workHeight - 30*fontsSize, 8*fontsSize*6+5*fontsSize, 10*fontsSize, BLACK);  
                gfx->setCursor(width-8*fontsSize*6-5*fontsSize, workTop + workHeight - 30*fontsSize);
                gfx->printf("%8d\n", (int)predictedPaddle[0]->getRightForce());
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
        xSemaphoreGive(gfxMutex);
    }
}

void GFXKayakDisplay::updateDisplay() {


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


void GFXKayakDisplay::showStatusLines() {
    if (xSemaphoreTake(gfxMutex, portMAX_DELAY) == pdTRUE) {
        int fontsSize = 1;
        if (width > 200) {
            fontsSize = 2;
        }
        if (firstShow){
            gfx->fillRect(0, 0, width, fontsSize*10*2, BLACK);

        }
        gfx->setTextColor(WHITE);
        gfx->setTextSize(fontsSize);
        if (motorDriver->getForce() != motorData.signal) {

            gfx->fillRect(6*7*fontsSize, 0, 6*4*fontsSize, fontsSize*10, BLACK);
            gfx->setCursor(0, 0);
            gfx->printf("Motor: %4d",motorDriver?motorDriver->getForce():0);
            motorData.signal = motorDriver->getForce();
        }
        gfx->fillRect(width - 8*fontsSize*6, 0, 8*fontsSize*6, fontsSize*10, BLACK);
        gfx->setCursor(width - 8*fontsSize*6, 0);
        gfx->setTextColor(GREENYELLOW);
        gfx->printf("%8d", millis());
        gfx->fillRect(width - 8*fontsSize*6, 10*fontsSize, 8*fontsSize*6, fontsSize*10*2, BLACK);
        gfx->setCursor(width - 8*fontsSize*6, 10*fontsSize);
        gfx->setTextColor(WHITE);
        uint32_t freeHeap = ESP.getMinFreeHeap();
        if (freeHeap<30000){
            gfx->setTextColor(ORANGE);
        }
        if (freeHeap<20000){
            gfx->setTextColor(RED);
        }
        gfx->printf("%8d", freeHeap);
        gfx->setTextColor(WHITE);

        if (predictedPaddle[0]){
            if ((ppaddleData[0].lastStatus != predictedPaddle[0]->status()) || firstShow) {
                ppaddleData[0].lastStatus = predictedPaddle[0]->status();
                gfx->fillRect(0, fontsSize*10, width, fontsSize*10, BLACK);
                gfx->setCursor(0, fontsSize*10);
                gfx->printf("Paddle: %s\n", ppaddleData[0].lastStatus==PADDLE_STATUS_CONNECTED?"Connected":ppaddleData[0].lastStatus==PADDLE_STATUS_PAIRING?"Pairing":"NO");
            }
        }
        xSemaphoreGive(gfxMutex);
    }
}

void GFXKayakDisplay::showModeLines() {
    if (xSemaphoreTake(gfxMutex, portMAX_DELAY) == pdTRUE) {
        int fontsSize = 1;
        if (width > 200) {
            fontsSize = 2;
        }
        if (firstShow){
            gfx->fillRect(0, height - fontsSize*10, width, fontsSize*10, BLACK);
        }
        // Predictor mode line
        if (kayak) {
            if (kayak->getPredictorMode()==1) {
                gfx->fillRect(width/2 - 10*fontsSize, height - fontsSize*10, 20*fontsSize, 20*fontsSize, ORANGE);
                if (kayak->isUsingPredictedForce()) {
                    gfx->fillRect(width/2 , height - fontsSize*10, 10*fontsSize, 20*fontsSize, RED);
                }
            } else {
                gfx->fillRect(width/2 - 10*fontsSize, height - fontsSize*10, 20*fontsSize, 20*fontsSize, BLACK);
            }
        }        

        gfx->setCursor(0, height - fontsSize*10);
        gfx->setTextColor(WHITE);
        gfx->printf("Mode: ");
        switch (motorSwitch?motorSwitch->getMode():MOTOR_OFF) {
            case MOTOR_OFF:
                gfx->fillRect(5*6*fontsSize, height - fontsSize*12, 5*6*fontsSize, 12*fontsSize, BLACK);
                gfx->setTextColor(WHITE);
                gfx->printf("OFF\n");
                break;
            case MOTOR_LOW_POWER:
                gfx->fillRect(5*6*fontsSize, height - fontsSize*12, 5*6*fontsSize, 12*fontsSize, GREEN);
                gfx->setTextColor(BLACK);
                gfx->printf("LOW\n");
                break;
            case MOTOR_MEDIUM_POWER:
                gfx->fillRect(5*6*fontsSize, height - fontsSize*12, 5*6*fontsSize, 12*fontsSize, YELLOW);
                gfx->setTextColor(BLACK);
                gfx->printf("MED\n");
                break;
            case MOTOR_HIGH_POWER:
                gfx->fillRect(5*6*fontsSize, height - fontsSize*12, 5*6*fontsSize, 12*fontsSize, RED);
                gfx->setTextColor(WHITE);
                gfx->printf("HIGH\n");
                break;
            case MOTOR_DEBUG:
                gfx->fillRect(5*6*fontsSize, height - fontsSize*12, 6*6*fontsSize, 12*fontsSize, WHITE);
                gfx->setTextColor(BLACK);
                gfx->printf("DEBUG\n");
                break;
        }
        gfx->setCursor(width - 8*fontsSize*6, height - fontsSize*10);
        if (logSwitch&&logSwitch->getLogStarted()) {
            gfx->fillRect(width - 4*fontsSize*6, height - fontsSize*12, 8*fontsSize*6, 12*fontsSize, BLUE);
        } else {
            gfx->fillRect(width - 4*fontsSize*6, height - fontsSize*12, 8*fontsSize*6, 12*fontsSize, BLACK);
        }
        gfx->setTextColor(WHITE);
        gfx->printf("Log: %s\n", logModeNames[logSwitch?logSwitch->getLogMode():LOG_MODE_OFF]);

        xSemaphoreGive(gfxMutex);
    }
}

void GFXKayakDisplay::startTasks() {
    xTaskCreate(updateGFXTask, "GFX Task", 4096, this, 5, &updateGFXTaskHandle);
}

void GFXKayakDisplay::setDebugData(int force, int load, bool scn) {
    debugData.force = force;
    debugData.load = load;
    debugData.scn = scn;
}

void GFXKayakDisplay::switchDebugScreen(bool on) {
    debugData.firstShow = true;
    debugScreen = on;
    firstShow = true;
}

void GFXKayakDisplay::showDebugScreen() {
    if (xSemaphoreTake(gfxMutex, portMAX_DELAY) == pdTRUE) {
        if (debugData.firstShow) {
            gfx->fillScreen(BLACK);
            debugData.firstShow = false;
        }
        gfx->fillRect(45, 56, 160, 32, BLACK);
        gfx->setCursor(0, 40);
        if (debugData.scn) {
            gfx->setTextColor(YELLOW);
        } else {
            gfx->setTextColor(WHITE);
        }
        gfx->setTextSize(2);
        gfx->printf("DEBUG\nFORCE: %d\nLOAD:  %d\n", debugData.force, debugData.load);

        gfx->setTextSize(1);
        gfx->setTextColor(WHITE);
        xSemaphoreGive(gfxMutex);
    }
}
