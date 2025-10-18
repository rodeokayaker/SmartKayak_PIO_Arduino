#include "LoadCellSetHX711.h"
#include <Preferences.h>

// Static task handle
TaskHandle_t LoadCellSetHX711::_taskHandle = nullptr;

LoadCellSetHX711::LoadCellSetHX711(const char* prefsName, PaddleType paddleType, BladeSideType bladeSide) 
    : ILoadCellSet(paddleType, bladeSide), scaleR(), scaleL() {
    this->prefs_Name = prefsName;
    
    // Initialize calibration data
    calibrationData.scale[0] = 1.0f; // right blade
    calibrationData.scale[1] = 1.0f; // left blade
    calibrationData.offset[0] = 0.0f;
    calibrationData.offset[1] = 0.0f;
    
    rightUpdated = false;
    leftUpdated = false;
    readyToRead = false;
    

}

bool LoadCellSetHX711::begin(uint8_t doutR_pin, uint8_t sclkR_pin, uint8_t doutL_pin, uint8_t sclkL_pin) {
    this->doutR_pin = doutR_pin;
    this->sclkR_pin = sclkR_pin;
    this->doutL_pin = doutL_pin;
    this->sclkL_pin = sclkL_pin;
        // Load saved calibration data
        readCalibrationData();
    
    // Initialize right scale
    scaleR.begin(doutR_pin, sclkR_pin);
    if(!scaleR.wait_ready_timeout(1000)) {
        if (logStream) {
            logStream->println("Right HX711 not found!");
        }
        return false;
    }
    
    // Initialize left scale
    scaleL.begin(doutL_pin, sclkL_pin);
    if(!scaleL.wait_ready_timeout(1000)) {
        if (logStream) {
            logStream->println("Left HX711 not found!");
        }
        return false;
    }
    
    readyToRead = true;
    return true;
}

void LoadCellSetHX711::startServices() {
    readyToRead = true;
    
    // Create FreeRTOS task if not already created
    if (LoadCellSetHX711::_taskHandle == nullptr) {
        xTaskCreatePinnedToCore(LoadCellSetHX711::taskEntry, "loadcell_hx711_task", 4096, this, 5, &LoadCellSetHX711::_taskHandle, tskNO_AFFINITY);
    }
    
    // Attach interrupts to DOUT pins
    if (doutR_pin >= 0) {
        attachInterrupt(digitalPinToInterrupt(doutR_pin), LoadCellSetHX711::intISR_R, FALLING);
    }
    if (doutL_pin >= 0) {
        attachInterrupt(digitalPinToInterrupt(doutL_pin), LoadCellSetHX711::intISR_L, FALLING);
    }
}

void LoadCellSetHX711::stopServices() {
    // Detach interrupts
    if (doutR_pin >= 0) {
        detachInterrupt(digitalPinToInterrupt(doutR_pin));
    }
    if (doutL_pin >= 0) {
        detachInterrupt(digitalPinToInterrupt(doutL_pin));
    }
    
    // Delete FreeRTOS task
    if (LoadCellSetHX711::_taskHandle) {
        vTaskDelete(LoadCellSetHX711::_taskHandle);
        LoadCellSetHX711::_taskHandle = nullptr;
    }
    
    // Power down both scales
    scaleR.power_down();
    scaleL.power_down();
    readyToRead = false;
}

bool LoadCellSetHX711::getData(loadData& data) {
    memcpy(&data, &currentData, sizeof(loadData));
    bool updated = rightUpdated || leftUpdated;
    rightUpdated = false;
    leftUpdated = false;
    return updated;
}

bool LoadCellSetHX711::isDataReady(BladeSideType bladeSide) {
    if (!readyToRead) {
        return false;
    }
    
    if (bladeSide == ALL_BLADES) {
        return scaleR.is_ready() || scaleL.is_ready();
    } else if (bladeSide == RIGHT_BLADE) {
        return scaleR.is_ready();
    } else if (bladeSide == LEFT_BLADE) {
        return scaleL.is_ready();
    }
    return false;
}

void LoadCellSetHX711::setFrequency(uint16_t freq) {
    ILoadCellSet::setFrequency(freq);
    // HX711 frequency is fixed, but we can adjust reading rate
}

void LoadCellSetHX711::calibrateBlade(BladeSideType bladeSide, float weight) {
    if (logStream) {
        logStream->printf("Calibrating load cells for %s\n", 
                         bladeSide == RIGHT_BLADE ? "right blade" : 
                         bladeSide == LEFT_BLADE ? "left blade" : "Error");
        logStream->println("Remove all loads");
    }
    
    if (bladeSide != RIGHT_BLADE && bladeSide != LEFT_BLADE) {
        return;
    }
    
    HX711* scale = (bladeSide == RIGHT_BLADE) ? &scaleR : &scaleL;
    int scaleIndex = (bladeSide == RIGHT_BLADE) ? 0 : 1;
    
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Collect tare data
    float sum = 0;
    for (int i = 0; i < 200; i++) {
//        if (scale->is_ready()) {
//            sum += scale->read();
//        }
        Serial.printf("Force %s: %d\n", bladeSide == RIGHT_BLADE ? "right blade" : "left blade", bladeSide == RIGHT_BLADE ? currentData.forceR_raw : currentData.forceL_raw);
        sum += (bladeSide == RIGHT_BLADE) ? currentData.forceR_raw : currentData.forceL_raw;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    float tare_raw = sum / 200;
    
    if (logStream) {
        logStream->printf("Zero offset for %s: %.2f\n", 
                         bladeSide == RIGHT_BLADE ? "right blade" : "left blade", 
                         tare_raw);
    }
    
    if (logStream) {
        logStream->printf("Place %1.1fg weight and wait...", weight);
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Collect weight data
    sum = 0;
    for (int i = 0; i < 200; i++) {
//        if (scale->is_ready()) {
//            sum += scale->read() - tare_raw;
//        }
        sum += (bladeSide == RIGHT_BLADE) ? currentData.forceR_raw - tare_raw : currentData.forceL_raw - tare_raw;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    float scale_factor = weight / ((float)sum / 200);
    calibrationData.offset[scaleIndex] = tare_raw * scale_factor;
    calibrationData.scale[scaleIndex] = scale_factor;
    
    saveCalibrationData();
    
    if (logStream) {
        logStream->println("Calibration completed");
    }
}

void LoadCellSetHX711::tareBlade(BladeSideType bladeSide) {
    if (logStream) {
        logStream->printf("Taring load cells for %s\n", 
                         bladeSide == RIGHT_BLADE ? "right blade" : 
                         bladeSide == LEFT_BLADE ? "left blade" : "Error");
        logStream->println("Remove all loads");
    }
    
    if (bladeSide != RIGHT_BLADE && bladeSide != LEFT_BLADE) {
        return;
    }
    
    HX711* scale = (bladeSide == RIGHT_BLADE) ? &scaleR : &scaleL;
    int scaleIndex = (bladeSide == RIGHT_BLADE) ? 0 : 1;
    
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    float sum = 0;
    int count = 0;
    
    // Collect data from multiple orientations
    for (int j = 0; j < 4; j++) {
        for(int i = 0; i < 200; i++) {
            if (scale->is_ready()) {
                int32_t raw = scale->read();
                float force = raw * calibrationData.scale[scaleIndex] - calibrationData.offset[scaleIndex];
                sum += force;
                count++;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        if (j < 3) {
            if (logStream) {
                logStream->printf("Turn 90 degrees\n");
            }
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
    
    if (count > 0) {
        float average_force = sum / count;
        calibrationData.offset[scaleIndex] += average_force;
    }
    
    saveCalibrationData();
    
    if (logStream) {
        logStream->println("Tare completed");
    }
}

void LoadCellSetHX711::tare(BladeSideType bladeSide) {
    if (bladeSide == ALL_BLADES) {
        tareBlade(RIGHT_BLADE);
        tareBlade(LEFT_BLADE);
    } else {
        tareBlade(bladeSide);
    }
}

void LoadCellSetHX711::calibrate(BladeSideType bladeSide, float weight) {
    if (bladeSide == ALL_BLADES) {
        calibrateBlade(RIGHT_BLADE, weight);
        calibrateBlade(LEFT_BLADE, weight);
    } else {
        calibrateBlade(bladeSide, weight);
    }
}

void LoadCellSetHX711::updateTare(BladeSideType bladeSide, float weight) {
    if (bladeSide == RIGHT_BLADE) {
        calibrationData.offset[0] += weight;
    } else if (bladeSide == LEFT_BLADE) {
        calibrationData.offset[1] += weight;
    }
    saveCalibrationData();
}

bool LoadCellSetHX711::isCalibrationValid(BladeSideType bladeSide) {
    if (bladeSide == RIGHT_BLADE || bladeSide == ALL_BLADES) {
        if (calibrationData.scale[0] == 1.0f && calibrationData.offset[0] == 0.0f) {
            return false;
        }
    }
    if (bladeSide == LEFT_BLADE || bladeSide == ALL_BLADES) {
        if (calibrationData.scale[1] == 1.0f && calibrationData.offset[1] == 0.0f) {
            return false;
        }
    }
    return true;
}

void LoadCellSetHX711::reset() {
    stopServices();
    delay(100);
    startServices();
}

void LoadCellSetHX711::setPins(uint8_t doutR_pin, uint8_t sclkR_pin, uint8_t doutL_pin, uint8_t sclkL_pin) {
    this->doutR_pin = doutR_pin;
    this->sclkR_pin = sclkR_pin;
    this->doutL_pin = doutL_pin;
    this->sclkL_pin = sclkL_pin;
}

void LoadCellSetHX711::saveCalibrationData() {
    Preferences prefs;
    prefs.begin(prefs_Name, false);
    prefs.putBytes("calibData", &calibrationData, sizeof(loadCellSetCalibrationData));
    prefs.end();
}

bool LoadCellSetHX711::readCalibrationData() {
    Preferences prefs;
    prefs.begin(prefs_Name, true);
    if(prefs.isKey("calibData")) {
        prefs.getBytes("calibData", &calibrationData, sizeof(loadCellSetCalibrationData));
        prefs.end();
        return true;
    }
    prefs.end();
    return false;
}

void LoadCellSetHX711::resetCalibration() {
    calibrationData.scale[0] = 1.0f;
    calibrationData.scale[1] = 1.0f;
    calibrationData.offset[0] = 0.0f;
    calibrationData.offset[1] = 0.0f;
}

// FreeRTOS task entry point
void LoadCellSetHX711::taskEntry(void* arg) {
    LoadCellSetHX711* loadcell = (LoadCellSetHX711*)arg;
    
    for (;;) {
        // Wait for notification from interrupt
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
        
        // Read right scale if ready
        if (loadcell->scaleR.is_ready()) {
            int32_t rawR = loadcell->scaleR.read();
            loadcell->currentData.forceR_raw = rawR;
            loadcell->currentData.forceR = rawR * loadcell->calibrationData.scale[0] - loadcell->calibrationData.offset[0];
            loadcell->rightUpdated = true;
        }
        
        // Read left scale if ready
        if (loadcell->scaleL.is_ready()) {
            int32_t rawL = loadcell->scaleL.read();
            loadcell->currentData.forceL_raw = rawL;
            loadcell->currentData.forceL = rawL * loadcell->calibrationData.scale[1] - loadcell->calibrationData.offset[1];
            loadcell->leftUpdated = true;
        }
        
        // Update timestamp
        if (loadcell->rightUpdated || loadcell->leftUpdated) {
            loadcell->currentData.timestamp = millis();
        }
        
        // Handle callbacks
        if (loadcell->loadDataCb) {
            if (loadcell->singleNotify) {
                if (loadcell->leftUpdated) {
                    loadcell->loadDataCb(loadcell->currentData, BladeSideType::LEFT_BLADE);
                    loadcell->leftUpdated = false;
                }
                if (loadcell->rightUpdated) {
                    loadcell->loadDataCb(loadcell->currentData, BladeSideType::RIGHT_BLADE);
                    loadcell->rightUpdated = false;
                }
            } else {
                if (loadcell->leftUpdated && loadcell->rightUpdated) {
                    loadcell->loadDataCb(loadcell->currentData, BladeSideType::ALL_BLADES);
                    loadcell->leftUpdated = false;
                    loadcell->rightUpdated = false;
                }
            }
        }
    }
}

// Interrupt service routines
void IRAM_ATTR LoadCellSetHX711::intISR_R() {
    BaseType_t hpw = pdFALSE;
    if (LoadCellSetHX711::_taskHandle) {
        vTaskNotifyGiveFromISR(LoadCellSetHX711::_taskHandle, &hpw);
    }
    if (hpw) portYIELD_FROM_ISR();
}

void IRAM_ATTR LoadCellSetHX711::intISR_L() {
    BaseType_t hpw = pdFALSE;
    if (LoadCellSetHX711::_taskHandle) {
        vTaskNotifyGiveFromISR(LoadCellSetHX711::_taskHandle, &hpw);
    }
    if (hpw) portYIELD_FROM_ISR();
}
