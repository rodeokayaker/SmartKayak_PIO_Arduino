#include "LoadCellSetADS1220.h"
#include <Preferences.h>

LoadCellSetADS1220::LoadCellSetADS1220(const char* prefsName, PaddleType paddleType, BladeSideType bladeSide)
    : ILoadCellSet(paddleType, bladeSide) {
    this->prefs_Name = prefsName;
    // Initialize calibration data
    calibrationData.scale[0] = 1.0f; // right blade
    calibrationData.scale[1] = 1.0f; // left blade
    calibrationData.offset[0] = 0.0f;
    calibrationData.offset[1] = 0.0f;
    
    rightUpdated = false;
    leftUpdated = false;
    firstRight = true; // by default start with right channel
    firstChanelSelected = true;
    
    // Load saved calibration data
    readCalibrationData();
}

LoadCellSetADS1220 *instance = nullptr;
Protocentral_ADS1220 LoadCellSetADS1220::pc_ads1220;
TaskHandle_t LoadCellSetADS1220::_taskHandle = nullptr;

bool LoadCellSetADS1220::begin(uint8_t sclk_pin, uint8_t miso_pin, uint8_t mosi_pin, uint8_t cs_pin, uint8_t drdy_pin) {
    
    this->sclk_pin = sclk_pin;
    this->miso_pin = miso_pin;
    this->mosi_pin = mosi_pin;
    this->cs_pin = cs_pin;
    this->drdy_pin = drdy_pin;
    SPI.begin(sclk_pin, miso_pin, mosi_pin, cs_pin);

    pc_ads1220.begin(cs_pin,drdy_pin);
    pc_ads1220.set_data_rate(getFreqID());
    pc_ads1220.set_pga_gain(PGA_GAIN_32);
    pc_ads1220.set_FIR_Filter(FIR_OFF); 
    pc_ads1220.select_mux_channels(MUX_AIN0_AIN1);  //Configure for differential measurement between AIN0 and AIN1
    pc_ads1220.set_conv_mode_continuous();          //Set continuous conversion mode
//    pc_ads1220.Start_Conv();  //Start continuous conversion mode
    return true;
}

int LoadCellSetADS1220::getFreqID() {
    uint16_t fr = requestedFreq();
    if (fr >1000) return 0xE0;
    if (fr >600) return DR_1000SPS;
    if (fr >330) return DR_600SPS;
    if (fr >175) return DR_330SPS;
    if (fr >90) return DR_175SPS;
    if (fr >45) return DR_90SPS;
    if (fr >20) return DR_45SPS;
    return DR_20SPS;
}

void LoadCellSetADS1220::setFrequency(uint16_t frequency) {
    ILoadCellSet::setFrequency(frequency);
    pc_ads1220.set_data_rate(getFreqID());
}

bool LoadCellSetADS1220::getData(loadData& data) {
    memcpy(&data, &currentData, sizeof(loadData));
    bool updated = rightUpdated || leftUpdated;
    rightUpdated = false;
    leftUpdated = false;
    return updated;
}

bool LoadCellSetADS1220::isDataReady(BladeSideType bladeSide) {
    if (bladeSide == ALL_BLADES) {
        return rightUpdated || leftUpdated;
    } else if (bladeSide == RIGHT_BLADE) {
        return rightUpdated;
    } else if (bladeSide == LEFT_BLADE) {
        return leftUpdated;
    }
    return false;
}

void LoadCellSetADS1220::startServices() {
    if (bladeSide == BladeSideType::ALL_BLADES||((bladeSide == BladeSideType::LEFT_BLADE) ^ firstRight)) {
        firstChanelSelected = true;
        pc_ads1220.select_mux_channels(MUX_AIN0_AIN1);
    } else {
        firstChanelSelected = false;
        pc_ads1220.select_mux_channels(MUX_AIN2_AIN3);
    }
    firstChanelSelected = true;
    pc_ads1220.select_mux_channels(MUX_AIN0_AIN1);
    if (LoadCellSetADS1220::_taskHandle == nullptr) {
        xTaskCreatePinnedToCore(LoadCellSetADS1220::taskEntry, "loadcell_ads1220_task", 4096, this, 5, &LoadCellSetADS1220::_taskHandle, tskNO_AFFINITY);
    }
    if (drdy_pin >= 0) {
        attachInterrupt(digitalPinToInterrupt(drdy_pin), LoadCellSetADS1220::intISR, FALLING);
    }
    pc_ads1220.Start_Conv();  //Start continuous conversion mode

}

void LoadCellSetADS1220::stopServices() {
    if (drdy_pin >= 0) {
        detachInterrupt(digitalPinToInterrupt(drdy_pin));
    }
    if (LoadCellSetADS1220::_taskHandle) vTaskDelete(LoadCellSetADS1220::_taskHandle);
    LoadCellSetADS1220::_taskHandle = nullptr;
}

void LoadCellSetADS1220::taskEntry(void* arg) {
    LoadCellSetADS1220* loadcell = (LoadCellSetADS1220*)arg;

    int32_t data;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));

        data = loadcell->pc_ads1220.Read_Data_Samples();
        bool isLeft = loadcell->firstChanelSelected ^ loadcell->firstRight;

        if (isLeft) {
            loadcell->currentData.forceL = data*loadcell->calibrationData.scale[1]-loadcell->calibrationData.offset[1];
            loadcell->currentData.forceL_raw = data;
            loadcell->leftUpdated = true;
        } else {
            loadcell->currentData.forceR = data*loadcell->calibrationData.scale[0]-loadcell->calibrationData.offset[0];
            loadcell->currentData.forceR_raw = data;
            loadcell->rightUpdated = true;
        }

        if (loadcell->bladeSide == BladeSideType::ALL_BLADES) {
            if (loadcell->firstChanelSelected) {
                loadcell->pc_ads1220.select_mux_channels(MUX_AIN2_AIN3);
                loadcell->firstChanelSelected = false;
            } else {
                loadcell->pc_ads1220.select_mux_channels(MUX_AIN0_AIN1);
                loadcell->firstChanelSelected = true;
            }
        }

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

void IRAM_ATTR LoadCellSetADS1220::intISR() {
    BaseType_t hpw = pdFALSE;
    if (LoadCellSetADS1220::_taskHandle) vTaskNotifyGiveFromISR(LoadCellSetADS1220::_taskHandle, &hpw);
    if (hpw) portYIELD_FROM_ISR();
}


void LoadCellSetADS1220::calibrateBlade(BladeSideType bladeSide, float weight) {
    if (logStream) {
        logStream->printf("Calibrating load cells for %s\n", 
                         bladeSide == RIGHT_BLADE ? "right blade" : 
                         bladeSide == LEFT_BLADE ? "left blade" : "Error");
        logStream->println("Remove all loads");
    }
    if (bladeSide != RIGHT_BLADE && bladeSide != LEFT_BLADE) {
        return;
    }
    
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    float sum = 0;
    for (int i = 0; i < 200; i++) {

            loadData data;
            getData(data);
            sum += bladeSide == RIGHT_BLADE ? data.forceR_raw : data.forceL_raw;
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
    
    sum = 0;
    for (int i = 0; i < 200; i++) {
            loadData data;
            getData(data);
            sum += bladeSide == RIGHT_BLADE ? data.forceR_raw : data.forceL_raw - tare_raw;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    float scale = weight / ((float)sum / 200);
    calibrationData.offset[bladeSide == RIGHT_BLADE ? 0 : 1] = tare_raw*scale;
    calibrationData.scale[bladeSide == RIGHT_BLADE ? 0 : 1] = scale;
    
    saveCalibrationData();
    
    if (logStream) {
        logStream->println("Calibration completed");
    }
}


void LoadCellSetADS1220::tareBlade(BladeSideType bladeSide) {
    if (logStream) {
        logStream->printf("Taring load cells for %s\n", 
                         bladeSide == RIGHT_BLADE ? "right blade" : 
                         bladeSide == LEFT_BLADE ? "left blade" : "Error");
        logStream->println("Remove all loads");
    }
    if (bladeSide != RIGHT_BLADE && bladeSide != LEFT_BLADE) {
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    float sum =0;
    int count = 0;
    
    for (int j = 0; j < 4; j++) {
    // Collect data for averaging
        for(int i = 0; i < 200; i++) {
                loadData data;
                getData(data);
                sum += bladeSide == RIGHT_BLADE ? data.forceR : data.forceL;
                count++;
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (j<3) logStream->printf("Turn 90 degrees\n");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
    
    
    if (count > 0) {
        if (bladeSide == RIGHT_BLADE) {
            calibrationData.offset[0] += (float)sum / count;
        }
        if (bladeSide == LEFT_BLADE) {
            calibrationData.offset[1] += (float)sum / count;
        }
    }
    
    if (logStream) {
        logStream->println("Tare completed");
    }
}

void LoadCellSetADS1220::tare(BladeSideType bladeSide) {
    if (bladeSide==ALL_BLADES) {
        tareBlade(RIGHT_BLADE);
        tareBlade(LEFT_BLADE);
    } else {
        tareBlade(bladeSide);
    }
}

void LoadCellSetADS1220::calibrate(BladeSideType bladeSide, float weight) {
    if (bladeSide==ALL_BLADES) {
        calibrateBlade(RIGHT_BLADE, weight);
        calibrateBlade(LEFT_BLADE, weight);
    } else {
        calibrateBlade(bladeSide, weight);
    }
}

void LoadCellSetADS1220::updateTare(BladeSideType bladeSide, float weight) {
    if (bladeSide==RIGHT_BLADE) {
        calibrationData.offset[0] += weight;
    } else if (bladeSide==LEFT_BLADE) {
        calibrationData.offset[1] += weight;
    }
}

bool LoadCellSetADS1220::isCalibrationValid(BladeSideType bladeSide) {
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

void LoadCellSetADS1220::reset() {
    stopServices();
    delay(100);
    startServices();
}

void LoadCellSetADS1220::saveCalibrationData() {
    Preferences prefs;
    prefs.begin(prefs_Name, false);
    prefs.putBytes("calibData", &calibrationData, sizeof(loadCellSetCalibrationData));
    prefs.end();
}

bool LoadCellSetADS1220::readCalibrationData() {
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

void LoadCellSetADS1220::resetCalibration() {
    calibrationData.scale[0] = 1.0f;
    calibrationData.scale[1] = 1.0f;
    calibrationData.offset[0] = 0.0f;
    calibrationData.offset[1] = 0.0f;
}

