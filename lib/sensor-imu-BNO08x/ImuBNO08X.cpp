/**
 * @file ImuBNO08X.cpp
 * @brief Implementation of BNO08X IMU sensor using SH2 protocol
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#include "ImuBNO08X.h"
#include <Arduino.h>
#include <Wire.h>

// Cross-platform sleep implementation
#if defined(ARDUINO_ARCH_ESP32)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
static inline void sleepMs(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }
#else
static inline void sleepMs(uint32_t ms) { delay(ms); }
#endif

// Static member variables initialization
int8_t ImuBNO08X::_interruptPin = -1;
uint8_t ImuBNO08X::_i2cAddress = 0x4B;
int8_t ImuBNO08X::_rstPin = -1;

IMUData ImuBNO08X::currentData = {};
OrientationData ImuBNO08X::currentOrientation = {};
int ImuBNO08X::accuracy[6] = {0, 0, 0, 0, 0, 0};

TwoWire* ImuBNO08X::_i2c;
sh2_Hal_t ImuBNO08X::_hal;
volatile bool ImuBNO08X::_haveEvent;
volatile bool ImuBNO08X::_haveOrientation;
volatile bool ImuBNO08X::_haveIMU;
sh2_SensorValue_t ImuBNO08X::_value;

#if defined(ARDUINO_ARCH_ESP32)
static TaskHandle_t _taskHandle;
static void taskEntry(void* arg);
static void IRAM_ATTR intISR();
#endif

// SH2 HAL methods

bool ImuBNO08X::waitForIntIfNeeded() {
    if (_interruptPin < 0) return true;
    
    for (int i = 0; i < 500; ++i) {
        if (!digitalRead(_interruptPin)) return true;
        sleepMs(1);
    }
    return false;
}

void ImuBNO08X::hwReset() {
    if (_rstPin < 0) return;
    
    pinMode(_rstPin, OUTPUT);
    digitalWrite(_rstPin, HIGH);
    sleepMs(10);
    digitalWrite(_rstPin, LOW);
    sleepMs(10);
    digitalWrite(_rstPin, HIGH);
    sleepMs(10);
}

uint32_t ImuBNO08X::hal_getTimeUs(sh2_Hal_t*) {
    return millis() * 1000UL;
}

int ImuBNO08X::i2c_open(sh2_Hal_t*) {
    if (_interruptPin >= 0) {
        if (!waitForIntIfNeeded()) return -1;
    }
    
    uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
    bool success = false;
    for (uint8_t attempts = 0; attempts < 5; attempts++) {
        _i2c->beginTransmission(_i2cAddress);
        if (_i2c->write(softreset_pkt, 5) == 5 && _i2c->endTransmission(true) == 0) {
            success = true;
            break;
        }
        sleepMs(30);
    }
    if (!success) return -1;
    sleepMs(300);
    return 0;
}

void ImuBNO08X::i2c_close(sh2_Hal_t*) {
}

bool ImuBNO08X::i2c_write_bytes(const uint8_t* buf, size_t len, bool stop) {
    if (len > kI2CMax) return false;
    ImuBNO08X::_i2c->beginTransmission(ImuBNO08X::_i2cAddress);
    if (ImuBNO08X::_i2c->write(buf, len) != (int)len) return false;
    return ImuBNO08X::_i2c->endTransmission(stop) == 0;
}

bool ImuBNO08X::i2c_read_bytes(uint8_t* buf, size_t len, bool stop) {
    if (len == 0) return true;
    size_t recv = ImuBNO08X::_i2c->requestFrom((uint8_t)ImuBNO08X::_i2cAddress, (uint8_t)len, (uint8_t)stop);
    if (recv != len) return false;
    for (size_t i = 0; i < len; ++i) buf[i] = ImuBNO08X::_i2c->read();
    return true;
}

int ImuBNO08X::i2c_write(sh2_Hal_t*, uint8_t* pBuffer, unsigned len) {
    // Запись не требует ожидания INT по протоколу
    uint16_t write_size = (len > kI2CMax) ? kI2CMax : len;
    if (!i2c_write_bytes(pBuffer, write_size, true)) return 0;
    return write_size;
}

int ImuBNO08X::i2c_read(sh2_Hal_t*, uint8_t* pBuffer, unsigned len, uint32_t*) {
    if (!waitForIntIfNeeded()) return false;
    uint8_t header[4];
    if (!i2c_read_bytes(header, 4, true)) return 0;

    uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
    packet_size &= ~0x8000;

    if (packet_size > len) return 0;
    
    uint16_t cargo_remaining = packet_size;
    uint8_t i2c_buffer[kI2CMax];
    uint16_t read_size;
    uint16_t cargo_read_amount = 0;
    bool first_read = true;

    while (cargo_remaining > 0) {
        if (first_read) {
            read_size = (cargo_remaining > kI2CMax) ? kI2CMax : cargo_remaining;
        } else {
            read_size = (cargo_remaining + 4 > kI2CMax) ? kI2CMax : cargo_remaining + 4;
        }

        if (!i2c_read_bytes(i2c_buffer, read_size, true)) return 0;

        if (first_read) {
            cargo_read_amount = read_size;
            memcpy(pBuffer, i2c_buffer, cargo_read_amount);
            first_read = false;
        } else {
            cargo_read_amount = read_size - 4;
            memcpy(pBuffer, i2c_buffer + 4, cargo_read_amount);
        }
        pBuffer += cargo_read_amount;
        cargo_remaining -= cargo_read_amount;
    }

    return packet_size;
}

void ImuBNO08X::hal_event(void*, sh2_AsyncEvent_t* e) {
    if (e->eventId == SH2_RESET) {
        // reset occurred
    }
}

void ImuBNO08X::sensor_cb(void* cookie, sh2_SensorEvent_t* ev) {
    (void)cookie;
    if (sh2_decodeSensorEvent(&ImuBNO08X::_value, ev) != SH2_OK) {
        ImuBNO08X::_haveEvent = false;
        return;
    }
    ImuBNO08X::_haveEvent = true;
    switch (ImuBNO08X::_value.sensorId) {
        case SH2_ROTATION_VECTOR:
            currentOrientation.q0 = _value.un.rotationVector.real;
            currentOrientation.q1 = _value.un.rotationVector.i;
            currentOrientation.q2 = _value.un.rotationVector.j;
            currentOrientation.q3 = _value.un.rotationVector.k;
            currentOrientation.timestamp = millis();
            accuracy[5] = _value.status;
            _haveOrientation = true;
            break;
        case SH2_GAME_ROTATION_VECTOR:
            currentData.q0 = _value.un.gameRotationVector.real;
            currentData.q1 = _value.un.gameRotationVector.i;
            currentData.q2 = _value.un.gameRotationVector.j;
            currentData.q3 = _value.un.gameRotationVector.k;
            currentData.timestamp = millis();
            accuracy[4] = _value.status;
            _haveIMU = true;
            break;
        case SH2_ACCELEROMETER:
        case SH2_LINEAR_ACCELERATION:
            currentData.ax = _value.un.accelerometer.x;
            currentData.ay = _value.un.accelerometer.y;
            currentData.az = _value.un.accelerometer.z;
            currentData.timestamp = millis();
            accuracy[0] = _value.status;
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            currentData.gx = _value.un.gyroscope.x;
            currentData.gy = _value.un.gyroscope.y;
            currentData.gz = _value.un.gyroscope.z;
            currentData.timestamp = millis();
            accuracy[1] = _value.status;
            break;
        case SH2_MAGNETIC_FIELD_CALIBRATED:
            currentData.mx = _value.un.magneticField.x;
            currentData.my = _value.un.magneticField.y;
            currentData.mz = _value.un.magneticField.z;
            currentData.timestamp = millis();
            accuracy[2] = _value.status;
            break;
        case SH2_RAW_MAGNETOMETER:
            currentData.mag_x = _value.un.rawMagnetometer.x;
            currentData.mag_y = _value.un.rawMagnetometer.y;
            currentData.mag_z = _value.un.rawMagnetometer.z;
            currentData.timestamp = millis();
            accuracy[3] = _value.status;
            _haveIMU = true;
            break;
        default:
            break;
    }
}

bool ImuBNO08X::enableReport(sh2_SensorId_t id, uint32_t interval_us, uint32_t sensorSpecific) {
    sh2_SensorConfig_t cfg = {};
    cfg.changeSensitivityEnabled = false;
    cfg.wakeupEnabled = false;
    cfg.changeSensitivityRelative = false;
    cfg.alwaysOnEnabled = false;
    cfg.changeSensitivity = 0;
    cfg.batchInterval_us = 0;
    cfg.reportInterval_us = interval_us;
    cfg.sensorSpecific = sensorSpecific;
    return sh2_setSensorConfig(id, &cfg) == SH2_OK;
}

int ImuBNO08X::begin(TwoWire* i2c, uint8_t i2cAddress, int8_t interruptPin, int8_t rstPin) {
    _i2c = i2c ? i2c : &Wire;

    _i2cAddress = i2cAddress == 0 ? 0x4B : i2cAddress;
    _interruptPin = interruptPin;
    _rstPin = rstPin;
    if (_interruptPin >= 0) pinMode(_interruptPin, INPUT_PULLUP);
    if (_rstPin >= 0) pinMode(_rstPin, INPUT_PULLUP);

    _hal.open = i2c_open;
    _hal.close = i2c_close;
    _hal.read = i2c_read;
    _hal.write = i2c_write;
    _hal.getTimeUs = hal_getTimeUs;

    hwReset();
    sleepMs(200);

    _i2c->beginTransmission(_i2cAddress);
    if (_i2c->endTransmission(true) != 0) {
        if (logStream) logStream->println(F("I2C connection failed"));
        return 0;
    }

    if (sh2_open(&_hal, hal_event, nullptr) != SH2_OK) {
        if (logStream) logStream->println(F("sh2_open failed"));
        return 0;
    }

    sh2_setSensorCallback(sensor_cb, nullptr);
    return 1;
}

bool ImuBNO08X::setOrientationFrequency(uint16_t frequency, bool emulate) {
    orientationFrequency = frequency;
    uint32_t ori_us = orientationFrequency > 0 ? (1000000UL / orientationFrequency) : 10000UL;
    enableReport(SH2_ROTATION_VECTOR, ori_us);
    sleepMs(50);
    return true;
}


bool ImuBNO08X::setIMUFrequency(uint16_t frequency, bool emulate) {
    imuFrequency = frequency;
    uint32_t imu_us = imuFrequency > 0 ? (1000000UL / imuFrequency) : 10000UL;
    enableReport(SH2_GAME_ROTATION_VECTOR, imu_us);
    sleepMs(50);
    enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, imu_us);
    sleepMs(50);
    enableReport(SH2_RAW_MAGNETOMETER, imu_us);
    sleepMs(50);
    enableReport(SH2_ACCELEROMETER, imu_us);
    sleepMs(50);
    enableReport(SH2_GYROSCOPE_CALIBRATED, imu_us);
    sleepMs(50);
    return true;
}

void ImuBNO08X::startServices() {
    #if defined(ARDUINO_ARCH_ESP32)
    if (ImuBNO08X::_taskHandle == nullptr) {
        xTaskCreatePinnedToCore(ImuBNO08X::taskEntry, "imu_bno08x_task", 4096, this, 5, &ImuBNO08X::_taskHandle, tskNO_AFFINITY);
    }
    if (_interruptPin >= 0) {
        attachInterrupt(digitalPinToInterrupt(_interruptPin), ImuBNO08X::intISR, FALLING);
    }
    #endif
}

void ImuBNO08X::stopServices() {
    #if defined(ARDUINO_ARCH_ESP32)
    if (_interruptPin >= 0) {
        detachInterrupt(digitalPinToInterrupt(_interruptPin));
    }
    if (ImuBNO08X::_taskHandle) vTaskDelete(ImuBNO08X::_taskHandle);
    ImuBNO08X::_taskHandle = nullptr;
    #endif
}

#if defined(ARDUINO_ARCH_ESP32)
TaskHandle_t ImuBNO08X::_taskHandle = nullptr;

void ImuBNO08X::taskEntry(void* arg) {
    ImuBNO08X* imu = (ImuBNO08X*)arg;

    TickType_t last = xTaskGetTickCount();
    TickType_t period = pdMS_TO_TICKS(1000UL / (imu->imuFrequency == 0 ? 100 : imu->imuFrequency));
    for (;;) {
        if (ImuBNO08X::_interruptPin >= 0) {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
        } else {
            vTaskDelayUntil(&last, period);
        }
        

        sh2_service();

        if (ImuBNO08X::_haveOrientation && imu->orientationCb) {
            imu->orientationCb(imu->currentOrientation);
            ImuBNO08X::_haveOrientation = false;
        }
        if (ImuBNO08X::_haveIMU && imu->imuDataCb) {
            imu->imuDataCb(imu->currentData);
            ImuBNO08X::_haveIMU = false;
        }
    }
}

void IRAM_ATTR ImuBNO08X::intISR() {
    BaseType_t hpw = pdFALSE;
    if (ImuBNO08X::_taskHandle) vTaskNotifyGiveFromISR(ImuBNO08X::_taskHandle, &hpw);
    if (hpw) portYIELD_FROM_ISR();
}
#endif

// Implementation of pure virtual methods from ImuSensor
bool ImuBNO08X::orientationDataReady() {
    sh2_service();
    return _haveOrientation;
}

bool ImuBNO08X::imuDataReady() {
    sh2_service();
    return _haveIMU;
}

bool ImuBNO08X::getOrientation(OrientationData& orientation) {
    sh2_service();
    orientation = currentOrientation;
    _haveOrientation = false;
    return true;
}

bool ImuBNO08X::getData(IMUData& data) {
    sh2_service();
    data = currentData;
    _haveIMU = false;
    return true;
}

uint16_t ImuBNO08X::getCalibrationStatus() {
    return _value.status;
}

void ImuBNO08X::getCalibrationData(void* data) {
    (void)data;
}

void ImuBNO08X::setCalibrationData(void* data) {
    (void)data;
}