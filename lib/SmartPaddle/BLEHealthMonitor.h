/**
 * @file BLEHealthMonitor.h
 * @brief Мониторинг состояния Bluetooth стека для предотвращения ошибок
 */

#ifndef BLE_HEALTH_MONITOR_H
#define BLE_HEALTH_MONITOR_H

#include <Arduino.h>
#include <BLEDevice.h>

class BLEHealthMonitor {
private:
    static unsigned long lastHealthCheck;
    static unsigned long lastMemoryCheck;
    static bool bleHealthy;
    static uint32_t errorCount;
    static uint32_t lastFreeHeap;

public:
    static void begin() {
        lastHealthCheck = millis();
        lastMemoryCheck = millis();
        bleHealthy = true;
        errorCount = 0;
        lastFreeHeap = ESP.getFreeHeap();
    }

    static bool checkBLEHealth() {
        unsigned long now = millis();
        
        // Проверяем здоровье BLE каждые 5 секунд
        if (now - lastHealthCheck > 5000) {
            lastHealthCheck = now;
            
            // Проверяем инициализацию BLE
            if (!BLEDevice::getInitialized()) {
                Serial.println("⚠️ BLE not initialized!");
                bleHealthy = false;
                errorCount++;
                return false;
            }
            
            // Проверяем состояние памяти
            uint32_t currentFreeHeap = ESP.getFreeHeap();
            if (currentFreeHeap < 15000) {
                Serial.printf("⚠️ Low memory: %d bytes (was %d)\n", currentFreeHeap, lastFreeHeap);
                if (currentFreeHeap < 10000) {
                    bleHealthy = false;
                    errorCount++;
                    return false;
                }
            }
            
            lastFreeHeap = currentFreeHeap;
            bleHealthy = true;
        }
        
        return bleHealthy;
    }

    static void reportError(const char* error) {
        errorCount++;
        Serial.printf("❌ BLE Error #%d: %s (free heap: %d)\n", errorCount, error, ESP.getFreeHeap());
        
        if (errorCount > 10) {
            Serial.println("🚨 Too many BLE errors, requesting restart...");
            ESP.restart();
        }
    }

    static bool isHealthy() {
        return bleHealthy && (ESP.getFreeHeap() > 10000);
    }

    static void reset() {
        errorCount = 0;
        bleHealthy = true;
    }
};

unsigned long BLEHealthMonitor::lastHealthCheck = 0;
unsigned long BLEHealthMonitor::lastMemoryCheck = 0;
bool BLEHealthMonitor::bleHealthy = true;
uint32_t BLEHealthMonitor::errorCount = 0;
uint32_t BLEHealthMonitor::lastFreeHeap = 0;
#endif // BLE_HEALTH_MONITOR_H
