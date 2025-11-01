/**
 * @file SP_BLESerialServer.h
 * @brief Серверная часть BLE Serial для SmartPaddle
 */

#pragma once
#include "SP_BLESerial.h"
#include "SmartPaddleServer.h"
#include <BLE2902.h>

class SP_BLESerialServer : public SP_BLESerial, public BLECharacteristicCallbacks {
private:
    BLEService* serialService;
    BLECharacteristic* txChar;
    BLECharacteristic* rxChar;
    

    void setupSerialService() {
        BLEServer* server = static_cast<SmartPaddleBLEServer*>(paddle)->getBLEServer();
        
        // Создаем сервис Serial
        serialService = server->createService(BLEUUID(SPSerialUUID::SERVICE_UUID),30,0);
        
        // Создаем характеристики
        rxChar = serialService->createCharacteristic(
            SPSerialUUID::RX_UUID,
            BLECharacteristic::PROPERTY_WRITE | 
            BLECharacteristic::PROPERTY_WRITE_NR
        );
        
        txChar = serialService->createCharacteristic(
            SPSerialUUID::TX_UUID,
            BLECharacteristic::PROPERTY_READ | 
            BLECharacteristic::PROPERTY_NOTIFY
        );
        
        // Добавляем дескрипторы
        txChar->addDescriptor(new BLE2902());
        rxChar->addDescriptor(new BLE2902());
        
        // Устанавливаем callback
        rxChar->setCallbacks(this);
        
        // Запускаем сервис
        serialService->start();
    }

public:
    SP_BLESerialServer(SmartPaddleBLE* p) : 
        SP_BLESerial(p)
         {        }


    void flush() override {
        if(!started || !paddle->connected()) return;

        if(transmitBufferLength == 0) {
            lastFlushTime = millis();
            return;
        }

        txChar->setValue(transmitBuffer, transmitBufferLength);
        txChar->notify();

        transmitBufferLength = 0;
        lastFlushTime = millis();
    }

    // Методы BLE
    void begin() override {
        if(started) return;
        setupSerialService();
        started = true;
        // Создаем JSON задачу только при первом запуске
        // При последующих подключениях она будет переиспользоваться
        startJsonProcessTask();
    }

    void end() override {
        if(!started) return;
        flush();
        serialService->stop();
        started = false;
        // НЕ останавливаем JSON задачу - она будет работать при переподключении
        // Задача удаляется только в деструкторе ~SP_BLESerial()
    }

    void update() override {
        if(!started) return;
        
        if(transmitBufferLength > 0 && 
           (millis() - lastFlushTime > 20)) {
            flush();
        }
    }

    // BLECharacteristicCallbacks
    void onWrite(BLECharacteristic* pCharacteristic) override {
        if(!started) return;
        
        if(pCharacteristic->getUUID().toString() == SPSerialUUID::RX_UUID) {
            std::string value = pCharacteristic->getValue();

            // Добавляем полученные данные в буфер
            for(const char& c : value) {
                receiveBuffer.add(c);
            }
        }
    }

};