/**
 * @file SP_BLESerial.h
 * @brief Базовый класс для BLE Serial коммуникации SmartPaddle
 */

#pragma once
#include <Arduino.h>
#include "ByteRingBuffer.h"
#include <BLECharacteristic.h>
#include <ArduinoJson.h>

#define SP_SERIAL_BUFFER_SIZE 4096
#define SP_JSON_BUFFER_SIZE 512

// UUID для сервиса Serial
namespace SPSerialUUID {
    extern const char* SERVICE_UUID;
    extern const char* RX_UUID;
    extern const char* TX_UUID;
}

// Типы JSON сообщений
enum class MessageType {
    LOG,        // Лог сообщение
    COMMAND,    // Команда
    RESPONSE,   // Ответ на команду
    DATA,       // Данные датчиков
    STATUS      // Статус устройства
};

// Базовый класс для обработки сообщений
class BLESerialMessageHandler {
public:
    virtual void onLogMessage(const char* message) {}
    virtual void onCommand(const char* command, JsonObject& params) {}
    virtual void onResponse(const char* command, bool success, const char* message) {}
    virtual void onData(const char* dataType, JsonObject& data) {}
    virtual void onStatus(JsonObject& status) {}
};

class SmartPaddle;
class SP_BLESerial : public Stream {
protected:
    SmartPaddle* paddle;
    ByteRingBuffer<SP_SERIAL_BUFFER_SIZE> receiveBuffer;
    JsonDocument jsonDoc;
    char jsonBuffer[SP_JSON_BUFFER_SIZE];
    char jsonIncomingBuffer[SP_JSON_BUFFER_SIZE];
    int jsonIncomingBufferLength;
    int jsonIncomingBufferIndex;
    BLESerialMessageHandler* messageHandler;
    bool started;

    uint8_t transmitBuffer[ESP_GATT_MAX_ATTR_LEN];
    size_t transmitBufferLength;
    unsigned long lastFlushTime;    

    // Методы для работы с JSON
    void sendJson(MessageType type, const JsonDocument& doc);
    virtual void processJsonMessage(char* message);

public:
    SP_BLESerial(SmartPaddle* p);
    virtual ~SP_BLESerial() = default;

    void setMessageHandler(BLESerialMessageHandler* handler);

    // Методы отправки сообщений
    void log(const char* message);
    void sendCommand(const char* command, JsonObject* params = nullptr);
    void sendResponse(const char* command, bool success, const char* message);

    // Реализация Stream
    virtual int available() override;
    virtual int read() override;
    virtual size_t readBytes(uint8_t* buffer, size_t bufferSize) override;
    virtual int peek() override;
    virtual size_t write(uint8_t byte) override;
    virtual size_t write(const uint8_t* buffer, size_t bufferSize) override;
    virtual void flush() override = 0;

    // Методы BLE
    virtual void begin() = 0;
    virtual void end() = 0;
    virtual void update();

    // Методы для обработки сообщений
    virtual bool updateJSON(bool printOther = false);

};