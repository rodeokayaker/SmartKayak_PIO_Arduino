/**
 * @file SP_BLESerial.cpp
 * @brief Реализация базового класса для BLE Serial коммуникации SmartPaddle
 */

#include "SP_BLESerial.h"
#include <ArduinoJson.h>
#include "SmartPaddle.h"
#include <BLEDevice.h>
namespace SPSerialUUID {
    const char* SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
    const char* RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
    const char* TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";
}

SP_BLESerial::SP_BLESerial(SmartPaddle* p) : 
    paddle(p),
    messageHandler(nullptr),
    started(false),
    receiveBuffer(),
    transmitBufferLength(0),
    lastFlushTime(0),
    jsonIncomingBufferLength(0),
    jsonIncomingBufferIndex(0) {}

void SP_BLESerial::setMessageHandler(BLESerialMessageHandler* handler) {
    messageHandler = handler;
}

// Реализация Stream
int SP_BLESerial::available(){
    return receiveBuffer.getLength();
}

int SP_BLESerial::read(){
    return receiveBuffer.pop();
}

int SP_BLESerial::peek(){
    if(receiveBuffer.getLength() == 0) return -1;
    return receiveBuffer.get(0);
}

size_t SP_BLESerial::readBytes(uint8_t* buffer, size_t bufferSize){
    if(!started || !paddle->connected()) return 0;
    size_t read = 0;
    for(int i = 0; i < bufferSize; i++) {
        buffer[i] = receiveBuffer.pop();
        read++;
    }
    return read;
}


size_t SP_BLESerial::write(uint8_t byte){
    if(!started || !paddle->connected()) return 0;

    transmitBuffer[transmitBufferLength++] = byte;
        
    if((transmitBufferLength >= ESP_GATT_MAX_ATTR_LEN) || 
       (millis() - lastFlushTime > 20) || (transmitBufferLength > BLEDevice::getMTU()-4)) {
        flush();
    }
        
    return 1;
}

size_t SP_BLESerial::write(const uint8_t* buffer, size_t bufferSize){
    if (!started || !paddle->connected())
        return 0;

    size_t written = 0;
    for (int i = 0; i < bufferSize; i++)
    {
        written += this->write(buffer[i]);
    }
    flush();
    return written;
}

void SP_BLESerial::update() {
    if(!started) return;
        
    if(transmitBufferLength > 0 && 
       (millis() - lastFlushTime > 20)) {
        flush();
    }
}



void SP_BLESerial::sendJson(MessageType type, const JsonDocument& doc) {
    if(!started || !paddle->connected()) return;
    
    serializeJson(doc, jsonBuffer);

    flush();
    println(jsonBuffer);
}

bool SP_BLESerial::updateJSON(bool printOther) 
{   
    if(!started || !paddle->connected()) return false;

    char ch;
    if (jsonIncomingBufferIndex==0) {
        while(receiveBuffer.getLength() > 0) {
            ch=receiveBuffer.pop();
            if(ch=='{') {
                jsonIncomingBufferIndex=1;
                jsonIncomingBuffer[0]=ch;
                jsonIncomingBufferLength=1;
                break;
            }
            if (printOther) {
                Serial.printf("%c", ch);
            }
        }
    }
    if (jsonIncomingBufferIndex>0){
        while (receiveBuffer.getLength() > 0) {
            ch=receiveBuffer.pop();
            jsonIncomingBuffer[jsonIncomingBufferLength++] = ch;

            if (ch=='{') jsonIncomingBufferIndex++; 
            else if (ch=='}') jsonIncomingBufferIndex--;

            if (jsonIncomingBufferIndex==0) 
            {
                jsonIncomingBuffer[jsonIncomingBufferLength] = '\0';
                processJsonMessage(jsonIncomingBuffer);
                jsonIncomingBufferIndex = 0;
                jsonIncomingBufferLength = 0;
                break;
            }
        }
    }
    return true;
}

void SP_BLESerial::processJsonMessage(char* message) {
    DeserializationError error = deserializeJson(jsonDoc, message);
    if(error || !messageHandler) return;

    const char* typeStr = jsonDoc["type"];
    if(!typeStr) return;

    JsonObject data = jsonDoc["data"];
    
    if(strcmp(typeStr, "log") == 0) {
        messageHandler->onLogMessage(data["msg"]);
    }
    else if(strcmp(typeStr, "cmd") == 0) {
        JsonObject params = data["params"];
        messageHandler->onCommand(data["cmd"], params);
    }
    else if(strcmp(typeStr, "response") == 0) {
        messageHandler->onResponse(data["cmd"], data["success"], data["msg"]);
    }
    else if(strcmp(typeStr, "data") == 0) {
        JsonObject value = data["value"];
        messageHandler->onData(data["dataType"], value);
    }
    else if(strcmp(typeStr, "status") == 0) {
        messageHandler->onStatus(data);
    }
}

void SP_BLESerial::log(const char* message) {
    jsonDoc.clear();
    jsonDoc["type"] = "log";
    jsonDoc["data"]["msg"] = message;
    sendJson(MessageType::LOG, jsonDoc);
}

void SP_BLESerial::sendCommand(const char* command, JsonObject* params) {
    jsonDoc.clear();
    jsonDoc["type"] = "cmd";
    jsonDoc["data"]["cmd"] = command;
    if(params) jsonDoc["data"]["params"] = *params;
    sendJson(MessageType::COMMAND, jsonDoc);
}

void SP_BLESerial::sendResponse(const char* command, bool success, const char* message) {
    jsonDoc.clear();
    jsonDoc["type"] = "response";
    jsonDoc["data"]["cmd"] = command;
    jsonDoc["data"]["success"] = success;
    jsonDoc["data"]["msg"] = message;
    sendJson(MessageType::RESPONSE, jsonDoc);
} 