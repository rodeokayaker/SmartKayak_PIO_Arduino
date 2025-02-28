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
    started(false),
    receiveBuffer(),
    transmitBufferLength(0),
    lastFlushTime(0),
    jsonIncomingBufferLength(0),
    jsonIncomingBufferIndex(0),
    messageProcessor() {}

void SP_BLESerial::setMessageHandler(SP_MessageHandler* handler) {
    messageProcessor.setHandler(handler);
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
//    flush();
    return written;
}

void SP_BLESerial::update() {
    if(!started) return;
        
    if(transmitBufferLength > 0 && 
       (millis() - lastFlushTime > 20)) {
        flush();
    }
}

void SP_BLESerial::sendJson(const JsonDocument& doc) {
    if(!started || !paddle->connected()) return;
    
    serializeJson(doc, jsonBuffer);

    flush();
    println(jsonBuffer);
    flush();
}

void SP_BLESerial::sendMessage(SP_Message& msg) {
    if(!started || !paddle->connected()) return;
    sendJson(msg.serializeDocument());
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
                messageProcessor.processJson(jsonIncomingBuffer);
                jsonIncomingBufferIndex = 0;
                jsonIncomingBufferLength = 0;
                break;
            }
        }
    }
    return true;
}

void SP_BLESerial::sendString(const String& str) {
    if(!started || !paddle->connected()) return;
    flush();
    this->println(str);
    flush();
}