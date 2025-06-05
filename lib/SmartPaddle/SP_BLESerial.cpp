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
    jsonProcessTaskHandle(NULL),
    jsonMessageQueue(NULL),
    taskRunning(false),
    messageProcessor() {}

SP_BLESerial::~SP_BLESerial() {
    stopJsonProcessTask();
}

void SP_BLESerial::setMessageHandler(SP_MessageHandler* handler) {
    messageProcessor.setHandler(handler);
}

void SP_BLESerial::startJsonProcessTask() {
    if (!taskRunning) {
        // Создаем очередь для сообщений
        jsonMessageQueue = xQueueCreate(JSON_QUEUE_SIZE, sizeof(JsonMessageTask));
        if (jsonMessageQueue == NULL) {
            Serial.println("Failed to create JSON message queue");
            return;
        }
        
        // Создаем задачу для обработки JSON
        BaseType_t taskCreated = xTaskCreate(
            jsonProcessTaskFunction,    // Функция задачи
            "JsonProcessTask",          // Имя задачи
            JSON_TASK_STACK_SIZE,       // Увеличенный размер стека
            this,                       // Параметр (указатель на текущий объект)
            1,                          // Приоритет
            &jsonProcessTaskHandle      // Дескриптор задачи
        );
        
        if (taskCreated != pdPASS) {
            Serial.println("Failed to create JSON processing task");
            vQueueDelete(jsonMessageQueue);
            jsonMessageQueue = NULL;
            return;
        }
        
        taskRunning = true;
    }
}

void SP_BLESerial::stopJsonProcessTask() {
    if (taskRunning) {
        if (jsonProcessTaskHandle != NULL) {
            vTaskDelete(jsonProcessTaskHandle);
            jsonProcessTaskHandle = NULL;
        }
        
        if (jsonMessageQueue != NULL) {
            vQueueDelete(jsonMessageQueue);
            jsonMessageQueue = NULL;
        }
        
        taskRunning = false;
    }
}

void SP_BLESerial::jsonProcessTaskFunction(void* parameter) {
    SP_BLESerial* serial = static_cast<SP_BLESerial*>(parameter);
    JsonMessageTask message;
    
    while (true) {
        // Проверяем, что очередь существует
        if (serial->jsonMessageQueue == NULL) {
            Serial.println("JSON message queue is NULL");
            vTaskDelay(pdMS_TO_TICKS(1000)); // Ждем секунду перед следующей попыткой
            continue;
        }
        
        // Ждем сообщения из очереди
        if (xQueueReceive(serial->jsonMessageQueue, &message, portMAX_DELAY) == pdTRUE) {
            // Обрабатываем JSON
            serial->messageProcessor.processJson(message.message);
        }
    }
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
//    while (receiveBuffer.getLength() > 0) {
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
                    
                    if (taskRunning && jsonMessageQueue != NULL) {
                        // Создаем структуру для сообщения
                        JsonMessageTask message;
                        // Копируем сообщение в структуру
                        strncpy(message.message, jsonIncomingBuffer, SP_JSON_BUFFER_SIZE - 1);
                        message.message[SP_JSON_BUFFER_SIZE - 1] = '\0';
                        
                        // Отправляем сообщение в очередь, с таймаутом 0 (неблокирующий режим)
                        if (xQueueSend(jsonMessageQueue, &message, 0) != pdTRUE) {
                            // Если очередь заполнена, обрабатываем здесь
                            Serial.println("JSON queue is full");
                            messageProcessor.processJson(jsonIncomingBuffer);

                        }
                    } else {
                        // Если задача не запущена или очередь не создана, обрабатываем здесь
                        Serial.println("JSON task is not running");
                        messageProcessor.processJson(jsonIncomingBuffer);
                    }
                    
                    jsonIncomingBufferIndex = 0;
                    jsonIncomingBufferLength = 0;
                    break;
                }
            }
        }
//    }
    return true;
}

void SP_BLESerial::sendString(const String& str) {
    if(!started || !paddle->connected()) return;
    flush();
    this->println(str);
    flush();
}