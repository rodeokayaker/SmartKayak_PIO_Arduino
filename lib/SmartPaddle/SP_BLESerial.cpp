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
    // Удаляем JSON задачу только при уничтожении объекта
    // (не при отключении - задача переиспользуется при переподключении)
    stopJsonProcessTask();
}

void SP_BLESerial::setMessageHandler(SP_MessageHandler* handler) {
    messageProcessor.setHandler(handler);
}

void SP_BLESerial::startJsonProcessTask() {
    // Создаем задачу только один раз
    if (!taskRunning) {
        Serial.printf("Starting JSON task (stack: %d bytes, free heap: %d bytes)...\n", 
                     JSON_TASK_STACK_SIZE, ESP.getFreeHeap());
        
        // Создаем очередь для сообщений
        jsonMessageQueue = xQueueCreate(JSON_QUEUE_SIZE, sizeof(JsonMessageTask));
        if (jsonMessageQueue == NULL) {
            Serial.println("❌ Failed to create JSON message queue");
            return;
        }
        Serial.println("✓ JSON queue created");
        
        // Создаем задачу для обработки JSON
        BaseType_t taskCreated = xTaskCreate(
            jsonProcessTaskFunction,    // Функция задачи
            "JsonProcessTask",          // Имя задачи
            JSON_TASK_STACK_SIZE,       // Размер стека
            this,                       // Параметр (указатель на текущий объект)
            1,                          // Приоритет
            &jsonProcessTaskHandle      // Дескриптор задачи
        );
        
        if (taskCreated != pdPASS) {
            Serial.printf("❌ Failed to create JSON processing task (error: %d, free heap: %d)\n", 
                         taskCreated, ESP.getFreeHeap());
            vQueueDelete(jsonMessageQueue);
            jsonMessageQueue = NULL;
            return;
        }
        
        Serial.println("✓ JSON task created successfully! (will persist across reconnections)");
        taskRunning = true;
    } else {
        Serial.println("ℹ️  JSON task already running (reusing existing task)");
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
    JsonMessageTask signal;  // Теперь всего 1 байт вместо 4096!
    
    // Проверяем свободную память в начале задачи
    Serial.printf("JSON task started, free heap: %d bytes\n", ESP.getFreeHeap());
    
    while (true) {
        // Проверяем состояние памяти перед каждой итерацией
        if (ESP.getFreeHeap() < 5000) {
            Serial.printf("⚠️ JSON task: critical low memory (%d bytes), pausing\n", ESP.getFreeHeap());
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        
        // Проверяем, что очередь существует
        if (serial->jsonMessageQueue == NULL) {
            Serial.println("JSON message queue is NULL");
            vTaskDelay(pdMS_TO_TICKS(1000)); // Ждем секунду перед следующей попыткой
            continue;
        }
        
        // Ждем сигнала из очереди
        if (xQueueReceive(serial->jsonMessageQueue, &signal, portMAX_DELAY) == pdTRUE) {
            // Проверяем память перед обработкой
            if (ESP.getFreeHeap() > 10000) {
                // Обрабатываем JSON из статического буфера класса (НЕ со стека задачи!)
                serial->messageProcessor.processJson(serial->jsonTaskBuffer);
            } else {
                Serial.printf("⚠️ JSON task: skipping processing due to low memory (%d bytes)\n", ESP.getFreeHeap());
            }
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

    // Ограничиваем размер буфера для предотвращения переполнения
    size_t maxWrite = min(bufferSize, (size_t)ESP_GATT_MAX_ATTR_LEN - transmitBufferLength);
    if (maxWrite == 0) {
        flush(); // Принудительно отправляем буфер если он полный
        maxWrite = min(bufferSize, (size_t)ESP_GATT_MAX_ATTR_LEN - transmitBufferLength);
    }

    size_t written = 0;
    for (size_t i = 0; i < maxWrite; i++)
    {
        written += this->write(buffer[i]);
    }
    return written;
}

void SP_BLESerial::update() {
    if(!started || !paddle->connected()) return;
    
    // Проверяем состояние памяти
    if (ESP.getFreeHeap() < 10000) {
        Serial.printf("⚠️ Low memory: %d bytes, skipping update\n", ESP.getFreeHeap());
        return;
    }
        
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
                        // Копируем данные в буфер задачи
                        strncpy(jsonTaskBuffer, jsonIncomingBuffer, SP_JSON_BUFFER_SIZE - 1);
                        jsonTaskBuffer[SP_JSON_BUFFER_SIZE - 1] = '\0';
                        
                        // Отправляем только сигнал в очередь (1 байт вместо 4096!)
                        JsonMessageTask signal;
                        signal.signal = 1;
                        
                        if (xQueueSend(jsonMessageQueue, &signal, 0) != pdTRUE) {
                            // Если очередь заполнена, обрабатываем здесь
                            Serial.println("⚠️ JSON queue is full, processing synchronously");
                            messageProcessor.processJson(jsonIncomingBuffer);
                        }
                    } else {
                        // Если задача не запущена или очередь не создана, обрабатываем синхронно
                        // (Это нормальный fallback режим, JSON всё равно обрабатывается)
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