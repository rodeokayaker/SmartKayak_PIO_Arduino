#ifndef OVERWRITING_QUEUE_H
#define OVERWRITING_QUEUE_H

#include <Arduino.h>

// Класс для очереди с перезаписью
template<typename T>
class OverwritingQueue {
private:
    QueueHandle_t queue;
    size_t maxSize;

public:
    OverwritingQueue(size_t size) : maxSize(size) {
        queue = xQueueCreate(size, sizeof(T));
    }

    void send(const T& data) {
        if (uxQueueSpacesAvailable(queue) == 0) {
            T dummy;
            xQueueReceive(queue, &dummy, 0);
        }
        xQueueSendToBack(queue, &data, 0);
    }

    bool receive(T& data, TickType_t timeout = portMAX_DELAY) {
        return xQueueReceive(queue, &data, timeout) == pdTRUE;
    }

    ~OverwritingQueue() {
        vQueueDelete(queue);
    }

    // Получить количество элементов в очереди
    size_t available() {
        return uxQueueMessagesWaiting(queue);
    }
    
    // Получить количество свободных мест
    size_t freeSpace() {
        return uxQueueSpacesAvailable(queue);
    }
    
    // Получить максимальный размер очереди
    size_t getMaxSize() {
        return maxSize;
    }
    
    // Проверить, пуста ли очередь
    bool isEmpty() {
        return available() == 0;
    }
    
    // Проверить, заполнена ли очередь
    bool isFull() {
        return freeSpace() == 0;
    }
};



#endif