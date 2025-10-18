/**
 * @file OverwritingQueue.h
 * @brief Template class for FreeRTOS queue with overwriting behavior
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#ifndef CORE_OVERWRITING_QUEUE_H
#define CORE_OVERWRITING_QUEUE_H

#include <Arduino.h>

/**
 * @class OverwritingQueue
 * @brief Template class for queue that overwrites oldest data when full
 * 
 * This class wraps FreeRTOS queue functionality with automatic overwriting
 * of the oldest element when the queue is full.
 */
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

    // Get number of elements in queue
    size_t available() {
        return uxQueueMessagesWaiting(queue);
    }
    
    // Get number of free spaces
    size_t freeSpace() {
        return uxQueueSpacesAvailable(queue);
    }
    
    // Get maximum queue size
    size_t getMaxSize() {
        return maxSize;
    }
    
    // Check if queue is empty
    bool isEmpty() {
        return available() == 0;
    }
    
    // Check if queue is full
    bool isFull() {
        return freeSpace() == 0;
    }
};

#endif // CORE_OVERWRITING_QUEUE_H

