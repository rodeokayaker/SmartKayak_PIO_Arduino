/**
 * @file SP_MessageHandler.h
 * @brief Универсальный обработчик сообщений с динамической регистрацией команд и данных
 * 
 * Новый подход позволяет регистрировать обработчики команд и данных через callback функции,
 * что устраняет необходимость создавать отдельную функцию для каждой команды.
 */

#pragma once
#include "SP_Message.h"
#include "SP_Types.h"
#include "../Core/Interfaces/IIMUSensor.h"
#include "../Core/Interfaces/ILoadCell.h"
#include <map>
#include <functional>

// Типы callback функций для обработчиков
using CommandCallback = std::function<void(SP_Command*)>;
using DataCallback = std::function<void(SP_Data*)>;

/**
 * @brief Универсальный обработчик сообщений SmartPaddle протокола
 * 
 * Поддерживает динамическую регистрацию обработчиков команд и данных,
 * что позволяет легко расширять функциональность без изменения базового класса.
 * 
 * Пример использования:
 * @code
 * class MyHandler : public SP_MessageHandler {
 * public:
 *     MyHandler(SmartPaddle* paddle) {
 *         registerCommand(SP_Protocol::Commands::CALIBRATE_IMU, 
 *             [paddle](SP_Command* cmd) {
 *                 paddle->calibrateIMU();
 *             });
 *     }
 * };
 * @endcode
 */
class SP_MessageHandler {
protected:
    std::map<String, CommandCallback> commandHandlers;
    std::map<String, DataCallback> dataHandlers;

public:
    SP_MessageHandler() {}
    
    // ============================================
    // РЕГИСТРАЦИЯ ОБРАБОТЧИКОВ
    // ============================================
    
    /**
     * @brief Регистрация обработчика команды
     * @param command Имя команды (из SP_Protocol::Commands)
     * @param handler Callback функция для обработки команды
     * 
     * Пример:
     * @code
     * registerCommand(SP_Protocol::Commands::SHUTDOWN, 
     *     [this](SP_Command* cmd) { handleShutdown(cmd); });
     * @endcode
     */
    void registerCommand(const char* command, CommandCallback handler) {
        commandHandlers[String(command)] = handler;
    }
    
    /**
     * @brief Регистрация обработчика данных
     * @param dataType Тип данных (из SP_Protocol::DataTypes)
     * @param handler Callback функция для обработки данных
     * 
     * Пример:
     * @code
     * registerData(SP_Protocol::DataTypes::SPECS,
     *     [this](SP_Data* data) { handleSpecs(data); });
     * @endcode
     */
    void registerData(const char* dataType, DataCallback handler) {
        dataHandlers[String(dataType)] = handler;
    }
    
    /**
     * @brief Удалить обработчик команды
     * @param command Имя команды
     */
    void unregisterCommand(const char* command) {
        commandHandlers.erase(String(command));
    }
    
    /**
     * @brief Удалить обработчик данных
     * @param dataType Тип данных
     */
    void unregisterData(const char* dataType) {
        dataHandlers.erase(String(dataType));
    }
    
    /**
     * @brief Очистить все обработчики команд
     */
    void clearCommandHandlers() {
        commandHandlers.clear();
    }
    
    /**
     * @brief Очистить все обработчики данных
     */
    void clearDataHandlers() {
        dataHandlers.clear();
    }
    
    // ============================================
    // ОБРАБОТКА СООБЩЕНИЙ
    // ============================================
    
    /**
     * @brief Обработка команды через зарегистрированные обработчики
     * @param command Указатель на команду
     * 
     * Ищет зарегистрированный обработчик для команды и вызывает его.
     * Если обработчик не найден, вызывается onUnknownCommand().
     */
    virtual void onCommand(SP_Command* command) {
        auto it = commandHandlers.find(command->command);
        if (it != commandHandlers.end()) {
            it->second(command);  // Вызываем зарегистрированный обработчик
        } else {
            onUnknownCommand(command);
        }
    }
    
    /**
     * @brief Обработка данных через зарегистрированные обработчики
     * @param data Указатель на данные
     * 
     * Ищет зарегистрированный обработчик для типа данных и вызывает его.
     * Если обработчик не найден, вызывается onUnknownData().
     */
    virtual void onData(SP_Data* data) {
        auto it = dataHandlers.find(data->dataType);
        if (it != dataHandlers.end()) {
            it->second(data);
        } else {
            onUnknownData(data);
        }
    }
    
    // ============================================
    // БАЗОВЫЕ ОБРАБОТЧИКИ
    // ============================================
    
    /**
     * @brief Обработка неизвестной команды
     * Переопределите для custom логики (например, логирование)
     */
    virtual void onUnknownCommand(SP_Command* command) {
        // По умолчанию ничего не делаем
        // Можно включить отладочный вывод при необходимости:
        // Serial.printf("Unknown command: %s\n", command->command.c_str());
    }
    
    /**
     * @brief Обработка неизвестного типа данных
     * Переопределите для custom логики
     */
    virtual void onUnknownData(SP_Data* data) {
        // По умолчанию ничего не делаем
    }
    
    /**
     * @brief Обработка ответа на команду
     */
    virtual void onResponse(SP_Response* response) {}
    
    /**
     * @brief Обработка лог-сообщения
     */
    virtual void onLog(SP_LogMessage* log) {}
    
    /**
     * @brief Обработка статусного сообщения
     */
    virtual void onStatus(SP_StatusMessage* status) {}
    
    /**
     * @brief Обработка любого сообщения (fallback)
     */
    virtual void onMessage(SP_Message* message) {}
    
    virtual ~SP_MessageHandler() = default;
};