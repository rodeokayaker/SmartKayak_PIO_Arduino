/**
 * @file SP_MessageProcessor.h
 * @brief Упрощенный процессор сообщений для SmartPaddle протокола
 * 
 * Процессор десериализует JSON сообщения и передает их в SP_MessageHandler.
 * Больше не содержит специфичной логики обработки команд - это делегируется
 * зарегистрированным callback'ам в handler'е.
 */

#pragma once
#include "SP_Protocol.h"
#include "SP_Message.h"
#include "SP_MessageHandler.h"
#include "SP_CommandParams.h"
#include <ArduinoJson.h>
#include "SP_Types.h"
#include "../Core/Interfaces/IIMUSensor.h"
#include "../Core/Interfaces/ILoadCell.h"

/**
 * @brief Процессор сообщений для SmartPaddle протокола
 * 
 * Отвечает за:
 * - Десериализацию JSON сообщений
 * - Определение типа сообщения
 * - Передачу сообщений в соответствующие обработчики
 * - Создание JSON сообщений для отправки
 */
class SP_MessageProcessor {
private:
    SP_MessageHandler* handler;  ///< Обработчик сообщений
    static JsonDocument doc;     ///< Статический документ для переиспользования

public:
    /**
     * @brief Конструктор
     * @param msgHandler Указатель на обработчик сообщений
     */
    SP_MessageProcessor(SP_MessageHandler* msgHandler = nullptr);
    
    /**
     * @brief Установить обработчик сообщений
     * @param msgHandler Указатель на обработчик
     */
    void setHandler(SP_MessageHandler* msgHandler);
    
    /**
     * @brief Обработать JSON строку
     * @param jsonStr JSON строка для обработки
     * 
     * Десериализует JSON, определяет тип сообщения и вызывает
     * соответствующий метод обработчика.
     */
    void processJson(const char* jsonStr);

    // ============================================
    // СОЗДАНИЕ БАЗОВЫХ СООБЩЕНИЙ
    // ============================================
    
    /**
     * @brief Создать JSON сообщение команды
     * @param command Имя команды
     * @param params Опциональные параметры
     * @return JSON строка
     */
    static String createCommandMessage(const char* command, JsonObject* params = nullptr);
    
    /**
     * @brief Создать JSON сообщение ответа
     * @param command Имя команды, на которую отвечаем
     * @param success Успешность выполнения
     * @param message Текстовое сообщение
     * @return JSON строка
     */
    static String createResponseMessage(const char* command, bool success, const char* message);
    
    /**
     * @brief Создать JSON сообщение с данными
     * @param dataType Тип данных
     * @param value Значения данных
     * @return JSON строка
     */
    static String createDataMessage(const char* dataType, JsonObject* value);
    
    /**
     * @brief Создать JSON лог-сообщение
     * @param message Текст лога
     * @return JSON строка
     */
    static String createLogMessage(const char* message);
    
    /**
     * @brief Создать JSON статусное сообщение
     * @param status Данные статуса
     * @return JSON строка
     */
    static String createStatusMessage(JsonObject* status);

    // ============================================
    // СОЗДАНИЕ КОМАНД (удобные методы)
    // ============================================
    
    static String createCalibrateCompassCommand();
    static String createCalibrateLoadsCommand(BladeSideType bladeSide);
    static String createCalibrateIMUCommand();
    static String createCalibrateBladeAngleCommand(BladeSideType bladeSide);
    static String createSendSpecsCommand();
    static String createStartPairCommand();
    static String createShutdownCommand();
    static String createTareLoadsCommand(BladeSideType bladeSide);
    static String createSendCalibrationDataCommand();
    static String createSetMagnetometerOffsetCommand(float* offset);
    static String createSetMagnetometerCalibrationCommand(float* offset, float* scale, float* softIron);

    // ============================================
    // СОЗДАНИЕ СООБЩЕНИЙ С ДАННЫМИ
    // ============================================
    
    static String createBladeOrientationMessage(const BladeOrientation& orientation);
    static String createSpecsMessage(const PaddleSpecs& specs);
    static String createStatusMessage(const PaddleStatus& status);

    // ============================================
    // СОЗДАНИЕ ОТВЕТОВ (удобные методы)
    // ============================================
    
    /**
     * @brief Создать успешный ответ
     * @param command Команда, на которую отвечаем
     * @param message Опциональное сообщение
     * @return JSON строка
     */
    static String createSuccessResponse(const char* command, const char* message = "");
    
    /**
     * @brief Создать ответ об ошибке
     * @param command Команда, на которую отвечаем
     * @param message Опциональное сообщение об ошибке
     * @return JSON строка
     */
    static String createErrorResponse(const char* command, const char* message = "");
};
