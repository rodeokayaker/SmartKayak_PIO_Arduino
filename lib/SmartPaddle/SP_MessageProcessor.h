/**
 * @file SP_MessageProcessor.h
 * @brief Процессор сообщений для SmartPaddle протокола
 */

#pragma once
#include "SP_Protocol.h"
#include "SP_Message.h"
#include "SP_MessageHandler.h"
#include <ArduinoJson.h>
#include "SP_Types.h" // Для доступа к структурам данных
#include "InterfaceIMU.h"
#include "InterfaceLoadCell.h"

/**
 * @brief Процессор сообщений для SmartPaddle протокола
 */

class SP_MessageProcessor {
private:
    SP_MessageHandler* handler;  ///< Обработчик сообщений

    // Вспомогательные методы для извлечения параметров
    template<typename T>
    static inline T getParam(const JsonObject& params, const char* paramName, T defaultValue) {
        return (params[paramName].is<T>()) ? params[paramName].as<T>() : defaultValue;
    }

    // Новые методы для обработки команд
    void processCommand(SP_Command* command);
    void processResponse(SP_Response* response);
    void processData(SP_Data* data);
    void processStatus(SP_StatusMessage* status);


public:
    SP_MessageProcessor(SP_MessageHandler* msgHandler = nullptr);
    void setHandler(SP_MessageHandler* msgHandler);
    void processJson(const char* jsonStr);

    // Базовые методы создания сообщений
    static String createCommandMessage(const char* command, JsonObject* params = nullptr);
    static String createResponseMessage(const char* command, bool success, const char* message);
    static String createDataMessage(const char* dataType, JsonObject* value);
    static String createLogMessage(const char* message);
    static String createStatusMessage(JsonObject* status);    

    // Команды
    static String createCalibrateCompassCommand();
    static String createCalibrateLoadsCommand(BladeSideType bladeSide);
    static String createCalibrateIMUCommand();
    static String createCalibrateBladeAngleCommand(BladeSideType bladeSide);
    static String createSendSpecsCommand();
    static String createStartPairCommand();
    static String createShutdownCommand();
    static String createTareLoadsCommand(BladeSideType bladeSide);

    // Данные датчиков
    static String createIMUDataMessage(const IMUData& data);
    static String createLoadDataMessage(const loadData& data);
    static String createOrientationDataMessage(const OrientationData& data);
    static String createBladeOrientationMessage(const BladeOrientation& orientation);
    static String createSpecsMessage(const PaddleSpecs& specs);
    static String createStatusMessage(const PaddleStatus& status);

    // Ответы
    static String createSuccessResponse(const char* command, const char* message = "");
    static String createErrorResponse(const char* command, const char* message = "");
    
}; 