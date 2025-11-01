/**
 * @file SP_MessageProcessor.cpp
 * @brief Реализация упрощенного процессора сообщений
 */

#include "SP_MessageProcessor.h"
#include <ArduinoJson.h>

JsonDocument SP_MessageProcessor::doc;  // Определение статического документа

SP_MessageProcessor::SP_MessageProcessor(SP_MessageHandler* msgHandler) : handler(msgHandler) {}

void SP_MessageProcessor::setHandler(SP_MessageHandler* msgHandler) {
    handler = msgHandler;
}

void SP_MessageProcessor::processJson(const char* jsonStr) {
    if (!handler) return;

    doc.clear();  // Очищаем документ перед использованием
    DeserializationError error = deserializeJson(doc, jsonStr);
    if (error) return;

    const char* typeStr = doc["type"];
    if (!typeStr) return;

    // Создаем сообщения на стеке и передаем напрямую в handler
    // Handler сам разберется с командами через зарегистрированные callback'и
    if (strcmp(typeStr, SP_Protocol::MessageType::LOG) == 0) {
        SP_LogMessage msg(doc);
        handler->onLog(&msg);
    } else if (strcmp(typeStr, SP_Protocol::MessageType::COMMAND) == 0) {
        SP_Command msg(doc);
        handler->onCommand(&msg);  // Напрямую в handler
    } else if (strcmp(typeStr, SP_Protocol::MessageType::RESPONSE) == 0) {
        SP_Response msg(doc);
        handler->onResponse(&msg);
    } else if (strcmp(typeStr, SP_Protocol::MessageType::DATA) == 0) {
        SP_Data msg(doc);
        handler->onData(&msg);  // Напрямую в handler
    } else if (strcmp(typeStr, SP_Protocol::MessageType::STATUS) == 0) {
        SP_StatusMessage msg(doc);
        handler->onStatus(&msg);
    } else {
        SP_Message msg(doc);
        handler->onMessage(&msg);
    }
}

// ============================================
// СОЗДАНИЕ БАЗОВЫХ СООБЩЕНИЙ
// ============================================

String SP_MessageProcessor::createCommandMessage(const char* command, JsonObject* params) {
    SP_Command msg(command);
    if (params){
        msg.params = *params;
        msg.serializeDocument();
    }
    return msg.serialize();
}

String SP_MessageProcessor::createResponseMessage(const char* command, bool success, const char* message) {
    SP_Response msg(command, success, message);
    return msg.serialize();
}

String SP_MessageProcessor::createDataMessage(const char* dataType, JsonObject* value) {
    SP_Data msg(dataType);
    if (value) {
        msg.value = *value;
        msg.serializeDocument();
    }
    return msg.serialize();
}

String SP_MessageProcessor::createLogMessage(const char* message) {
    SP_LogMessage msg(message);
    return msg.serialize();
}

String SP_MessageProcessor::createStatusMessage(JsonObject* status) {
    SP_StatusMessage msg(status);
    return msg.serialize();
}

// ============================================
// СОЗДАНИЕ КОМАНД (удобные методы)
// ============================================

String SP_MessageProcessor::createCalibrateCompassCommand() {
    return createCommandMessage(SP_Protocol::Commands::CALIBRATE_COMPASS);
}

String SP_MessageProcessor::createCalibrateLoadsCommand(BladeSideType bladeSide) {
    SP_Command cmd(SP_Protocol::Commands::CALIBRATE_LOADS);
    cmd.params[SP_Protocol::Commands::Params::BLADE_SIDE] = bladeSide;
    return cmd.serialize();
}

String SP_MessageProcessor::createCalibrateIMUCommand() {
    return createCommandMessage(SP_Protocol::Commands::CALIBRATE_IMU);
}

String SP_MessageProcessor::createCalibrateBladeAngleCommand(BladeSideType bladeSide) {
    SP_Command cmd(SP_Protocol::Commands::CALIBRATE_BLADE_ANGLE);
    cmd.params[SP_Protocol::Commands::Params::BLADE_SIDE] = bladeSide;
    return cmd.serialize();
}

String SP_MessageProcessor::createSendSpecsCommand() {
    return createCommandMessage(SP_Protocol::Commands::SEND_SPECS);
}

String SP_MessageProcessor::createSendPaddleOrientationCommand() {
    return createCommandMessage(SP_Protocol::Commands::SEND_PADDLE_ORIENTATION);
}

String SP_MessageProcessor::createStartPairCommand() {
    return createCommandMessage(SP_Protocol::Commands::START_PAIR);
}

String SP_MessageProcessor::createShutdownCommand() {
    return createCommandMessage(SP_Protocol::Commands::SHUTDOWN);
}

String SP_MessageProcessor::createTareLoadsCommand(BladeSideType bladeSide) {
    SP_Command cmd(SP_Protocol::Commands::TARE_LOADS);
    cmd.params[SP_Protocol::Commands::Params::BLADE_SIDE] = bladeSide;
    return cmd.serialize();
}

String SP_MessageProcessor::createSendCalibrationDataCommand() {
    return createCommandMessage(SP_Protocol::Commands::SEND_CALIBRATION_DATA);
}

String SP_MessageProcessor::createSetMagnetometerOffsetCommand(float* offset) {
    SP_Command cmd(SP_Protocol::Commands::SET_MAGNETOMETER_OFFSET);
    cmd.params[SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_X] = offset[0];
    cmd.params[SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_Y] = offset[1];
    cmd.params[SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_Z] = offset[2];
    return cmd.serialize();
}

String SP_MessageProcessor::createSetMagnetometerCalibrationCommand(float* offset, float* scale, float* softIron) {
    SP_Command cmd(SP_Protocol::Commands::SET_MAGNETOMETER_CALIBRATION);
    cmd.params[SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_X] = offset[0];
    cmd.params[SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_Y] = offset[1];
    cmd.params[SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_Z] = offset[2];
    cmd.params[SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_0_0] = scale[0];
    cmd.params[SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_1_1] = scale[1];
    cmd.params[SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_2_2] = scale[2];
    cmd.params[SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_0_1] = softIron[0];
    cmd.params[SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_0_2] = softIron[1];
    cmd.params[SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_1_2] = softIron[2];
    return cmd.serialize();
}

// ============================================
// СОЗДАНИЕ СООБЩЕНИЙ С ДАННЫМИ
// ============================================

String SP_MessageProcessor::createBladeOrientationMessage(const BladeOrientation& orientation) {
    SP_Data msg(SP_Protocol::DataTypes::BLADE_ORIENTATION);
    msg.value[SP_Protocol::DataTypes::BladeOrientation::RIGHT_BLADE_ANGLE] = orientation.rightBladeAngle;
    msg.value[SP_Protocol::DataTypes::BladeOrientation::LEFT_BLADE_ANGLE] = orientation.leftBladeAngle;
    msg.value[SP_Protocol::DataTypes::BladeOrientation::RIGHT_BLADE_VECTOR_X] = orientation.rightBladeVector[0];
    msg.value[SP_Protocol::DataTypes::BladeOrientation::RIGHT_BLADE_VECTOR_Y] = orientation.rightBladeVector[1];
    msg.value[SP_Protocol::DataTypes::BladeOrientation::RIGHT_BLADE_VECTOR_Z] = orientation.rightBladeVector[2];
    msg.value[SP_Protocol::DataTypes::BladeOrientation::LEFT_BLADE_VECTOR_X] = orientation.leftBladeVector[0];
    msg.value[SP_Protocol::DataTypes::BladeOrientation::LEFT_BLADE_VECTOR_Y] = orientation.leftBladeVector[1];
    msg.value[SP_Protocol::DataTypes::BladeOrientation::LEFT_BLADE_VECTOR_Z] = orientation.leftBladeVector[2];
    return msg.serialize();
}

String SP_MessageProcessor::createSpecsMessage(const PaddleSpecs& specs) {
    SP_Data msg(SP_Protocol::DataTypes::SPECS);
    msg.value[SP_Protocol::DataTypes::Specs::PADDLE_ID] = specs.paddleID;
    msg.value[SP_Protocol::DataTypes::Specs::PADDLE_TYPE] = specs.paddleType;
    msg.value[SP_Protocol::DataTypes::Specs::PADDLE_MODEL] = specs.paddleModel;
    msg.value[SP_Protocol::DataTypes::Specs::LENGTH] = specs.length;
    msg.value[SP_Protocol::DataTypes::Specs::IMU_DISTANCE] = specs.imuDistance;
    msg.value[SP_Protocol::DataTypes::Specs::IMU_FREQUENCY] = specs.imuFrequency;
    msg.value[SP_Protocol::DataTypes::Specs::HAS_LEFT_BLADE] = specs.hasLeftBlade;
    msg.value[SP_Protocol::DataTypes::Specs::HAS_RIGHT_BLADE] = specs.hasRightBlade;
    msg.value[SP_Protocol::DataTypes::Specs::FIRMWARE_VERSION] = specs.firmwareVersion;
    msg.value[SP_Protocol::DataTypes::Specs::AXIS_DIRECTION] = (int)specs.axisDirection;
    msg.value[SP_Protocol::DataTypes::Specs::BLADE_WEIGHT] = specs.bladeWeight;
    msg.value[SP_Protocol::DataTypes::Specs::BLADE_CENTER] = specs.bladeCenter;
    msg.value[SP_Protocol::DataTypes::Specs::BLADE_MOMENT_INERTIA] = specs.bladeMomentInertia;
    return msg.serialize();
}

String SP_MessageProcessor::createStatusMessage(const PaddleStatus& status) {
    SP_StatusMessage msg;
    msg.create();
    msg.status[SP_Protocol::DataTypes::StatusData::BATTERY] = status.batteryLevel;
    msg.status[SP_Protocol::DataTypes::StatusData::TEMPERATURE] = status.temperature;
    return msg.serialize();
}

// ============================================
// СОЗДАНИЕ ОТВЕТОВ (удобные методы)
// ============================================

String SP_MessageProcessor::createSuccessResponse(const char* command, const char* message) {
    return createResponseMessage(command, true, message ? message : "Success");
}

String SP_MessageProcessor::createErrorResponse(const char* command, const char* message) {
    return createResponseMessage(command, false, message ? message : "Error");
}
