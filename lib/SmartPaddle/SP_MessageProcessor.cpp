#include "SP_MessageProcessor.h"
#include <ArduinoJson.h>

SP_MessageProcessor::SP_MessageProcessor(SP_MessageHandler* msgHandler) : handler(msgHandler) {}

void SP_MessageProcessor::setHandler(SP_MessageHandler* msgHandler) {
    handler = msgHandler;
}

void SP_MessageProcessor::processJson(const char* jsonStr) {
    if (!handler) return;

    Serial.printf("processJsonMessage: %s\n", jsonStr);

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonStr);
    if (error) return;

    const char* typeStr = doc["type"];
    if (!typeStr) return;
    SP_Message* msg = nullptr;

    if (strcmp(typeStr, SP_Protocol::MessageType::LOG) == 0) {
        msg = new SP_LogMessage(doc);
        handler->onLog((SP_LogMessage*)msg);
    } else if (strcmp(typeStr, SP_Protocol::MessageType::COMMAND) == 0) {
        msg = new SP_Command(doc);
        processCommand((SP_Command*)msg);
    } else if (strcmp(typeStr, SP_Protocol::MessageType::RESPONSE) == 0) {
        msg = new SP_Response(doc);
        processResponse((SP_Response*)msg);
    } else if (strcmp(typeStr, SP_Protocol::MessageType::DATA) == 0) {
        msg = new SP_Data(doc);
        processData((SP_Data*)msg);
    } else if (strcmp(typeStr, SP_Protocol::MessageType::STATUS) == 0) {
        msg = new SP_StatusMessage(doc);
        processStatus((SP_StatusMessage*)msg);
    } else {
        msg = new SP_Message(doc);
        handler->onMessage(msg);
    }
    
    delete msg;
}

void SP_MessageProcessor::processCommand(SP_Command* command) {
    if (!handler || !command) return;
    
    // Сначала вызываем общий обработчик команд
//    handler->onCommand(command);
    
    // Затем обрабатываем конкретные команды
    if (strcmp(command->command.c_str(), SP_Protocol::Commands::CALIBRATE_IMU) == 0) {
        handler->onCalibrateIMUCommand(command);
    }
    else if (strcmp(command->command.c_str(), SP_Protocol::Commands::CALIBRATE_LOADS) == 0) {
        BladeSideType bladeSide = (BladeSideType)getParam<int>(command->params, SP_Protocol::Commands::Params::BLADE_SIDE, (int)ALL_BLADES);
        handler->onCalibrateLoadsCommand(command, bladeSide);
    }
    else if (strcmp(command->command.c_str(), SP_Protocol::Commands::CALIBRATE_BLADE_ANGLE) == 0) {
        BladeSideType bladeSide = (BladeSideType)getParam<int>(command->params, SP_Protocol::Commands::Params::BLADE_SIDE, (int)ALL_BLADES);
        handler->onCalibrateBladeAngleCommand(command, bladeSide);
    }
    else if (strcmp(command->command.c_str(), SP_Protocol::Commands::SEND_SPECS) == 0) {
        handler->onSendSpecsCommand(command);
    }
    else if (strcmp(command->command.c_str(), SP_Protocol::Commands::START_PAIR) == 0) {
        handler->onStartPairCommand(command);
    }
    else if (strcmp(command->command.c_str(), SP_Protocol::Commands::SHUTDOWN) == 0) {
        handler->onShutdownCommand(command);
    }
    else if (strcmp(command->command.c_str(), SP_Protocol::Commands::TARE_LOADS) == 0) {
        BladeSideType bladeSide = (BladeSideType)getParam<int>(command->params, SP_Protocol::Commands::Params::BLADE_SIDE, (int)ALL_BLADES);
        handler->onTareLoadsCommand(command, bladeSide);
    }
    else if (strcmp(command->command.c_str(), SP_Protocol::Commands::CALIBRATE_COMPASS) == 0) {
        handler->onCalibrateCompassCommand(command);
    }
    else if (strcmp(command->command.c_str(), SP_Protocol::Commands::SET_MAGNETOMETER_CALIBRATION) == 0) {
        float offset[3];
        float softIron[9];
        offset[0] = getParam<float>(command->params, SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_X, 0.0f);
        offset[1] = getParam<float>(command->params, SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_Y, 0.0f);
        offset[2] = getParam<float>(command->params, SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_Z, 0.0f);
        softIron[0] = getParam<float>(command->params, SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_0_0, 1.0f);
        softIron[1] = getParam<float>(command->params, SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_0_1, 0.0f);
        softIron[2] = getParam<float>(command->params, SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_0_2, 0.0f);
        softIron[3] = getParam<float>(command->params, SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_1_0, 0.0f);
        softIron[4] = getParam<float>(command->params, SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_1_1, 1.0f);
        softIron[5] = getParam<float>(command->params, SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_1_2, 0.0f);
        softIron[6] = getParam<float>(command->params, SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_2_0, 0.0f);
        softIron[7] = getParam<float>(command->params, SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_2_1, 0.0f);
        softIron[8] = getParam<float>(command->params, SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_2_2, 1.0f);
        handler->onSetMagnetometerCalibrationCommand(command, offset, softIron);
    }
    else if (strcmp(command->command.c_str(), SP_Protocol::Commands::SET_MAGNETOMETER_OFFSET) == 0) {
        float offset[3];
        offset[0] = getParam<float>(command->params, SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_X, 0.0f);
        offset[1] = getParam<float>(command->params, SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_Y, 0.0f);
        offset[2] = getParam<float>(command->params, SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_Z, 0.0f);
        handler->onSetMagnetometerOffsetCommand(command, offset);
    } else {
        handler->onCommand(command);
    }
}

void SP_MessageProcessor::processResponse(SP_Response* response) {
    if (!handler || !response) return;
    
    // Вызываем обработчик ответов
    handler->onResponse(response);
}

void SP_MessageProcessor::processData(SP_Data* data) {
    if (!handler || !data) return;
    
    // Сначала вызываем общий обработчик данных
    handler->onData(data);
    
    // Затем обрабатываем конкретные типы данных
    if (strcmp(data->dataType.c_str(), SP_Protocol::DataTypes::IMU) == 0) {
        IMUData imuData;
        // Извлечение данных из JSON
        imuData.ax = getParam<float>(data->value, SP_Protocol::DataTypes::IMUData::ACCEL_X, 0.0f);
        imuData.ay = getParam<float>(data->value, SP_Protocol::DataTypes::IMUData::ACCEL_Y, 0.0f);
        imuData.az = getParam<float>(data->value, SP_Protocol::DataTypes::IMUData::ACCEL_Z, 0.0f);
        imuData.gx = getParam<float>(data->value, SP_Protocol::DataTypes::IMUData::GYRO_X, 0.0f);
        imuData.gy = getParam<float>(data->value, SP_Protocol::DataTypes::IMUData::GYRO_Y, 0.0f);
        imuData.gz = getParam<float>(data->value, SP_Protocol::DataTypes::IMUData::GYRO_Z, 0.0f);
        imuData.mx = getParam<float>(data->value, SP_Protocol::DataTypes::IMUData::MAG_X, 0.0f);
        imuData.my = getParam<float>(data->value, SP_Protocol::DataTypes::IMUData::MAG_Y, 0.0f);
        imuData.mz = getParam<float>(data->value, SP_Protocol::DataTypes::IMUData::MAG_Z, 0.0f);
        imuData.q0 = getParam<float>(data->value, SP_Protocol::DataTypes::IMUData::QUAT_0, 0.0f);
        imuData.q1 = getParam<float>(data->value, SP_Protocol::DataTypes::IMUData::QUAT_1, 0.0f);
        imuData.q2 = getParam<float>(data->value, SP_Protocol::DataTypes::IMUData::QUAT_2, 0.0f);
        imuData.q3 = getParam<float>(data->value, SP_Protocol::DataTypes::IMUData::QUAT_3, 0.0f);
        imuData.mag_x = getParam<float>(data->value, SP_Protocol::DataTypes::IMUData::MAG_X_ALT, 0.0f);
        imuData.mag_y = getParam<float>(data->value, SP_Protocol::DataTypes::IMUData::MAG_Y_ALT, 0.0f);
        imuData.mag_z = getParam<float>(data->value, SP_Protocol::DataTypes::IMUData::MAG_Z_ALT, 0.0f);
        imuData.timestamp = getParam<int>(data->value, SP_Protocol::DataTypes::IMUData::TIMESTAMP, 0);
        
        handler->onIMUData(data, imuData);
    }
    else if (strcmp(data->dataType.c_str(), SP_Protocol::DataTypes::LOAD) == 0) {
        loadData loadData;
        loadData.forceL = getParam<float>(data->value, SP_Protocol::DataTypes::LoadData::LEFT, 0.0f);
        loadData.forceR = getParam<float>(data->value, SP_Protocol::DataTypes::LoadData::RIGHT, 0.0f);
        loadData.forceL_raw = getParam<int>(data->value, SP_Protocol::DataTypes::LoadData::LEFT_RAW, 0);
        loadData.forceR_raw = getParam<int>(data->value, SP_Protocol::DataTypes::LoadData::RIGHT_RAW, 0);
        loadData.timestamp = getParam<int>(data->value, SP_Protocol::DataTypes::LoadData::TIMESTAMP, 0);
        
        handler->onLoadData(data, loadData);
    }
    else if (strcmp(data->dataType.c_str(), SP_Protocol::DataTypes::ORIENTATION) == 0) {
        OrientationData orientationData;
        orientationData.q0 = getParam<float>(data->value, SP_Protocol::DataTypes::OrientationData::QUAT_0, 0.0f);
        orientationData.q1 = getParam<float>(data->value, SP_Protocol::DataTypes::OrientationData::QUAT_1, 0.0f);
        orientationData.q2 = getParam<float>(data->value, SP_Protocol::DataTypes::OrientationData::QUAT_2, 0.0f);
        orientationData.q3 = getParam<float>(data->value, SP_Protocol::DataTypes::OrientationData::QUAT_3, 0.0f);
        orientationData.timestamp = getParam<int>(data->value, SP_Protocol::DataTypes::OrientationData::TIMESTAMP, 0);
        
        handler->onOrientationData(data, orientationData);
    }
    else if (strcmp(data->dataType.c_str(), SP_Protocol::DataTypes::BLADE_ORIENTATION) == 0) {
        BladeOrientation bladeOrientation;
        bladeOrientation.YAxisDirection = getParam<int>(data->value, SP_Protocol::DataTypes::BladeOrientation::Y_AXIS_DIRECTION, 0);
        bladeOrientation.rightBladeAngle = getParam<float>(data->value, SP_Protocol::DataTypes::BladeOrientation::RIGHT_BLADE_ANGLE, 0.0f);
        bladeOrientation.leftBladeAngle = getParam<float>(data->value, SP_Protocol::DataTypes::BladeOrientation::LEFT_BLADE_ANGLE, 0.0f);
        bladeOrientation.rightBladeVector[0] = getParam<float>(data->value, SP_Protocol::DataTypes::BladeOrientation::RIGHT_BLADE_VECTOR_X, 0.0f);
        bladeOrientation.rightBladeVector[1] = getParam<float>(data->value, SP_Protocol::DataTypes::BladeOrientation::RIGHT_BLADE_VECTOR_Y, 0.0f);
        bladeOrientation.rightBladeVector[2] = getParam<float>(data->value, SP_Protocol::DataTypes::BladeOrientation::RIGHT_BLADE_VECTOR_Z, 0.0f);
        bladeOrientation.leftBladeVector[0] = getParam<float>(data->value, SP_Protocol::DataTypes::BladeOrientation::LEFT_BLADE_VECTOR_X, 0.0f);
        bladeOrientation.leftBladeVector[1] = getParam<float>(data->value, SP_Protocol::DataTypes::BladeOrientation::LEFT_BLADE_VECTOR_Y, 0.0f);
        bladeOrientation.leftBladeVector[2] = getParam<float>(data->value, SP_Protocol::DataTypes::BladeOrientation::LEFT_BLADE_VECTOR_Z, 0.0f);
        
        handler->onBladeOrientationData(data, bladeOrientation);
    }
    else if (strcmp(data->dataType.c_str(), SP_Protocol::DataTypes::SPECS) == 0) {
        PaddleSpecs specs;
        specs.paddleID = getParam<String>(data->value, SP_Protocol::DataTypes::Specs::PADDLE_ID, "");
        specs.paddleType = (PaddleType)getParam<int>(data->value, SP_Protocol::DataTypes::Specs::PADDLE_TYPE, 0);
        specs.paddleModel = getParam<String>(data->value, SP_Protocol::DataTypes::Specs::PADDLE_MODEL, "");
        specs.length = getParam<int>(data->value, SP_Protocol::DataTypes::Specs::LENGTH, 0);
        specs.imuFrequency = getParam<int>(data->value, SP_Protocol::DataTypes::Specs::IMU_FREQUENCY, 0);
        specs.hasLeftBlade = getParam<bool>(data->value, SP_Protocol::DataTypes::Specs::HAS_LEFT_BLADE, false);
        specs.hasRightBlade = getParam<bool>(data->value, SP_Protocol::DataTypes::Specs::HAS_RIGHT_BLADE, false);
        specs.firmwareVersion = getParam<int>(data->value, SP_Protocol::DataTypes::Specs::FIRMWARE_VERSION, 0);
        specs.imuDistance = getParam<float>(data->value, SP_Protocol::DataTypes::Specs::IMU_DISTANCE, 0.0f);
        handler->onSpecsData(data, specs);
    } else if (strcmp(data->dataType.c_str(), SP_Protocol::DataTypes::MAGNETOMETER_CALIBRATION) == 0) {
        float offset[3];
        float softIron[9];
        offset[0] = getParam<float>(data->value, SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_X, 0.0f);
        offset[1] = getParam<float>(data->value, SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_Y, 0.0f);
        offset[2] = getParam<float>(data->value, SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_Z, 0.0f);
        softIron[0] = getParam<float>(data->value, SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_0_0, 1.0f);
        softIron[1] = getParam<float>(data->value, SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_0_1, 0.0f);
        softIron[2] = getParam<float>(data->value, SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_0_2, 0.0f);
        softIron[3] = getParam<float>(data->value, SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_1_0, 0.0f);
        softIron[4] = getParam<float>(data->value, SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_1_1, 1.0f);
        softIron[5] = getParam<float>(data->value, SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_1_2, 0.0f);
        softIron[6] = getParam<float>(data->value, SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_2_0, 0.0f);
        softIron[7] = getParam<float>(data->value, SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_2_1, 0.0f);
        softIron[8] = getParam<float>(data->value, SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_2_2, 1.0f);
        handler->onMagnetometerCalibrationData(data, offset, softIron);
    }
    else if (strcmp(data->dataType.c_str(), SP_Protocol::DataTypes::MAGNETOMETER_CALIBRATION_STATUS) == 0) {
        int status = getParam<int>(data->value, SP_Protocol::DataTypes::MagnetometerCalibration::STATUS, 0);
        handler->onMagnetometerCalibrationStatusData(data, status);
    }
    else if (strcmp(data->dataType.c_str(), SP_Protocol::DataTypes::MAGNETOMETER_OFFSET) == 0) {
        float offset[3];
        offset[0] = getParam<float>(data->value, SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_X, 0.0f);
        offset[1] = getParam<float>(data->value, SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_Y, 0.0f);
        offset[2] = getParam<float>(data->value, SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_Z, 0.0f);
        handler->onMagnetometerOffsetData(data, offset);
    }
}

void SP_MessageProcessor::processStatus(SP_StatusMessage* status) {
    if (!handler || !status) return;
    
    // Вызываем обработчик статуса
    handler->onStatus(status);
}

// Базовые методы создания сообщений
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

// Команды
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

// Данные датчиков
String SP_MessageProcessor::createIMUDataMessage(const IMUData& data) {
    SP_Data msg(SP_Protocol::DataTypes::IMU);
    msg.value[SP_Protocol::DataTypes::IMUData::ACCEL_X] = data.ax;
    msg.value[SP_Protocol::DataTypes::IMUData::ACCEL_Y] = data.ay;
    msg.value[SP_Protocol::DataTypes::IMUData::ACCEL_Z] = data.az;
    msg.value[SP_Protocol::DataTypes::IMUData::GYRO_X] = data.gx;
    msg.value[SP_Protocol::DataTypes::IMUData::GYRO_Y] = data.gy;
    msg.value[SP_Protocol::DataTypes::IMUData::GYRO_Z] = data.gz;
    msg.value[SP_Protocol::DataTypes::IMUData::MAG_X] = data.mx;
    msg.value[SP_Protocol::DataTypes::IMUData::MAG_Y] = data.my;
    msg.value[SP_Protocol::DataTypes::IMUData::MAG_Z] = data.mz;
    msg.value[SP_Protocol::DataTypes::IMUData::QUAT_0] = data.q0;
    msg.value[SP_Protocol::DataTypes::IMUData::QUAT_1] = data.q1;
    msg.value[SP_Protocol::DataTypes::IMUData::QUAT_2] = data.q2;
    msg.value[SP_Protocol::DataTypes::IMUData::QUAT_3] = data.q3;
    msg.value[SP_Protocol::DataTypes::IMUData::MAG_X_ALT] = data.mag_x;
    msg.value[SP_Protocol::DataTypes::IMUData::MAG_Y_ALT] = data.mag_y;
    msg.value[SP_Protocol::DataTypes::IMUData::MAG_Z_ALT] = data.mag_z;
    msg.value[SP_Protocol::DataTypes::IMUData::TIMESTAMP] = data.timestamp;
    return msg.serialize();
}

String SP_MessageProcessor::createLoadDataMessage(const loadData& data) {
    SP_Data msg(SP_Protocol::DataTypes::LOAD);
    msg.value[SP_Protocol::DataTypes::LoadData::LEFT] = data.forceL;
    msg.value[SP_Protocol::DataTypes::LoadData::RIGHT] = data.forceR;
    msg.value[SP_Protocol::DataTypes::LoadData::LEFT_RAW] = data.forceL_raw;
    msg.value[SP_Protocol::DataTypes::LoadData::RIGHT_RAW] = data.forceR_raw;
    msg.value[SP_Protocol::DataTypes::LoadData::TIMESTAMP] = data.timestamp;
    return msg.serialize();
}

String SP_MessageProcessor::createOrientationDataMessage(const OrientationData& data) {
    SP_Data msg(SP_Protocol::DataTypes::ORIENTATION);
    msg.value[SP_Protocol::DataTypes::OrientationData::QUAT_0] = data.q0;
    msg.value[SP_Protocol::DataTypes::OrientationData::QUAT_1] = data.q1;
    msg.value[SP_Protocol::DataTypes::OrientationData::QUAT_2] = data.q2;
    msg.value[SP_Protocol::DataTypes::OrientationData::QUAT_3] = data.q3;
    msg.value[SP_Protocol::DataTypes::OrientationData::TIMESTAMP] = data.timestamp;
    return msg.serialize();
}

String SP_MessageProcessor::createBladeOrientationMessage(const BladeOrientation& orientation) {
    SP_Data msg(SP_Protocol::DataTypes::BLADE_ORIENTATION);
    msg.value[SP_Protocol::DataTypes::BladeOrientation::Y_AXIS_DIRECTION] = orientation.YAxisDirection;
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
    return msg.serialize();
}

String SP_MessageProcessor::createStatusMessage(const PaddleStatus& status) {
    SP_StatusMessage msg;
    msg.create();
    msg.status[SP_Protocol::DataTypes::StatusData::BATTERY] = status.batteryLevel;
    msg.status[SP_Protocol::DataTypes::StatusData::TEMPERATURE] = status.temperature;
    return msg.serialize();
}

// Ответы
String SP_MessageProcessor::createSuccessResponse(const char* command, const char* message) {
    return createResponseMessage(command, true, message ? message : "Success");
}

String SP_MessageProcessor::createErrorResponse(const char* command, const char* message) {
    return createResponseMessage(command, false, message ? message : "Error");
} 