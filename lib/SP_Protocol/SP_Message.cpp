#include "SP_Message.h"
#include "SP_Protocol.h"
#include <ArduinoJson.h>

JsonDocument& SP_Command::serializeDocument() {
    document->clear();
    (*document)[SP_Protocol::MessageType::TITLE] = SP_Protocol::MessageType::COMMAND;
    (*document)[SP_Protocol::Commands::TITLE] = command;
    if (params.size() > 0) {
        (*document)[SP_Protocol::Commands::Params::TITLE].set(params);
    }
    return *document;
}

bool SP_Command::deserialize() {
    command = (*document)[SP_Protocol::Commands::TITLE].as<String>();
    if ((*document)[SP_Protocol::Commands::Params::TITLE].is<JsonObject>()) {
        params = (*document)[SP_Protocol::Commands::Params::TITLE];
    }
    return true;
}

void SP_Command::create(const char* cmd) {
    if (!cmd) return;  // Защита от null-указателя
    
    command = cmd;
    document->clear();
    (*document)[SP_Protocol::MessageType::TITLE] = SP_Protocol::MessageType::COMMAND;
    (*document)[SP_Protocol::Commands::TITLE] = cmd;
    params = (*document)[SP_Protocol::Commands::Params::TITLE].to<JsonObject>();
}

JsonDocument& SP_Response::serializeDocument() {
    document->clear();
    (*document)[SP_Protocol::MessageType::TITLE] = SP_Protocol::MessageType::RESPONSE;
    (*document)[SP_Protocol::Commands::TITLE] = command;
    (*document)[SP_Protocol::Commands::Responses::TITLE] = success;
    (*document)[SP_Protocol::Commands::Responses::MESSAGE] = message;
    return *document;
}

bool SP_Response::deserialize() {

    
    command = (*document)[SP_Protocol::Commands::TITLE].as<String>();
    success = (*document)[SP_Protocol::Commands::Responses::TITLE].as<bool>();
    
    // Проверка наличия поля "msg", но делаем его опциональным
    if ((*document)[SP_Protocol::Commands::Responses::MESSAGE].is<String>()) {
        message = (*document)[SP_Protocol::Commands::Responses::MESSAGE].as<String>();
    } else {
        message = "";  // Пустое сообщение по умолчанию
    }
    
    return true;
}

void SP_Response::create(const char* cmd, bool success, const char* msg) {
    if (!cmd) return;  // Защита от null-указателя
    
    command = cmd;
    this->success = success;
    message = msg ? msg : "";  // Защита от null-указателя
    
    document->clear();
    (*document)[SP_Protocol::MessageType::TITLE] = SP_Protocol::MessageType::RESPONSE;
    (*document)[SP_Protocol::Commands::TITLE] = cmd;
    (*document)[SP_Protocol::Commands::Responses::TITLE] = success;
    (*document)[SP_Protocol::Commands::Responses::MESSAGE] = message;
}

JsonDocument& SP_Data::serializeDocument() {
    document->clear();
    (*document)[SP_Protocol::MessageType::TITLE] = SP_Protocol::MessageType::DATA;
    (*document)[SP_Protocol::DataTypes::TITLE] = dataType;
    (*document)[SP_Protocol::DataTypes::VALUE].set(value);
    return *document;
}

bool SP_Data::deserialize() {
    dataType = (*document)[SP_Protocol::DataTypes::TITLE].as<String>();
    value = (*document)[SP_Protocol::DataTypes::VALUE].as<JsonObject>();
    return true;
}

void SP_Data::create(const char* dataType) {
    if (!dataType) return;  // Защита от null-указателя
    
    this->dataType = dataType;
    document->clear();
    (*document)[SP_Protocol::MessageType::TITLE] = SP_Protocol::MessageType::DATA;
    (*document)[SP_Protocol::DataTypes::TITLE] = dataType;
    value = (*document)[SP_Protocol::DataTypes::VALUE].to<JsonObject>();
}

JsonDocument& SP_LogMessage::serializeDocument() {
    document->clear();
    (*document)[SP_Protocol::MessageType::TITLE] = SP_Protocol::MessageType::LOG;
    (*document)[SP_Protocol::LogMessages::MESSAGE] = message;
    return *document;
}

void SP_LogMessage::create(const char* msg) {
    if (!msg) return;  // Защита от null-указателя
    
    message = msg;
    document->clear();
    (*document)[SP_Protocol::MessageType::TITLE] = SP_Protocol::MessageType::LOG;
    (*document)[SP_Protocol::LogMessages::MESSAGE] = msg;
}

bool SP_LogMessage::deserialize() {
    if (!(*document)[SP_Protocol::LogMessages::MESSAGE].is<String>()) {
        return false;
    }
    
    message = (*document)[SP_Protocol::LogMessages::MESSAGE].as<String>();
    return true;  // Добавлен возврат значения
}

JsonDocument& SP_StatusMessage::serializeDocument() {
    document->clear();
    (*document)[SP_Protocol::MessageType::TITLE] = SP_Protocol::MessageType::STATUS;
    if (status.size() > 0) {
        (*document)[SP_Protocol::Status::TITLE].set(status);
    }
    return *document;
}

bool SP_StatusMessage::deserialize() {
    if (!(*document)[SP_Protocol::Status::TITLE].is<JsonObject>()) {
        return false;
    }
    
    status = (*document)[SP_Protocol::Status::TITLE].as<JsonObject>();
    return true;
}

void SP_StatusMessage::create() {
    document->clear();
    (*document)[SP_Protocol::MessageType::TITLE] = SP_Protocol::MessageType::STATUS;
    status = (*document)[SP_Protocol::Status::TITLE].to<JsonObject>();
}