#include "SP_Message.h"
#include "SP_Protocol.h"
#include <ArduinoJson.h>

JsonDocument& SP_Command::serializeDocument() {
    document.clear();
    document["type"] = SP_Protocol::MessageType::COMMAND;
    document["data"]["cmd"] = command;
    if (params.size() > 0) {
        document["data"]["params"].set(params);
    }
    return document;
}

bool SP_Command::deserialize() {
    if (!document["data"].is<JsonObject>()) {
        return false;
    }
    if (!document["data"]["cmd"].is<String>()) {
        return false;
    }
    command = document["data"]["cmd"].as<String>();
    if (document["data"]["params"].is<JsonObject>()) {
        params = document["data"]["params"];
    }
    return true;
}

void SP_Command::create(const char* cmd) {
    if (!cmd) return;  // Защита от null-указателя
    
    command = cmd;
    document.clear();
    document["type"] = SP_Protocol::MessageType::COMMAND;
    document["data"]["cmd"] = cmd;
    params = document["data"]["params"].to<JsonObject>();
}

JsonDocument& SP_Response::serializeDocument() {
    document.clear();
    document["type"] = SP_Protocol::MessageType::RESPONSE;
    document["data"]["cmd"] = command;
    document["data"]["success"] = success;
    document["data"]["msg"] = message;
    return document;
}

bool SP_Response::deserialize() {
    if (!document["data"].is<JsonObject>()) {
        return false;
    }
    if (!document["data"]["cmd"].is<String>()) {
        return false;
    }
    if (!document["data"]["success"].is<bool>()) {
        return false;
    }
    
    command = document["data"]["cmd"].as<String>();
    success = document["data"]["success"].as<bool>();
    
    // Проверка наличия поля "msg", но делаем его опциональным
    if (document["data"]["msg"].is<String>()) {
        message = document["data"]["msg"].as<String>();
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
    
    document.clear();
    document["type"] = SP_Protocol::MessageType::RESPONSE;
    document["data"]["cmd"] = cmd;
    document["data"]["success"] = success;
    document["data"]["msg"] = message;
}

JsonDocument& SP_Data::serializeDocument() {
    document.clear();
    document["type"] = SP_Protocol::MessageType::DATA;
    document["data"]["dataType"] = dataType;
    document["data"]["value"].set(value);
    return document;
}

bool SP_Data::deserialize() {
    if (!document["data"].is<JsonObject>()) {
        return false;
    }
    if (!document["data"]["dataType"].is<String>()) {
        return false;
    }
    if (!document["data"]["value"].is<JsonObject>()) {
        return false;
    }
    
    dataType = document["data"]["dataType"].as<String>();
    value = document["data"]["value"].to<JsonObject>();
    return true;
}

void SP_Data::create(const char* dataType) {
    if (!dataType) return;  // Защита от null-указателя
    
    this->dataType = dataType;
    document.clear();
    document["type"] = SP_Protocol::MessageType::DATA;
    document["data"]["dataType"] = dataType;
    value = document["data"]["value"].to<JsonObject>();
}

JsonDocument& SP_LogMessage::serializeDocument() {
    document.clear();
    document["type"] = SP_Protocol::MessageType::LOG;
    document["data"]["msg"] = message;
    return document;
}

void SP_LogMessage::create(const char* msg) {
    if (!msg) return;  // Защита от null-указателя
    
    message = msg;
    document.clear();
    document["type"] = SP_Protocol::MessageType::LOG;
    document["data"]["msg"] = msg;
}

bool SP_LogMessage::deserialize() {
    if (!document["data"].is<JsonObject>()) {
        return false;
    }
    if (!document["data"]["msg"].is<String>()) {
        return false;
    }
    
    message = document["data"]["msg"].as<String>();
    return true;  // Добавлен возврат значения
}

JsonDocument& SP_StatusMessage::serializeDocument() {
    document.clear();
    document["type"] = SP_Protocol::MessageType::STATUS;
    if (status.size() > 0) {
        document["data"].set(status);
    }
    return document;
}

bool SP_StatusMessage::deserialize() {
    if (!document["data"].is<JsonObject>()) {
        return false;
    }
    
    status = document["data"].to<JsonObject>();
    return true;
}

void SP_StatusMessage::create() {
    document.clear();
    document["type"] = SP_Protocol::MessageType::STATUS;
    status = document["data"].to<JsonObject>();
}