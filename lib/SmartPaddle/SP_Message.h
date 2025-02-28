#pragma once
#include <ArduinoJson.h>

class SP_Message {
protected:
    JsonDocument document;
public:
    SP_Message(JsonDocument doc) : document(doc) {}
    SP_Message() : document() {}
    virtual JsonDocument& serializeDocument() {return document;};
    virtual String serialize() {
        String result; 
        serializeJson(document, result); 
        return result;
    }
    virtual bool deserialize() {return true;};
    virtual bool deserialize(const JsonDocument& doc) {document = doc; return deserialize();}
    virtual bool deserialize(const char* str) {return ((deserializeJson(document, str)==DeserializationError::Ok) && deserialize());}
    virtual void clear() {document.clear();}
    virtual ~SP_Message() = default;
};

class SP_Command : public SP_Message {

public:
    String command;
    JsonObject params;

    SP_Command(const String& cmd = "", JsonObject* p = nullptr) : 
        command(cmd){

        if (cmd!=""){
            create(cmd.c_str());
        }
        if (p){
            params=*p;
        }
    }
    SP_Command(JsonDocument doc) : SP_Message(doc) {deserialize();}
    JsonDocument& serializeDocument() override;
    void create(const char* cmd);
    bool deserialize() override;
};

class SP_Response : public SP_Message {

public:
    String command;
    bool success;
    String message;

    SP_Response(const String& cmd = "", bool s = false, const String& msg = "") :
        command(cmd), success(s), message(msg) 
    {
        if (cmd!=""){
            create(cmd.c_str(), s, msg.c_str());
        }
    }
    SP_Response(JsonDocument doc) : SP_Message(doc) {deserialize();}

    JsonDocument& serializeDocument() override;
    bool deserialize() override;
    void create(const char* cmd, bool success, const char* msg);
};

class SP_Data : public SP_Message {

public:
    String dataType;
    JsonObject value;

    SP_Data(const String& type = "", JsonObject* val = nullptr) :
        dataType(type){
        if (type!=""){
            create(type.c_str());
        }
        if (val){
            value=*val;
        }
    }
    SP_Data(JsonDocument doc) : SP_Message(doc) {deserialize();}

    JsonDocument& serializeDocument() override;
    bool deserialize() override;
    void create(const char* dataType);
};

class SP_LogMessage : public SP_Message {

public:
    String message;

    SP_LogMessage(const String& msg = "") : message(msg) {
        if (msg!=""){
            create(msg.c_str());
        }
    }
    SP_LogMessage(JsonDocument doc) : SP_Message(doc) {deserialize();}

    JsonDocument& serializeDocument() override;
    bool deserialize() override;
    void create(const char* msg);
};

class SP_StatusMessage : public SP_Message {

public:
    JsonObject status;

    SP_StatusMessage(JsonObject* stat = nullptr) {
        if (stat){
            create();
            status=*stat;
        }
    }
    SP_StatusMessage(JsonDocument doc) : SP_Message(doc) {deserialize();}

    JsonDocument& serializeDocument() override;
    bool deserialize() override;
    void create();
}; 