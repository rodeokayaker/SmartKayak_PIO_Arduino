#pragma once
#include "SP_Message.h"
#include "SP_Types.h"
#include "InterfaceIMU.h"
#include "InterfaceLoadCell.h"

class SP_MessageHandler {
public:
    // Базовые обработчики сообщений
    virtual void onMessage(SP_Message* message) {};
    virtual void onCommand(SP_Command* command) {};
    virtual void onResponse(SP_Response* response) {};
    virtual void onData(SP_Data* data) {};
    virtual void onLog(SP_LogMessage* log) {};
    virtual void onStatus(SP_StatusMessage* status) {};
    
    // Специализированные обработчики команд
    virtual void onCalibrateIMUCommand(SP_Command* command) {};
    virtual void onCalibrateLoadsCommand(SP_Command* command, BladeSideType bladeSide) {};
    virtual void onCalibrateBladeAngleCommand(SP_Command* command, BladeSideType bladeSide) {};
    virtual void onCalibrateCompassCommand(SP_Command* command) {};
    virtual void onSendSpecsCommand(SP_Command* command) {};
    virtual void onStartPairCommand(SP_Command* command) {};
    virtual void onShutdownCommand(SP_Command* command) {};
    virtual void onTareLoadsCommand(SP_Command* command, BladeSideType bladeSide) {};
    
    // Специализированные обработчики данных
    virtual void onIMUData(SP_Data* data, const IMUData& imuData) {};
    virtual void onLoadData(SP_Data* data, const loadData& loadData) {};
    virtual void onOrientationData(SP_Data* data, const OrientationData& orientationData) {};
    virtual void onBladeOrientationData(SP_Data* data, const BladeOrientation& bladeOrientation) {};
    virtual void onSpecsData(SP_Data* data, const PaddleSpecs& specs) {};
    
    virtual ~SP_MessageHandler() = default;
};