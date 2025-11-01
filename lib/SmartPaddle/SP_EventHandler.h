/**
 * @file SP_EventHandler.h
 * @brief Обработчик событий SmartPaddle
 */
#pragma once
#include "SP_Types.h"
#include "../Core/Interfaces/IIMUSensor.h"
#include "../Core/Interfaces/ILoadCell.h"

class SmartPaddle;
class BLEAddress;

class SP_EventHandler {
public:
    virtual ~SP_EventHandler() = default;
    virtual void onUpdateIMU(IMUData& imuData, SmartPaddle* paddle) {}
    virtual void onUpdateOrientation(OrientationData& orientationData, SmartPaddle* paddle) {}
    virtual void onUpdateLoad(loadData& loadData, SmartPaddle* paddle) {}
    virtual void onUpdateBlade(BladeData& bladeData, SmartPaddle* paddle) {}
    virtual void onUpdateSpecs(PaddleSpecs& specsData, SmartPaddle* paddle) {}
    virtual void onUpdateStatus(PaddleStatus& statusData, SmartPaddle* paddle) {}
    virtual void onUpdateBladeAngle(BladeOrientation& bladeOrientation, SmartPaddle* paddle) {}
    virtual void onConnect(SmartPaddle* paddle) {}
    virtual void onDisconnect(SmartPaddle* paddle) {}
    virtual void onPairing(SmartPaddle* paddle) {}
    virtual void onPairingDone(SmartPaddle* paddle, BLEAddress* address) {}
    virtual void onUnpair(SmartPaddle* paddle) {}
    virtual void onShutdown(SmartPaddle* paddle) {}
    virtual void onError(SmartPaddle* paddle) {}
    virtual void onUpdate(SmartPaddle* paddle) {}
}; 