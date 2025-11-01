/**
 * @file SmartPaddle.h
 * @brief Базовый класс SmartPaddle
 */
#ifndef SmartPaddle_h
#define SmartPaddle_h

#ifndef SP_MAX_EVENT_HANDLERS
#define SP_MAX_EVENT_HANDLERS 2
#endif


#include "../Core/Interfaces/IIMUSensor.h"
#include "../Core/Interfaces/ILoadCell.h"
#include "../Core/Types.h"
#include "SP_EventHandler.h"


// Class for smart paddle unified interface
class SmartPaddle {

    protected:
        PaddleSpecs specs;
        SP_EventHandler* eventHandler[SP_MAX_EVENT_HANDLERS];
        int eventHandlerCount;
        BladeOrientation bladeOrientation;

    public:
        SmartPaddle():
            specs(),
            eventHandlerCount(0){
                specs.paddleID = "";
                bladeOrientation.rightBladeAngle = 0;
                bladeOrientation.leftBladeAngle = 0;
            }
        virtual IMUData getIMUData()=0;
        virtual loadData getLoadData()=0;
        virtual OrientationData getOrientationData()=0;
    
        virtual void calibrateIMU()=0;
        virtual void calibrateLoads(BladeSideType blade_side)=0;
        virtual void calibrateBladeAngle(BladeSideType blade_side)=0;
        
        virtual BladeOrientation getBladeAngles() {return bladeOrientation;}
        virtual PaddleSpecs getSpecs() {return specs;}
        virtual void setSpecs(const PaddleSpecs& specs, bool save = true) {this->specs = specs;}
        
        virtual ~SmartPaddle() {for (int i = 0; i < eventHandlerCount; i++) {delete eventHandler[i]; eventHandler[i] = nullptr;}}
        
        int setEventHandler(SP_EventHandler* handler) 
        {
            if (eventHandlerCount>=SP_MAX_EVENT_HANDLERS) return -1;
            eventHandler[eventHandlerCount++] = handler;
            return eventHandlerCount;
        }
        int removeEventHandler(int index) 
        {
            if (index<=0 || index>eventHandlerCount) return -1;
            for (int i = index; i < eventHandlerCount-1; i++) {
                eventHandler[i] = eventHandler[i+1];
            }
            eventHandler[eventHandlerCount-1] = nullptr;
            eventHandlerCount--;
            return eventHandlerCount;
        }
        virtual bool operating()=0;
        virtual uint32_t paddleMillis()=0;
 
};

#endif

