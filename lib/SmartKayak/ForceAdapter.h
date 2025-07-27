#ifndef FORCEADAPTER_H
#define FORCEADAPTER_H
#include "Arduino.h"

class ForceAdapter {
private:
    static const uint32_t REVERSE_SWITCH_TIME = 400;
    static const uint32_t DIMMING_TIME = 1700;

    uint32_t lastAcceptForceTime;
    int currentForce;

    int dimmingForce;
    uint32_t dimmingFinishTime;

public:
    ForceAdapter():lastAcceptForceTime(millis()),currentForce(0),dimmingForce(0),dimmingFinishTime(0) {}

    int GetAdaptedForce(int force)
    {

        //DELAY FOR STROKE CHANGING DIRECTION

        if ((currentForce >0 && force<0) || (currentForce < 0 && force>0)) {
            // if force is changing direction
            if (millis() - lastAcceptForceTime < REVERSE_SWITCH_TIME) {
                force=0;
            } else {
                dimmingFinishTime = millis()+DIMMING_TIME;
                dimmingForce = force;
                currentForce = force;
                lastAcceptForceTime = millis();
                return force;
            }
        }

        if (force!=0) lastAcceptForceTime = millis();

        //DIMMING CURVE

        float calculatedDimmer = 0;
        if (dimmingFinishTime>millis())
        {
            //SQUARE APPROXIMATION
            float alpha = ((float)(dimmingFinishTime-millis())) / (float)(DIMMING_TIME);
            calculatedDimmer = alpha*alpha*dimmingForce;
        }

        if (force > 0){
            if (calculatedDimmer>force){
                currentForce = calculatedDimmer;
            } else {
                dimmingFinishTime=millis()+DIMMING_TIME;
                dimmingForce=force;
                currentForce=force;
            }
        } else {
            if (force < 0) {
                if (calculatedDimmer<force){
                    currentForce = calculatedDimmer;
                } else {
                    dimmingFinishTime=millis()+DIMMING_TIME;
                    dimmingForce=force;
                    currentForce=force;
                }
    
            } else {
                //force == 0;
                currentForce = calculatedDimmer;
            }

        }

        return currentForce;                
    }

};

#endif
