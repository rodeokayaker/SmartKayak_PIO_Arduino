#ifndef STROKEPREDICTOR_H
#define STROKEPREDICTOR_H

#include "Arduino.h"
#include "SP_Types.h"

class StrokePredictor {
    public:
        StrokePredictor();
        virtual ~StrokePredictor();
        virtual float PredictForce(float shaft_z, float blade_z, BladeSideType bladeSide, 
                                  float current_force, float& coefPredict);
};

#endif