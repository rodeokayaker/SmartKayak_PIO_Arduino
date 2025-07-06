#ifndef STROKEPREDICTOR_H
#define STROKEPREDICTOR_H

#include "Arduino.h"
#include "SP_Types.h"

class StrokePredictor {
    public:
        StrokePredictor();
        virtual ~StrokePredictor();
        virtual float PredictForce (float shaftRotationAngle,
         float shaftTiltAngle,
         float bladeRotationAngle, 
         BladeSideType bladeSide, 
         float CalculatedForce, 
         float& coefPredict, 
         float& coefCalculate); 
#endif