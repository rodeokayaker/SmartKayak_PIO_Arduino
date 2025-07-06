#if 0 

#include "StrokePredictorOne.h"

StrokePredictorOne::StrokePredictorOne() {
    // Инициализация
}

StrokePredictorOne::~StrokePredictorOne() {
    // Очистка
}

float StrokePredictorOne::PredictForce(float shaftRotationAngle,
         float shaftTiltAngle,
         float bladeRotationAngle, 
         BladeSideType bladeSide, 
         float CalculatedForce, 
         float& coefPredict, 
         float& coefCalculate) 
{

    //Запоминаем данные из датчиков
    if (millis() >= nextTimeStroke) {
        paddleBuffer.buffer_index++;
        if (paddleBuffer.buffer_index >= 100) {
            paddleBuffer.buffer_index = 0;
        }
        paddleBuffer.pitch_buffer[paddleBuffer.buffer_index] = pitch;
        paddleBuffer.strain_buffer[paddleBuffer.buffer_index] = strain;
        if (paddleBuffer.buffer_index == 0) {
            currentStroke.is_active = true;
            currentStroke.start_time = millis();
        }
        nextTimeStroke = nextTimeStroke + PATTERN_DIMENSION;
    }

    if (currentStrokeIndex == -1 || currentStroke[currentStrokeIndex].is_active == false) {
        //NO ACTIVE STROKE
    }





}
#endif