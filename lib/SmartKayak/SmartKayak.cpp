#include "SmartKayak.h"
#include "../Core/Interfaces/IIMUSensor.h"
#include "Arduino.h"
#include "SP_Quaternion.h"
#include "StrokePredictorGrid.h"
#include "../SmartKayak_Algorythms/OrientationUtils.h"
#include "Preferences.h"

#define POWER_LOW_SCALE 1.2f
#define POWER_MEDIUM_SCALE 2.6f
#define POWER_HIGH_SCALE 4.0f

#define IMU_STACK_SIZE 4096
#define MAGNETOMETER_STACK_SIZE 4096

SmartKayak* mainKayak = nullptr;

//---------------------------------TEMPORARY FIX---------------------------------

void FIXROTATIONVECTOR(float& q0, float& q1, float& q2, float& q3, uint8_t how = 0){
    if (how==0) return;
    
    // Сохраняем исходный кватернион
    float orig_q0 = q0, orig_q1 = q1, orig_q2 = q2, orig_q3 = q3;
    
    // Кватернион поворота вокруг Z: [w, 0, 0, z]
    float rot_w, rot_z;
    
    switch (how) {
        case 1: // Поворот на 90° против часовой стрелки
            rot_w = 0.70710678f;   // cos(π/4)
            rot_z = 0.70710678f;   // sin(π/4)
            break;
        case 2: // Поворот на 180°
            rot_w = 0.0f;          // cos(π/2)
            rot_z = 1.0f;          // sin(π/2)
            break;
        case 3: // Поворот на 270° против часовой стрелки
            rot_w = -0.70710678f;  // cos(3π/4)
            rot_z = 0.70710678f;   // sin(3π/4)
            break;
        default:
            return; // Неизвестный код поворота
    }
    
    // Умножение кватернионов: result = rotation * original
    // rotation = [rot_w, 0, 0, rot_z], original = [orig_q0, orig_q1, orig_q2, orig_q3]
    q0 = rot_w * orig_q0 - rot_z * orig_q3;
    q1 = rot_w * orig_q1 + rot_z * orig_q2;
    q2 = rot_w * orig_q2 - rot_z * orig_q1;
    q3 = rot_w * orig_q3 + rot_z * orig_q0;
} 



class SmartKayakRTOS{
    private:
    
    static void onIMUData(const IMUData& data) {
        return;
    }
    
    static void onOrientation(const OrientationData& data) {
    
    
        mainKayak->kayakOrientationQuat[0]=data.q0;
        mainKayak->kayakOrientationQuat[1]=data.q1;
        mainKayak->kayakOrientationQuat[2]=data.q2;
        mainKayak->kayakOrientationQuat[3]=data.q3;

        mainKayak->predictedPaddle->updateKayakOrientation(data);    
                
    }


    public:
    //All tasks setup and start
    static void startTasks(SmartKayak* kayak) {
        mainKayak = kayak;  // Используем глобальную переменную

        kayak->imu->onIMUData(SmartKayakRTOS::onIMUData);
        kayak->imu->onOrientation(SmartKayakRTOS::onOrientation);
        kayak->imu->startServices();
    }
};

SmartKayak::SmartKayak(String prefs_name):
prefsName(prefs_name),
paddle(nullptr),
motorDriver(nullptr),
modeSwitch(nullptr),
imu(nullptr),
display(nullptr),
currentBladeSide(BladeSideType::ALL_BLADES),
currentForceGramms(0),
predictorMode(0),
predictedTimeUsed(0),
predictedForceUsed(false),
usingPredictedForce(false),
predictedPaddle(nullptr)
{


}

void SmartKayak::begin() {
    loadSpecs();

//    imu->begin();
//    imu->setOrientationFrequency(100);

}


void SmartKayak::update() {
//  check if the mode is off or paddle is not connected
/* ****************************************** */
/* Uncomment this code if you want to use it */
/* ****************************************** */
/*
    if (modeSwitch->getMode() == MOTOR_OFF){
        if (motorDriver->getForce() != 0) {
            motorDriver->stop();
        }
        return; 
    }*/
   
    if (!paddle->operating()) {
        motorDriver->stop();
        return; 

    }   

    uint32_t timestamp = millis();

    int force = 0;
    static int borderLoadForce = 600;

    if (paddle->getOrientationData().q0 == 0) {
//        Serial.printf("Paddle orientation is not valid: %f, %f, %f, %f\n", paddleOrientation.q0, paddleOrientation.q1, paddleOrientation.q2, paddleOrientation.q3);
        return; 
    }
    if (kayakOrientationQuat[0] == 0) {
//        Serial.printf("Kayak orientation is not valid: %f, %f, %f, %f\n", kayakOrientationQuat[0], kayakOrientationQuat[1], kayakOrientationQuat[2], kayakOrientationQuat[3]);
        return; 
    }

    currentBladeSide = predictedPaddle->getBladeSide();
    if (currentBladeSide == BladeSideType::ALL_BLADES) {
        motorDriver->stop();
        return; 
    }
    
 
 
    float fForce = predictedPaddle->getAxialForce();
    float predictedForce = 0;
    float confidence = 0;

    if (predictorMode == 1 ) {

        predictedForce = predictedPaddle->predictStroke(forceAdapter.goingForward(), DEFAULT_PREDICTION_TIME, &confidence);
//        Serial.printf("Gf: %d, TIME: %f, Predicted force: %f\n", forceAdapter.goingForward(), DEFAULT_PREDICTION_TIME, predictedForce);

        if (modeSwitch->getMode() != MOTOR_OFF) {
            // Обучаем предиктор только если мотор работает

            float forceToLearn = fForce;

            if (fabs(fForce) < DEFAULT_FORCE_THRESHOLD) {
                forceToLearn = 0;
            }

            predictedPaddle->teachStrokePredictor(forceToLearn, forceAdapter.goingForward());
        }
    } 

    float bladeForce = predictedPaddle->getForce(currentBladeSide);
    if ((bladeForce < borderLoadForce) && (bladeForce > -borderLoadForce)) {
        fForce = 0;
    } else {
        predictedForceUsed = false;
        strokeTimeUsed = millis();
    }

    usingPredictedForce = false;
    
    
    if (predictorMode == 1) {
        if (fForce == 0 && (!predictedForceUsed || (millis() - predictedTimeUsed < DEFAULT_PREDICTION_TIME+100)) && (millis() - strokeTimeUsed > DEFAULT_PREDICTION_TIME+200)) {

//            Serial.printf("Predicted force: %f, Confidence: %f\n", predictedForce, confidence);

            if ((fabs(predictedForce) > borderLoadForce )&& (confidence > 0.5)) {
                if (!predictedForceUsed) {
                    predictedTimeUsed = millis();
                    predictedForceUsed = true;
                }
                fForce = predictedForce;
                usingPredictedForce = true;
            }
        }
    }

    if (usingPredictedForce) {
        
//        Serial.printf("Predicted force used: %f, Confidence: %f\n", predictedForce, confidence);
    }

    currentForceGramms = (int)fForce;

    if (modeSwitch->getMode() == MOTOR_OFF){
        force=0;
    }

    if (modeSwitch->getMode() == MOTOR_LOW_POWER){
         force=(int)(fForce*POWER_LOW_SCALE);
    }
    if (modeSwitch->getMode() == MOTOR_MEDIUM_POWER){
        force = (int)(fForce*POWER_MEDIUM_SCALE);
    }
    if (modeSwitch->getMode() == MOTOR_HIGH_POWER){
        force = (int)(fForce*POWER_HIGH_SCALE);
    }
    

//  check if the mode is off or paddle is not connected
    if (modeSwitch->getMode() == MOTOR_OFF){
        if (motorDriver->getForce() != 0) {
            motorDriver->stop();
        }
        return; 
    }


    motorDriver->setForce(forceAdapter.GetAdaptedForce(force));   
//    Serial.printf("Heap: %d, time: %d\n", ESP.getFreeHeap(), millis()-timestamp);

}


void SmartKayak::setPaddle(SmartPaddle* paddle) {
    if (predictedPaddle != nullptr) {
        delete predictedPaddle;
    }
    this->paddle = paddle;
    predictedPaddle = new PredictedPaddle(paddle);
    if (display != nullptr) {
        display->addPredictedPaddle(predictedPaddle);
    }
    predictedPaddle->setKayakSpecs(specs);
}

void SmartKayak::setMotorDriver(IMotorDriver* motorDriver) {
    this->motorDriver = motorDriver;
}

void SmartKayak::setModeSwitch(IModeSwitch* modeSwitch) {
    this->modeSwitch = modeSwitch;
}

void SmartKayak::setIMU(IIMUSensor* imu, uint32_t frequency) {
    this->imu = imu;

}

void SmartKayak::setDisplay(KayakDisplay* newDisplay) {
    display = newDisplay;
    if (predictedPaddle != nullptr) {
        display->addPredictedPaddle(predictedPaddle);
    }
}

void SmartKayak::logCall(ILogInterface* logger, LogMode logMode, int* loadCell, int* externalForce){
    IMUData imuData;
    OrientationData kayakOrientation;
    OrientationData paddleOrientation;
    IMUData paddleIMUData;
    loadData loads;
    int forceGramms = motorDriver->getForceGramms();
    int force=motorDriver->getForce();
    if (externalForce) {
        force = *externalForce;
    }
    switch (logMode) {
        case LogMode::LOG_MODE_OFF:
            break;
        case LogMode::LOG_MODE_KAYAK_MAG:
            imu->getData(imuData);
            logger->printf("%d\t%d\t%d\n", imuData.mag_x, imuData.mag_y, imuData.mag_z);
            Serial.printf("%d,%d,%d\n", imuData.mag_x, imuData.mag_y, imuData.mag_z);

            break;
        case LogMode::LOG_MODE_PADDLE_MAG:
            imuData = paddle->getIMUData();
            logger->printf("%d\t%d\t%d\n", imuData.mag_x, imuData.mag_y, imuData.mag_z);
            Serial.printf("%d,%d,%d\n", imuData.mag_x, imuData.mag_y, imuData.mag_z);
            break;
        case LogMode::LOG_MODE_ALL:
            imu->getOrientation(kayakOrientation);

            kayakOrientation.q0=kayakOrientationQuat[0];
            kayakOrientation.q1=kayakOrientationQuat[1];
            kayakOrientation.q2=kayakOrientationQuat[2];
            kayakOrientation.q3=kayakOrientationQuat[3];

            paddleOrientation = paddle->getOrientationData();
            imu->getData(imuData);
            paddleIMUData=paddle->getIMUData();
            loads = paddle->getLoadData();

            logger->printf("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,",
                millis(),
            kayakOrientation.q0, kayakOrientation.q1, kayakOrientation.q2, kayakOrientation.q3,
            imuData.q0, imuData.q1, imuData.q2, imuData.q3,
            imuData.ax, imuData.ay, imuData.az,
            imuData.gx, imuData.gy, imuData.gz,
            imuData.mx, imuData.my, imuData.mz,
            imuData.mag_x, imuData.mag_y, imuData.mag_z);

//            logger->printf("%d,%f,%f,%f,%f,",
//            millis(),
//            kayakOrientation.q0, kayakOrientation.q1, kayakOrientation.q2, kayakOrientation.q3);

            logger->printf("%d,%d,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,",
                paddle->paddleMillis(),
            paddleOrientation.timestamp,
            paddleOrientation.q0, paddleOrientation.q1, paddleOrientation.q2, paddleOrientation.q3,
            paddleIMUData.timestamp,
            paddleIMUData.q0, paddleIMUData.q1, paddleIMUData.q2, paddleIMUData.q3,
            paddleIMUData.ax, paddleIMUData.ay, paddleIMUData.az,
            paddleIMUData.gx, paddleIMUData.gy, paddleIMUData.gz,
            paddleIMUData.mx, paddleIMUData.my, paddleIMUData.mz,
            paddleIMUData.mag_x, paddleIMUData.mag_y, paddleIMUData.mag_z);

            logger->printf("%d,%f,%f,%f,%f,",
            loads.timestamp,
            loads.forceL, predictedPaddle->getLeftTare(),
            loads.forceR, predictedPaddle->getRightTare()
            );
            logger->printf("%d,%d,", currentForceGramms, currentBladeSide);

            String modeString;
            switch (modeSwitch->getMode()) {
                case MOTOR_OFF:
                    modeString = "OFF";
                    break;
                case MOTOR_LOW_POWER:
                    modeString = "LOW";
                    break;  
                case MOTOR_MEDIUM_POWER:
                    modeString = "MED";
                    break;
                case MOTOR_HIGH_POWER:
                    modeString = "HIGH";
                    break;
                case MOTOR_DEBUG:
                    modeString = "DEBUG";
                    break;
                default:
                    modeString = "UNKNOWN";
                    break;
            }
            logger->printf("%s,%d,%d,", modeString.c_str(), forceGramms, force);
            if (loadCell) {
                logger->printf("%d\n", *loadCell);
            } else {
                logger->printf("0\n");
            }


            break;
    }
}

void SmartKayak::startTasks() {
    SmartKayakRTOS::startTasks(this);
}

/*SP_Math::Quaternion SmartKayak::getRelativeOrientation(SP_Math::Quaternion& qp)
{
    // 1. Получаем направление оси X каяка в мировой системе координат
    SP_Math::Vector x_k = kayakOrientationQuat.rotate(SP_Math::Vector(1, 0, 0));

    // 2. Проецируем на горизонтальную плоскость (убираем наклон каяка)
    float norm_sq = x_k[0]*x_k[0]+x_k[1]*x_k[1];

    SP_Math::Vector X_new(0, 0, 0);  // Инициализируем нулями
    if (norm_sq < 1e-10f) {
        // Если каяк направлен вертикально, используем направление по умолчанию
        X_new[0]=1;
        X_new[1]=0;
        X_new[2]=0;
    } else {
        // Нормализуем горизонтальную проекцию
        float inv_norm = 1.0f / std::sqrt(norm_sq);
        X_new[0] = x_k[0] * inv_norm;
        X_new[1] = x_k[1] * inv_norm;
        X_new[2] = 0;
    }

    // 3. Строим кватернион поворота от [1,0,0] к X_new (только направление каяка, без наклона)
    // Используем формулы половинного угла для избежания тригонометрии:
    // cos(θ/2) = sqrt((1 + cos(θ))/2), где cos(θ) = X_new[0]
    // sin(θ/2) = sign(sin(θ)) * sqrt((1 - cos(θ))/2), где sign(sin(θ)) = sign(X_new[1])
    float cos_half_theta = std::sqrt(0.5f * (1 + X_new[0]));
    float sin_half_theta = (X_new[1] >= 0 ? 1 : -1) * std::sqrt(0.5f * (1 - X_new[0]));
    SP_Math::Quaternion q_new(cos_half_theta, 0, 0, sin_half_theta);

    // 4. Вычисляем ориентацию весла относительно горизонтального направления каяка
    SP_Math::Quaternion q_relative = q_new.conjugate() * qp;

    // Нормализация для численной стабильности
    return q_relative.normalize();
}*/

void SmartKayak::loadSpecs() {
    Preferences prefs;
    if (!prefs.begin(prefsName.c_str(), true)) {
        Serial.println("Failed to open preferences for loading specs");
        return;
    }
    specs.axisDirection = (AxisDirection)prefs.getInt("axisDirection", (int)Y_AXIS_RIGHT);
    prefs.end();
    if (predictedPaddle != nullptr) {
        predictedPaddle->setKayakSpecs(specs);
    }
}

void SmartKayak::saveSpecs() {
    Preferences prefs;
    if (!prefs.begin(prefsName.c_str(), false)) {
        Serial.println("Failed to open preferences for saving specs");
        return;
    }
    prefs.putInt("axisDirection", (int)specs.axisDirection);
    prefs.end();
}

void SmartKayak::setSpecs(const KayakSpecs& s, bool save) {
    specs = s;
    if (save) {
        saveSpecs();
    }
    if (predictedPaddle != nullptr) {
        predictedPaddle->setKayakSpecs(specs);
    }
}