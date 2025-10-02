#include "SmartKayak.h"
//#include "InterfaceIMU.h"
#include "ImuSensor.h"
#include "Arduino.h"
#include "SP_Quaternion.h"
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

                //---------------------------------TEMPORARY FIX---------------------------------
                FIXROTATIONVECTOR(mainKayak->kayakOrientationQuat[0], mainKayak->kayakOrientationQuat[1], mainKayak->kayakOrientationQuat[2], mainKayak->kayakOrientationQuat[3], 3);
                //---------------------------------TEMPORARY FIX---------------------------------
        
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

SmartKayak::SmartKayak():
paddle(nullptr),
motorDriver(nullptr),
modeSwitch(nullptr),
imu(nullptr),
paddleNullVector(0,1,0),
paddleShaftAngle(0),
display(nullptr),
currentBladeSide(BladeSideType::ALL_BLADES),
currentForceGramms(0)
{
    currentLoadCellData.forceR = 0;
    currentLoadCellData.forceL = 0;
    currentLoadCellData.forceR_raw = 0;
    currentLoadCellData.forceL_raw = 0;
    currentLoadCellData.timestamp = 0;
}

void SmartKayak::begin() {

//    imu->begin();
//    imu->setOrientationFrequency(100);

}

void getPaddleAngles(const SP_Math::Quaternion& currentPaddleQ, const SP_Math::Quaternion& currentKayakQ, 
                     float& shaftRotationAngle,  // поворот вокруг оси Z каяка
                     float& shaftTiltAngle,      // наклон вокруг оси X каяка
                     float& bladeRotationAngle)  // поворот вокруг оси Y весла
{

    SP_Math::Vector paddleY(0, 1, 0);  // ось шафта
    SP_Math::Vector globalZ(0, 0, 1);  // вертикальный вектор
    
    // Поворачиваем векторы в текущее положение
    SP_Math::Vector currentShaftDir = currentKayakQ.conjugate().rotate(currentPaddleQ.rotate(paddleY));
    SP_Math::Vector currentZinPaddle = currentPaddleQ.conjugate().rotate(globalZ); // Вертикальный вектор в системе весла
    
    // 5. Вычисляем углы поворота шафта
    // Проекция оси шафта на плоскость XY каяка
    SP_Math::Vector shaftProjectionXY(currentShaftDir.x(), currentShaftDir.y(), 0);
    shaftProjectionXY.normalize();
    
    // Угол поворота шафта вокруг Z (в плоскости XY)
    SP_Math::Vector sideDir(0,1,0);

    // 5. Вычисляем угол между векторами
    float cosAngle = sideDir.dot(currentShaftDir);
    float crossZ = sideDir.x() * currentShaftDir.y() - sideDir.y() * currentShaftDir.x();    

    shaftRotationAngle = atan2(crossZ, cosAngle) * RAD_TO_DEG;
    
    // Угол наклона шафта относительно плоскости XY
    shaftTiltAngle = asin(currentShaftDir.z()) * RAD_TO_DEG;
    
    // 6. Вычисляем угол поворота лопасти вокруг оси шафта
    // Проекция нормали лопасти на плоскостьX,Z


    // Угол поворота лопасти
    bladeRotationAngle = atan2(currentZinPaddle.x(),currentZinPaddle.z());
    bladeRotationAngle = bladeRotationAngle * RAD_TO_DEG;
}

// Перегруженная версия для работы с относительной ориентацией весла
void getPaddleAngles(const SP_Math::Quaternion& relativePaddleQ, 
                     float& shaftRotationAngle,  // поворот вокруг оси Z каяка
                     float& shaftTiltAngle,      // наклон вокруг оси X каяка
                     float& bladeRotationAngle)  // поворот вокруг оси Y весла
{
    SP_Math::Vector paddleY(0, 1, 0);  // ось шафта
    SP_Math::Vector globalZ(0, 0, 1);  // вертикальный вектор
    
    // Поворачиваем векторы с использованием относительной ориентации
    SP_Math::Vector currentShaftDir = relativePaddleQ.rotate(paddleY);
    SP_Math::Vector currentZinPaddle = relativePaddleQ.conjugate().rotate(globalZ); // Вертикальный вектор в системе весла
    
    // Вычисляем углы поворота шафта
    // Проекция оси шафта на плоскость XY каяка
    SP_Math::Vector shaftProjectionXY(currentShaftDir.x(), currentShaftDir.y(), 0);
    shaftProjectionXY.normalize();
    
    // Угол поворота шафта вокруг Z (в плоскости XY)
    SP_Math::Vector sideDir(0,1,0);

    // Вычисляем угол между векторами
    float cosAngle = sideDir.dot(currentShaftDir);
    float crossZ = sideDir.x() * currentShaftDir.y() - sideDir.y() * currentShaftDir.x();    

    shaftRotationAngle = atan2(crossZ, cosAngle) * RAD_TO_DEG;
    
    // Угол наклона шафта относительно плоскости XY
    shaftTiltAngle = asin(currentShaftDir.z()) * RAD_TO_DEG;
    
    // Угол поворота лопасти вокруг оси шафта
    bladeRotationAngle = atan2(currentZinPaddle.x(),currentZinPaddle.z());
    bladeRotationAngle = bladeRotationAngle * RAD_TO_DEG;
}

BladeSideType getLowerBladeSide(const SP_Math::Quaternion& paddleQ, int Y_axis_sign) {
    if (Y_axis_sign == 0) {
        return BladeSideType::ALL_BLADES;
    }
    // Получаем текущее направление оси весла направленного на правую лопатку в глобальной системе
    SP_Math::Vector paddleYAxis(0, Y_axis_sign, 0);
    SP_Math::Vector globalYAxis = paddleQ.rotate(paddleYAxis);

    if (globalYAxis.z() < 0) {
        return BladeSideType::RIGHT_BLADE;
    } else {
        return BladeSideType::LEFT_BLADE;
    }

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
   
    if (!paddle->connected()) {
        motorDriver->stop();
        return; 

    }   

    int force = 0;
    int borderLoadForce = 600;

    loadData loads = paddle->getLoadData();
    if (LOADCELL_SMOOTHING_FACTOR > 0) {
        currentLoadCellData.forceR = (currentLoadCellData.forceR * (1 - LOADCELL_SMOOTHING_FACTOR) + loads.forceR * LOADCELL_SMOOTHING_FACTOR);
        currentLoadCellData.forceL = (currentLoadCellData.forceL * (1 - LOADCELL_SMOOTHING_FACTOR) + loads.forceL * LOADCELL_SMOOTHING_FACTOR);
        currentLoadCellData.forceR_raw = (currentLoadCellData.forceR_raw * (1 - LOADCELL_SMOOTHING_FACTOR) + loads.forceR_raw * LOADCELL_SMOOTHING_FACTOR);
        currentLoadCellData.forceL_raw = (currentLoadCellData.forceL_raw * (1 - LOADCELL_SMOOTHING_FACTOR) + loads.forceL_raw * LOADCELL_SMOOTHING_FACTOR);
        currentLoadCellData.timestamp = loads.timestamp;
        loads = currentLoadCellData;
    } 

    OrientationData paddleOrientation = paddle->getOrientationData();
    if (paddleOrientation.q0 == 0 && paddleOrientation.q1 == 0 && paddleOrientation.q2 == 0 && paddleOrientation.q3 == 0) {
        return; 
    }

    SP_Math::Quaternion currentPaddleQ(paddleOrientation.q0,paddleOrientation.q1,paddleOrientation.q2,paddleOrientation.q3);
    SP_Math::Quaternion paddleRelativeQuat = getRelativeOrientation(currentPaddleQ,paddle);
    //Determine which blade is lower
    BladeSideType bladeSide = getLowerBladeSide(currentPaddleQ, paddle->getBladeAngles().YAxisDirection);
    currentBladeSide = bladeSide;

    if (bladeSide == BladeSideType::ALL_BLADES) {
        motorDriver->stop();
        return; 
    }

    // Обновляем калибровку с учетом IMU
    loadCellCalibrator.updateTare(
        (bladeSide == BladeSideType::RIGHT_BLADE),
        loads.forceL,
        loads.forceR,
        paddle->getIMUData(),
        paddle->getBladeAngles()
    );
    
    // Получаем откалиброванное значение силы с учетом гравитации
    float bladeForce = loadCellCalibrator.getCalibratedForce(
        (bladeSide == BladeSideType::RIGHT_BLADE),
        (bladeSide == BladeSideType::RIGHT_BLADE) ? loads.forceR : loads.forceL,
        paddle->getIMUData(),
        paddle->getBladeAngles()
    );
    
 
    SP_Math::Vector paddleNormal( 
        (bladeSide == BladeSideType::RIGHT_BLADE) ? 
        paddle->getBladeAngles().rightBladeVector : 
        paddle->getBladeAngles().leftBladeVector);
    

    SP_Math::Vector kayakPaddleCorrectedNormal = paddleRelativeQuat.rotate(paddleNormal);
//        SP_Math::Vector kayakPaddleCorrectedNormal = kayakOrientationQuat.conjugate().rotate(currentPaddleQ.rotate(paddleNormal));
     
    float shaftRotationAngle;  // поворот вокруг оси Z каяка
    float shaftTiltAngle;      // наклон вокруг оси X каяка
    float bladeRotationAngle;  // поворот вокруг оси Y весла
    getPaddleAngles(paddleRelativeQuat, shaftRotationAngle, shaftTiltAngle, bladeRotationAngle);

//    shaftRotationAngle = 0;
    int shaftRotationAngleInt = (int)shaftRotationAngle;
    int shaftTiltAngleInt = (int)shaftTiltAngle;
    int bladeRotationAngleInt = (int)bladeRotationAngle;

    kayakPaddleCorrectedNormal.normalize();
    float cosAngle = kayakPaddleCorrectedNormal.x();

    float fForce = bladeForce*cosAngle;

     if ((bladeForce < borderLoadForce) && (bladeForce > -borderLoadForce)) {
        fForce = 0;
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
    displayData=display->getCurrentDisplayData();

    // Обновляем данные для отображения
    loads = paddle->getLoadData();
    displayData.shaftRotationAngle = shaftRotationAngle;
    displayData.shaftTiltAngle = shaftTiltAngle;
    displayData.bladeRotationAngle = bladeRotationAngle;
    displayData.leftForce = loads.forceL;
    displayData.rightForce = loads.forceR;
    displayData.leftTare = loadCellCalibrator.getLeftTare();
    displayData.rightTare = loadCellCalibrator.getRightTare();
    displayData.isRightBlade = (bladeSide == BladeSideType::RIGHT_BLADE);
    displayData.leftBladeAngle = paddle->getBladeAngles().leftBladeAngle;
    displayData.rightBladeAngle = paddle->getBladeAngles().rightBladeAngle;
    displayData.motorForce = motorDriver->getForce();
    
    if (display) {
        display->update(displayData);
    }

//  check if the mode is off or paddle is not connected
    if (modeSwitch->getMode() == MOTOR_OFF){
        if (motorDriver->getForce() != 0) {
            motorDriver->stop();
        }
        return; 
    }


    motorDriver->setForce(forceAdapter.GetAdaptedForce(force));    

}


void SmartKayak::setPaddle(SmartPaddle* paddle) {
    this->paddle = paddle;
}

void SmartKayak::setMotorDriver(IMotorDriver* motorDriver) {
    this->motorDriver = motorDriver;
}

void SmartKayak::setModeSwitch(IModeSwitch* modeSwitch) {
    this->modeSwitch = modeSwitch;
}

void SmartKayak::setIMU(ImuSensor* imu, uint32_t frequency) {
    this->imu = imu;

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
            loads.forceL, loadCellCalibrator.getLeftTare(),
            loads.forceR, loadCellCalibrator.getRightTare()
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

SP_Math::Quaternion SmartKayak::getRelativeOrientation(SP_Math::Quaternion& qp, SmartPaddle* paddle)
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


}

//---------------------------------Anticipation---------------------------------

void SmartKayak::updateAnticipationLogic(float shaftTiltAngle, float bladeForce, int& force) {
    unsigned long currentTime = millis();
    float pitchThreshold = anticipationSettings.triggerPitchAngle;
    float hysteresis = anticipationSettings.hysteresis;
    
    // Машина состояний для предвосхищения
    switch (anticipationState) {
        case AnticipationState::IDLE:
            // Ждем срабатывания по углу pitch
            if (shaftTiltAngle <= pitchThreshold) {
                transitionToState(AnticipationState::ANTICIPATION_TRIGGERED);
                anticipationStartTime = currentTime;
                anticipationTriggerCount++;
                
                // Рассчитываем предварительную силу (50% от максимальной)
                anticipatedForce = calculateAnticipatedForce(bladeForce, anticipationSettings.minMotorPower);
                force = anticipatedForce;
                
                Serial.printf("🎯 Anticipation triggered at pitch: %.1f°\n", shaftTiltAngle);
            } else {
                force = 0;
            }
            break;
            
        case AnticipationState::ANTICIPATION_TRIGGERED:
            // Мотор включен на минимальной мощности, ждем подтверждения
            force = anticipatedForce;
            
            // Проверяем появление силы
            if (abs(bladeForce) > anticipationSettings.borderLoadForce) {
                transitionToState(AnticipationState::FORCE_CONFIRMED);
                successfulAnticipationCount++;
                Serial.printf("✅ Force confirmed: %.0f, switching to full power\n", bladeForce);
            }
            // Проверяем таймаут предвосхищения
            else if (currentTime - anticipationStartTime > anticipationSettings.anticipationTime) {
                transitionToState(AnticipationState::MOTOR_ACTIVE);
                Serial.printf("⏱️ Anticipation timeout, checking for real force\n");
            }
            // Проверяем возврат угла (ложное срабатывание)
            else if (shaftTiltAngle > pitchThreshold + hysteresis) {
                transitionToState(AnticipationState::IDLE);
                falsePositiveCount++;
                force = 0;
                Serial.printf("❌ False positive, angle returned: %.1f°\n", shaftTiltAngle);
            }
            break;
            
        case AnticipationState::MOTOR_ACTIVE:
            // Проверяем реальную силу после времени предвосхищения
            if (abs(bladeForce) > anticipationSettings.borderLoadForce) {
                transitionToState(AnticipationState::FORCE_CONFIRMED);
                successfulAnticipationCount++;
                Serial.printf("✅ Delayed force confirmation: %.0f\n", bladeForce);
            } else {
                // Нет реальной силы - возможно ложное срабатывание
                transitionToState(AnticipationState::IDLE);
                falsePositiveCount++;
                force = 0;
                Serial.printf("❌ No force detected, false anticipation\n");
            }
            break;
            
        case AnticipationState::FORCE_CONFIRMED:
            // Работаем на полной мощности согласно режиму
            {
                SP_Math::Vector kayakPaddleCorrectedNormal(1,0,0);  //WRONG!!!
                
                
                kayakPaddleCorrectedNormal.normalize();
                float cosAngle = kayakPaddleCorrectedNormal.x();
                float fForce = bladeForce * cosAngle;
                
                if (modeSwitch->getMode() == MOTOR_LOW_POWER){
                     force = fForce * (POWER_LOW_SCALE/10.f);
                }
                else if (modeSwitch->getMode() == MOTOR_MEDIUM_POWER){
                    force = fForce * (POWER_MEDIUM_SCALE/10.f);
                }
                else if (modeSwitch->getMode() == MOTOR_HIGH_POWER){
                    force = fForce * (POWER_HIGH_SCALE/10.f);
                }
                
                // Проверяем окончание гребка
                if (abs(bladeForce) < anticipationSettings.borderLoadForce && 
                    currentTime - lastStateChangeTime > 500) { // Минимум 0.5 сек работы
                    transitionToState(AnticipationState::IDLE);
                    force = 0;
                    Serial.printf("🏁 Stroke finished, returning to idle\n");
                }
            }
            break;
    }
    
    // Проверяем общий таймаут для сброса в любом состоянии
    if (currentTime - lastStateChangeTime > anticipationSettings.timeoutTime) {
        if (anticipationState != AnticipationState::IDLE) {
            Serial.printf("⚠️ State timeout, resetting to idle\n");
            transitionToState(AnticipationState::IDLE);
            force = 0;
        }
    }
}

void SmartKayak::transitionToState(AnticipationState newState) {
    if (newState != anticipationState) {
        Serial.printf("🔄 State transition: %d -> %d\n", (int)anticipationState, (int)newState);
        anticipationState = newState;
        lastStateChangeTime = millis();
    }
}

int SmartKayak::calculateAnticipatedForce(float bladeForce, float anticipationFactor) {
    // Возвращаем фиксированную силу для предвосхищения
    // Или можем использовать историю предыдущих гребков
    int baseForce = 200; // Базовая сила для предвосхищения
    
    if (modeSwitch->getMode() == MOTOR_LOW_POWER) {
        return baseForce * POWER_LOW_SCALE / 10;
    }
    else if (modeSwitch->getMode() == MOTOR_MEDIUM_POWER) {
        return baseForce * POWER_MEDIUM_SCALE / 10;
    }
    else if (modeSwitch->getMode() == MOTOR_HIGH_POWER) {
        return baseForce * POWER_HIGH_SCALE / 10;
    }
    
    return baseForce;
}

