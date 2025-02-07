#include "SmartKayak.h"
#include "InterfaceIMU.h"
#include "Arduino.h"
#include "SP_Quaternion.h"
#define POWER_LOW_SCALE 20
#define POWER_MEDIUM_SCALE 30
#define POWER_HIGH_SCALE 40


SmartKayak::SmartKayak():
paddle(nullptr),
motorDriver(nullptr),
modeSwitch(nullptr),
imu(nullptr),
paddleNullVector(0,1,0),
paddleShaftAngle(0),
textLCD1(nullptr),
textLCD2(nullptr)
{
}

void SmartKayak::begin() {
    leftTare={0,0,0};
    rightTare={0,0,0};
    log_level = 0;
    lastPrintLCD1 = millis();
    lastPrintLCD2 = millis();
}

void getPaddleAngles(const SP_Math::Quaternion& currentPaddleQ, const SP_Math::Quaternion& currentKayakQ, 
                     const SP_Math::Quaternion& paddleCalibQuaternion, 
                     float& shaftRotationAngle,  // поворот вокруг оси Z каяка
                     float& shaftTiltAngle,      // наклон вокруг оси X каяка
                     float& bladeRotationAngle)  // поворот вокруг оси Y весла
{

    SP_Math::Vector paddleY(0, 1, 0);  // ось шафта
    SP_Math::Vector globalZ(0, 0, 1);  // вертикальный вектор
    
    // Поворачиваем векторы в текущее положение
    SP_Math::Vector currentShaftDir = paddleCalibQuaternion.conjugate().rotate(currentKayakQ.conjugate().rotate(currentPaddleQ.rotate(paddleY)));
    SP_Math::Vector currentZinPaddle = currentPaddleQ.conjugate().rotate(globalZ); // Вертикальный вектор в системе весла
    
    // 5. Вычисляем углы поворота шафта
    // Проекция оси шафта на плоскость XY каяка
    SP_Math::Vector shaftProjectionXY(currentShaftDir.x(), currentShaftDir.y(), 0);
    shaftProjectionXY.normalize();
    
    // Угол поворота шафта вокруг Z (в плоскости XY)
    SP_Math::Vector sideDir(0,1,0);
//    sideDir = paddleCalibQuaternion.conjugate().rotate(sideDir);
//    Serial.printf("Side dir: %f,%f,%f\n", sideDir.x(), sideDir.y(), sideDir.z());

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

BladeSideType getLowerBladeSide(const SP_Math::Quaternion& paddleQ, int Y_axis_sign) {
    if (Y_axis_sign == 0) {
        return BladeSideType::ALL_BLADES;
    }
    // Получаем текущее направление оси Y весла в глобальной системе
    SP_Math::Vector paddleYAxis(0, 1, 0);
    SP_Math::Vector globalYAxis = paddleQ.rotate(paddleYAxis);

    if (globalYAxis.z() * Y_axis_sign > 0) {
        return BladeSideType::LEFT_BLADE;
    } else {
        return BladeSideType::RIGHT_BLADE;
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
        if (motorDriver->getForce() != 0) {
            motorDriver->stop();
        }
        return; 
    }   


    int force = 0;
    int borderLoadForce = 3000;

    loadData loads = paddle->getLoadData();
    IMUData imuData = paddle->getIMUData();
    SP_Math::Quaternion currentPaddleQ(imuData.q0,imuData.q1,imuData.q2,imuData.q3);



    //Determine which blade is lower
    BladeSideType bladeSide = getLowerBladeSide(currentPaddleQ, paddle->getBladeAngles().YAxisDirection);



    if (bladeSide == BladeSideType::ALL_BLADES) {
        if (motorDriver->getForce() != 0) {
            motorDriver->stop();
        }
        return; 
    }

    //get the force of the lower blade
    float bladeForce = bladeSide == BladeSideType::RIGHT_BLADE ? loads.forceR : loads.forceL;
    


    if (bladeSide == BladeSideType::RIGHT_BLADE) {
        if (rightTare.samples> 100) {
            //changed blade
            double avg = rightTare.sum/rightTare.samples;
            rightTare.average = rightTare.average*0.5+avg*0.5;
//            Serial.printf("Right tare average: %f\n", rightTare.average);
            rightTare.samples = 0;
            rightTare.sum = 0;
        }
        leftTare.samples++;
        leftTare.sum+=loads.forceL;
    } else {
        if (leftTare.samples> 100) {
            //changed blade
            double avg = leftTare.sum/leftTare.samples;
            leftTare.average = leftTare.average*0.8+avg*0.2;
            
//            Serial.printf("Left tare average: %f\n", leftTare.average);
            leftTare.samples = 0;
            leftTare.sum = 0;
        }
        rightTare.samples++;
        rightTare.sum += loads.forceR;
    }

    float tareForce = (bladeSide == BladeSideType::RIGHT_BLADE) ? rightTare.average : leftTare.average;



    bladeForce = bladeForce - tareForce;

//    Serial.printf("Blade force: %f\n", bladeForce);
 


    imuData = imu->getData();
    SP_Math::Quaternion currentKayakQ(imuData.q0,imuData.q1,imuData.q2,imuData.q3);

    SP_Math::Vector paddleNormal( 
        (bladeSide == BladeSideType::RIGHT_BLADE) ? 
        paddle->getBladeAngles().rightBladeVector : 
        paddle->getBladeAngles().leftBladeVector);

    SP_Math::Vector kayakPaddleCorrectedNormal = paddleCalibQuaternion.conjugate().rotate(currentKayakQ.conjugate().rotate(currentPaddleQ.rotate(paddleNormal)));
     
    float shaftRotationAngle;  // поворот вокруг оси Z каяка
    float shaftTiltAngle;      // наклон вокруг оси X каяка
    float bladeRotationAngle;  // поворот вокруг оси Y весла
    getPaddleAngles(currentPaddleQ, currentKayakQ, paddleCalibQuaternion, shaftRotationAngle, shaftTiltAngle, bladeRotationAngle);

    int shaftRotationAngleInt = (int)shaftRotationAngle;
    int shaftTiltAngleInt = (int)shaftTiltAngle;
    int bladeRotationAngleInt = (int)bladeRotationAngle;

    if (textLCD2 && (millis() - lastPrintLCD2 > 100)) {
        lastPrintLCD2 = millis();
        textLCD2->setCursor(0, 0);
        char buf[22];
        snprintf(buf, sizeof(buf), "%4d %4d %4d", shaftRotationAngleInt, shaftTiltAngleInt, bladeRotationAngleInt);
        textLCD2->print(buf);
        int leftBladeAngleInt = -(int)(paddle->getBladeAngles().leftBladeAngle*RAD_TO_DEG)+bladeRotationAngleInt+90;
        int rightBladeAngleInt = -(int)(paddle->getBladeAngles().rightBladeAngle*RAD_TO_DEG)+bladeRotationAngleInt+90;
        if (leftBladeAngleInt > 180) {
            leftBladeAngleInt = leftBladeAngleInt - 360;
        }
        if (rightBladeAngleInt > 180) {
            rightBladeAngleInt = rightBladeAngleInt - 360;
        }
        if (leftBladeAngleInt < -180) {
            leftBladeAngleInt = leftBladeAngleInt + 360;
        }
        if (rightBladeAngleInt < -180) {
            rightBladeAngleInt = rightBladeAngleInt + 360;
        }
        snprintf(buf, sizeof(buf), "%4d %s %4d", leftBladeAngleInt, (bladeSide == BladeSideType::RIGHT_BLADE) ? "right>" : "<left ", rightBladeAngleInt);
        textLCD2->setCursor(0, 1);
        textLCD2->print(buf);
    }


    kayakPaddleCorrectedNormal.normalize();
    float cosAngle = kayakPaddleCorrectedNormal.x();

    float fForce = bladeForce*cosAngle;
    if (modeSwitch->getMode() == MOTOR_OFF){
        force=0;
    }

    if (modeSwitch->getMode() == MOTOR_LOW_POWER){
         force=fForce*(POWER_LOW_SCALE/10.f);
    }
    if (modeSwitch->getMode() == MOTOR_MEDIUM_POWER){
        force = fForce*(POWER_MEDIUM_SCALE/10.f);
    }
    if (modeSwitch->getMode() == MOTOR_HIGH_POWER){
        force = fForce*(POWER_HIGH_SCALE/10.f);
    }
    if (log_level & SMARTKAYAK_LOG_FORCE != 0) {
        Serial.printf("Force: %d\n",force);
    }

    if (textLCD1 && (millis() - lastPrintLCD1 > 100)) {
        lastPrintLCD1 = millis();
        textLCD1->setCursor(0, 0);
        char buf[22];
        snprintf(buf, sizeof(buf), "%6d  TARE  %6d", (int)(leftTare.average), (int)(rightTare.average));
        textLCD1->print(buf);
        snprintf(buf, sizeof(buf), "%6d  LOAD  %6d", (int)((loads.forceL-leftTare.average)), (int)((loads.forceR-rightTare.average)));
        textLCD1->setCursor(0, 1);
        textLCD1->print(buf);
        float cosL = paddleCalibQuaternion.conjugate().rotate(currentKayakQ.conjugate().rotate(currentPaddleQ.rotate(paddle->getBladeAngles().leftBladeVector))).normalize().x();
        float cosR = paddleCalibQuaternion.conjugate().rotate(currentKayakQ.conjugate().rotate(currentPaddleQ.rotate(paddle->getBladeAngles().rightBladeVector))).normalize().x();
        snprintf(buf, sizeof(buf), "%6.3f  PROJ  %6.3f", cosL,cosR);
        textLCD1->setCursor(0, 2);
        textLCD1->print(buf);
        snprintf(buf, sizeof(buf), "Force: %6d", force);
        textLCD1->setCursor(0, 3);
        textLCD1->print(buf);
    }

//  check if the mode is off or paddle is not connected
    if (modeSwitch->getMode() == MOTOR_OFF){
        if (motorDriver->getForce() != 0) {
            motorDriver->stop();
        }
        return; 
    }
    //if the force is too low, stop the motor
    if ((bladeForce < borderLoadForce) && (bladeForce > -borderLoadForce)) {
        if (motorDriver->getForce() != 0) {
            motorDriver->stop();
        }
        return; 
    }


    motorDriver->setForce(force);


}

void SmartKayak::updateIMU() {
    if (imu) {
        imu->readData();
    }
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

void SmartKayak::setIMU(IIMU* imu, uint32_t frequency) {
    this->imu = imu;
    this->imuFrequency = frequency;
//    imu->setFrequency(frequency);
}

void SmartKayak::logState(ILogInterface* logger) {

    logger->printf("%d;",millis());
    logger->logIMU(imu->getData());
    logger->logOrientation(imu->getOrientation());
    logger->logIMU(paddle->getIMUData());
    logger->logOrientation(paddle->getOrientationData());
    logger->logLoads(paddle->getLoadData());
    uint8_t mode=(uint8_t)modeSwitch->getMode();
    logger->printf("%d;",mode);
    logger->printf("%d;",motorDriver->getForce());
    logger->flush();

}

struct Vector3 {
    float x, y, z;
};

float dot(const Vector3& a, const Vector3& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vector3 cross(const Vector3& a, const Vector3& b) {
    return {
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    };
}

void normalize(Vector3& v) {
    float len = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if (len > 0) {
        v.x /= len;
        v.y /= len;
        v.z /= len;
    }
}



void SmartKayak::calibratePaddle(){


    IMUData imuData = paddle->getIMUData();
    SP_Math::Quaternion paddleQ(imuData.q0,imuData.q1,imuData.q2,imuData.q3);
    imuData = imu->getData();
    SP_Math::Quaternion kayakQ(imuData.q0,imuData.q1,imuData.q2,imuData.q3);

    // 1. Получаем направление оси Y весла в глобальной системе
    SP_Math::Vector paddleYAxis(0, 1, 0);
    SP_Math::Vector globalPaddleY = paddleQ.rotate(paddleYAxis);
    
    // 2. Переводим это направление в систему координат каяка
    paddleNullVector = kayakQ.conjugate().rotate(globalPaddleY);
    
    // 3. Проецируем на плоскость XY каяка (обнуляем Z)
    paddleNullVector.z() = 0;
    paddleNullVector.normalize();  // нормализуем после проекции
    
    // 4. Ось Y в системе каяка (она уже в системе каяка)
    SP_Math::Vector kayakY(0, 1, 0);
    
    // 5. Вычисляем угол между векторами
    float cosAngle = paddleNullVector.dot(kayakY);
    float crossZ = paddleNullVector.x() * kayakY.y() - paddleNullVector.y() * kayakY.x();
    
    // Используем atan2 для получения угла с правильным знаком
    paddleShaftAngle = atan2(crossZ, cosAngle);
    if (paddleNullVector.y() > 0) {
        float w = sqrt((1 + paddleNullVector.y())/2);
        float z = -paddleNullVector.x()/(2*w);
        paddleCalibQuaternion = SP_Math::Quaternion(w, 0, 0, z);
    } else {
        float z = sqrt((1 - paddleNullVector.y())/2);
        float w = -paddleNullVector.x()/(2*z);
        paddleCalibQuaternion = SP_Math::Quaternion(w, 0, 0, z);
    }

}

void SmartKayak::logVizualizeSerial(){
    IMUData kayakIMUData = imu->getData();
    IMUData paddleIMUData = paddle->getIMUData();
    Serial.printf("Kayak,%f,%f,%f,%f\n", kayakIMUData.q0, kayakIMUData.q1, kayakIMUData.q2, kayakIMUData.q3);
//    Serial.printf("Paddle,%f,%f,%f,%f\n", paddleIMUData.q0, paddleIMUData.q1, paddleIMUData.q2, paddleIMUData.q3);
    SP_Math::Quaternion kayakQ(kayakIMUData.q0,kayakIMUData.q1,kayakIMUData.q2,kayakIMUData.q3);
    SP_Math::Quaternion paddleQ(paddleIMUData.q0,paddleIMUData.q1,paddleIMUData.q2,paddleIMUData.q3);
    SP_Math::Quaternion kayakPaddleQ = paddleCalibQuaternion*paddleQ;
    Serial.printf("Paddle,%f,%f,%f,%f\n", kayakPaddleQ.x(), kayakPaddleQ.y(), kayakPaddleQ.z(), kayakPaddleQ.w());
}

void SmartKayak::logVizualizeMag(){
    IMUData kayakIMUData = imu->getData();
    OrientationData kayakOrientation = imu->getOrientation();
    Serial.printf("DMP,%f,%f,%f,%f\n", kayakIMUData.q0, kayakIMUData.q1, kayakIMUData.q2, kayakIMUData.q3);
    Serial.printf("Fusion,%f,%f,%f,%f\n", kayakOrientation.q0, kayakOrientation.q1, kayakOrientation.q2, kayakOrientation.q3);
    Serial.printf("Mag,%f,%f,%f\n", kayakIMUData.mx, kayakIMUData.my, kayakIMUData.mz);
    Serial.printf("Accel,%f,%f,%f\n", kayakIMUData.ax, kayakIMUData.ay, kayakIMUData.az);

}
