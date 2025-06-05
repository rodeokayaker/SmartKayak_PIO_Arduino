#include "SmartKayak.h"
#include "InterfaceIMU.h"
#include "Arduino.h"
#include "SP_Quaternion.h"
#define POWER_LOW_SCALE 10
#define POWER_MEDIUM_SCALE 25
#define POWER_HIGH_SCALE 40

#define IMU_STACK_SIZE 4096
#define MAGNETOMETER_STACK_SIZE 4096

// Объявляем глобальную переменную в начале файла
SmartKayak* mainKayak = nullptr;

// Функция прерывания вне класса
void IRAM_ATTR dmpDataReady() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(mainKayak->imuTaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

class SmartKayakRTOS{
    private:
    // Убираем static SmartKayak* mainKayak; отсюда
    
    static void imuTask(void *pvParameters) {        
        SmartKayak* kayak = (SmartKayak*)pvParameters;
        if (!kayak->imu){
            vTaskDelete(NULL);
            return;
        }
        TickType_t xLastWakeTime = xTaskGetTickCount();
        uint32_t frequency = kayak->imuFrequency;
        TickType_t xFrequency = (frequency>0)?pdMS_TO_TICKS(1000/frequency):0;    
        while(1) {
            if (xFrequency==0){
               // Ждем уведомления от прерывания
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            }

            kayak->updateIMU();

            if (xFrequency>0)
                xTaskDelayUntil(&xLastWakeTime, xFrequency);

        }
    }


    static void magnetometerTask(void *pvParameters) {
        SmartKayak* kayak = (SmartKayak*)pvParameters;
        TickType_t xLastWakeTime = xTaskGetTickCount();
        if (!kayak->imu){
            vTaskDelete(NULL);
            return;
        }
        uint32_t frequency = kayak->imu->magnetometerFrequency();
        if (frequency == 0){
            vTaskDelete(NULL);
            return;
        }
        const TickType_t xFrequency = pdMS_TO_TICKS(1000/frequency);
        while(1) {
            kayak->imu->magnetometerUpdate();
 //           Serial.printf("Magnetometer task\n");
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    public:
    //All tasks setup and start
    static void startTasks(SmartKayak* kayak) {
        mainKayak = kayak;  // Используем глобальную переменную

    
        xTaskCreatePinnedToCore(
            SmartKayakRTOS::imuTask,
            "IMU",
            IMU_STACK_SIZE,
            kayak,
            1,
            &kayak->imuTaskHandle,
            1
        );

/*
        xTaskCreatePinnedToCore(
            SmartKayakRTOS::magnetometerTask,
            "Magnetometer",
            MAGNETOMETER_STACK_SIZE,
            kayak,
            1,
            &kayak->magnetometerTaskHandle,
            1
        );
        */

        // Установка прерывания для IMU на interruptPIN если он задан

        if (kayak->imu && kayak->imu->DMPEnabled() && kayak->imu->interruptPIN()>=0){
            pinMode(kayak->imu->interruptPIN(), INPUT);
            attachInterrupt(digitalPinToInterrupt(kayak->imu->interruptPIN()), dmpDataReady, RISING);
            Serial.printf("IMU interrupt pin initialized: %d\n", kayak->imu->interruptPIN());
        }

    }
};

SmartKayak::SmartKayak():
paddle(nullptr),
motorDriver(nullptr),
modeSwitch(nullptr),
imu(nullptr),
paddleNullVector(0,1,0),
paddleShaftAngle(0),
display(nullptr)
{
}

void SmartKayak::begin() {

    if (imu){
        imuFrequency = imu->getFrequency();
        if (imu->DMPEnabled()&&imu->interruptPIN()>=0)
            imuFrequency = 0;
    }
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

    if (globalYAxis.z() * Y_axis_sign < 0) {
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
//   Serial.printf("IMU:%f,%f,%f,%f\n", imu->getData().q0, imu->getData().q1, imu->getData().q2, imu->getData().q3);
   
    if (!paddle->connected()) {
        if (motorDriver->getForce() != 0) {
            motorDriver->stop();
        }
//        Serial.printf("Paddle not connected\n");
        return; 

    }   


    int force = 0;
    int borderLoadForce = 400;

    loadData loads = paddle->getLoadData();
    OrientationData paddleOrientation = paddle->getOrientationData();
//    Serial.printf("Paddle orientation: %f,%f,%f,%f\n", paddleOrientation.q0, paddleOrientation.q1, paddleOrientation.q2, paddleOrientation.q3);
    SP_Math::Quaternion currentPaddleQ(paddleOrientation.q0,paddleOrientation.q1,paddleOrientation.q2,paddleOrientation.q3);

    //Determine which blade is lower
    BladeSideType bladeSide = getLowerBladeSide(currentPaddleQ, paddle->getBladeAngles().YAxisDirection);


    if (bladeSide == BladeSideType::ALL_BLADES) {
//        Serial.printf("All blades\n");
        if (motorDriver->getForce() != 0) {
            motorDriver->stop();
        }
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

 //   Serial.printf("%s, %d, %f, %f\n", bladeSide == BladeSideType::RIGHT_BLADE ? "Right" : "Left", paddle->getBladeAngles().YAxisDirection, paddle->getBladeAngles().leftBladeAngle, paddle->getBladeAngles().rightBladeAngle);
    
    // Получаем откалиброванное значение силы с учетом гравитации
    float bladeForce = loadCellCalibrator.getCalibratedForce(
        (bladeSide == BladeSideType::RIGHT_BLADE),
        (bladeSide == BladeSideType::RIGHT_BLADE) ? loads.forceR : loads.forceL,
        paddle->getIMUData(),
        paddle->getBladeAngles()
    );
    


    OrientationData kayakOrientation = imu->getOrientation();
 //   Serial.printf("IMU orientation: %f,%f,%f,%f\n", kayakOrientation.q0, kayakOrientation.q1, kayakOrientation.q2, kayakOrientation.q3);
    SP_Math::Quaternion currentKayakQ(kayakOrientation.q0,kayakOrientation.q1,kayakOrientation.q2,kayakOrientation.q3);
 //   Serial.printf("Kayak orientation: %f,%f,%f,%f\n", kayakOrientation.q0, kayakOrientation.q1, kayakOrientation.q2, kayakOrientation.q3);

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

    if ((bladeForce < borderLoadForce) && (bladeForce > -borderLoadForce)) {
        force = 0;
    }


    motorDriver->setForce(force);    

}

void SmartKayak::updateIMU() {
    if (imu) {
        imu->readData();
        imu->updateOrientation();
//        OrientationData imuData = imu->getOrientation();
//        Serial.printf("q: %f, %f, %f, %f \n", imuData.q0, imuData.q1, imuData.q2, imuData.q3);
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

void SmartKayak::logCall(ILogInterface* logger, LogMode logMode, int* loadCell, int* externalForce){
    IMUData imuData;
    OrientationData kayakOrientation;
    OrientationData paddleOrientation;
    IMUData paddleIMUData;
    loadData loads;
    int force=motorDriver->getForce();
    if (externalForce) {
        force = *externalForce;
    }
    switch (logMode) {
        case LogMode::LOG_MODE_OFF:
            break;
        case LogMode::LOG_MODE_KAYAK_MAG:
            imuData = imu->getData();
            logger->printf("%d\t%d\t%d\n", imuData.mag_x, imuData.mag_y, imuData.mag_z);
            Serial.printf("%d,%d,%d\n", imuData.mag_x, imuData.mag_y, imuData.mag_z);

            break;
        case LogMode::LOG_MODE_PADDLE_MAG:
            imuData = paddle->getIMUData();
            logger->printf("%d\t%d\t%d\n", imuData.mag_x, imuData.mag_y, imuData.mag_z);
            Serial.printf("%d,%d,%d\n", imuData.mag_x, imuData.mag_y, imuData.mag_z);
            break;
        case LogMode::LOG_MODE_ALL:
            kayakOrientation = imu->getOrientation();
            paddleOrientation = paddle->getOrientationData();
            imuData=imu->getData();
            paddleIMUData=paddle->getIMUData();
            loads = paddle->getLoadData();
            force = motorDriver->getForce();


            logger->printf("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,",
                millis(),
            kayakOrientation.q0, kayakOrientation.q1, kayakOrientation.q2, kayakOrientation.q3,
            imuData.q0, imuData.q1, imuData.q2, imuData.q3,
            imuData.ax, imuData.ay, imuData.az,
            imuData.gx, imuData.gy, imuData.gz,
            imuData.mx, imuData.my, imuData.mz,
            imuData.mag_x, imuData.mag_y, imuData.mag_z);

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
            logger->printf("%d,",
            force);
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

