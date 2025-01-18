#include "SmartKayak.h"
#include "InterfaceIMU.h"

SmartKayak::SmartKayak():
paddle(nullptr),
motorDriver(nullptr),
modeSwitch(nullptr),
imu(nullptr)
{

}

void SmartKayak::begin() {
}

void SmartKayak::update() {
    updateIMU();
}

void SmartKayak::updateIMU() {
    if (imu) imu->update();
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
    
}
