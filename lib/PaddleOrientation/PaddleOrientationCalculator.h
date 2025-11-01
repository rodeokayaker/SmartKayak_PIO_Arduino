#ifndef PADDLEORIENTATIONCALCULATOR_H
#define PADDLEORIENTATIONCALCULATOR_H

#include "SmartPaddle.h"
#include "../Math/SP_Quaternion.h"
#include "AngularAccelerationCalculator.h"

#define PO_MODIFICATION_VALID 0b0001
#define PO_ORIENTATION_VALID 0b0010
#define PO_ANGLES_VALID 0b0100
#define PO_ALL_ORIENTATION_VALID 0b0111
#define PO_ANGULAR_ACCELERATION_VALID 0b1000

class PaddleOrientationCalculator {
    private:
        SmartPaddle* paddle;
        AxisDirection kayakAxisDirection;
        AxisDirection paddleAxisDirection;
        SP_Math::Vector paddleShaftVector;
        SP_Math::Vector kayakDirectionVector;
        AngularAccelerationCalculator angularAccelerationCalculator;

        SP_Math::Quaternion kayakOrientationQuat;
        SP_Math::Quaternion paddleOrientationQuat;
        SP_Math::Quaternion paddleRelativeQuat;
        SP_Math::Quaternion paddleModificationQuat;
        SP_Math::Vector angularVelocity;
        SP_Math::Vector angularAcceleration;
        float paddleShaftRotationAngle;
        float paddleShaftTiltAngle;
        float paddleBladeRotationAngle;
        uint8_t calculationState;  

        void updateRelativeOrientation();
        void updateAngles();

        


    public:
        PaddleOrientationCalculator(SmartPaddle* sp=nullptr) : paddle(sp),
         kayakAxisDirection(X_AXIS_FORWARD),
        paddleAxisDirection(Y_AXIS_RIGHT),
        kayakOrientationQuat(1, 0, 0, 0),
        paddleOrientationQuat(1, 0, 0, 0),
        paddleRelativeQuat(1, 0, 0, 0),
        paddleModificationQuat(1, 0, 0, 0),
        calculationState(0),
        angularAccelerationCalculator(),
        paddleShaftVector(0, 1, 0),
        kayakDirectionVector(1, 0, 0)
        {
            
            if (sp) {
                setPaddleAxisDirection(sp->getSpecs().axisDirection);
            }
        }
        ~PaddleOrientationCalculator() {}

        void setKayakAxisDirection(AxisDirection kaDir); 
        void setPaddleAxisDirection(AxisDirection paDir);

        void updateKayakOrientation(const OrientationData& kayakOrientation) {
            kayakOrientationQuat[0] = kayakOrientation.q0;
            kayakOrientationQuat[1] = kayakOrientation.q1;
            kayakOrientationQuat[2] = kayakOrientation.q2;
            kayakOrientationQuat[3] = kayakOrientation.q3;
            calculationState &= ~PO_ALL_ORIENTATION_VALID;

        };
        void updateKayakIMU(const IMUData& kayakIMU) {};

        void updatePaddleOrientation(const OrientationData& paddleOrientation)
        {
            paddleOrientationQuat[0] = paddleOrientation.q0;
            paddleOrientationQuat[1] = paddleOrientation.q1;
            paddleOrientationQuat[2] = paddleOrientation.q2;
            paddleOrientationQuat[3] = paddleOrientation.q3;
            calculationState &= ~PO_ALL_ORIENTATION_VALID;
        }

        SP_Math::Quaternion& getPaddleRelativeOrientation(){ 
            if (!(calculationState & PO_ORIENTATION_VALID)) updateRelativeOrientation(); 
            return paddleRelativeQuat;
        }

        void updatePaddleIMU(const IMUData& paddleIMU) {
            angularAccelerationCalculator.updateAngularVelocity(paddleIMU);
            angularVelocity[0] = paddleIMU.gx;
            angularVelocity[1] = paddleIMU.gy;
            angularVelocity[2] = paddleIMU.gz;
            calculationState &= ~PO_ANGULAR_ACCELERATION_VALID;
        };

        void getPaddleAngles(float& shaftRotationAngle, float& shaftTiltAngle, float& bladeRotationAngle);

        
        BladeSideType getLowerBladeSide() const;

        SP_Math::Quaternion& getPaddleModificationQuat() {
            if (!(calculationState & PO_MODIFICATION_VALID)) updateRelativeOrientation();
            return paddleModificationQuat;
        }

        SP_Math::Vector getAngularAcceleration(){
            if (!(calculationState & PO_ANGULAR_ACCELERATION_VALID)) {
                angularAcceleration = angularAccelerationCalculator.calculateAngularAcceleration();
                calculationState |= PO_ANGULAR_ACCELERATION_VALID;
            }
            return angularAcceleration;
        }

        SP_Math::Vector getAngularVelocity() const{
            return angularVelocity;
        }

};

#endif