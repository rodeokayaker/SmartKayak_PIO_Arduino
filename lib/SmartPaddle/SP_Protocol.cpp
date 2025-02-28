#include "SP_Protocol.h"

namespace SP_Protocol {
    namespace MessageType {
        const char* LOG = "log";
        const char* COMMAND = "cmd";
        const char* RESPONSE = "response";
        const char* DATA = "data";
        const char* STATUS = "status";
    }

    namespace Commands {
        const char* CALIBRATE_COMPASS = "calibrate_compass";
        const char* CALIBRATE_LOADS = "calibrate_loads";
        const char* CALIBRATE_IMU = "calibrate_imu";
        const char* CALIBRATE_BLADE_ANGLE = "calibrate_blade_angle";
        const char* SEND_SPECS = "send_specs";
        const char* START_PAIR = "start_pair";
        const char* SHUTDOWN = "shutdown";
        const char* TARE_LOADS = "tare_loads";

        namespace Params {
            const char* BLADE_SIDE = "blade_side";
        }
        
        namespace Messages {
            const char* SUCCESS = "Success";
            const char* ERROR = "Error";
        }
    }

    namespace DataTypes {
        const char* SPECS = "specs";
        const char* IMU = "imu";
        const char* LOAD = "load";
        const char* ORIENTATION = "orientation";
        const char* BLADE_ORIENTATION = "blade_orientation";
        const char* STATUS = "status";

        namespace Specs {
            const char* PADDLE_ID = "paddleID";
            const char* PADDLE_TYPE = "paddleType";
            const char* PADDLE_MODEL = "paddleModel";
            const char* BLADE_POWER = "bladePower";
            const char* LENGTH = "length";
            const char* IMU_FREQUENCY = "imuFrequency";
            const char* HAS_LEFT_BLADE = "hasLeftBlade";
            const char* HAS_RIGHT_BLADE = "hasRightBlade";
            const char* FIRMWARE_VERSION = "firmwareVersion";
        }

        namespace BladeOrientation {
            const char* Y_AXIS_DIRECTION = "yAxisDirection";
            const char* RIGHT_BLADE_ANGLE = "rightBladeAngle";
            const char* LEFT_BLADE_ANGLE = "leftBladeAngle";
            const char* RIGHT_BLADE_VECTOR_X = "rightBladeVectorX";
            const char* RIGHT_BLADE_VECTOR_Y = "rightBladeVectorY";
            const char* RIGHT_BLADE_VECTOR_Z = "rightBladeVectorZ";
            const char* LEFT_BLADE_VECTOR_X = "leftBladeVectorX";
            const char* LEFT_BLADE_VECTOR_Y = "leftBladeVectorY";
            const char* LEFT_BLADE_VECTOR_Z = "leftBladeVectorZ";
        }
        
        namespace IMUData {
            const char* ACCEL_X = "ax";
            const char* ACCEL_Y = "ay";
            const char* ACCEL_Z = "az";
            const char* GYRO_X = "gx";
            const char* GYRO_Y = "gy";
            const char* GYRO_Z = "gz";
            const char* MAG_X = "mx";
            const char* MAG_Y = "my";
            const char* MAG_Z = "mz";
            const char* QUAT_0 = "q0";
            const char* QUAT_1 = "q1";
            const char* QUAT_2 = "q2";
            const char* QUAT_3 = "q3";
            const char* MAG_X_ALT = "magX";
            const char* MAG_Y_ALT = "magY";
            const char* MAG_Z_ALT = "magZ";
            const char* TIMESTAMP = "ts";
        }
        
        namespace LoadData {
            const char* LEFT = "left";
            const char* RIGHT = "right";
            const char* LEFT_RAW = "left_raw";
            const char* RIGHT_RAW = "right_raw";
            const char* TIMESTAMP = "ts";
        }
        
        namespace OrientationData {
            const char* QUAT_0 = "q0";
            const char* QUAT_1 = "q1";
            const char* QUAT_2 = "q2";
            const char* QUAT_3 = "q3";
            const char* TIMESTAMP = "ts";
        }
        
        namespace StatusData {
            const char* BATTERY = "battery";
            const char* TEMPERATURE = "temperature";
        }
    }
    
    namespace JsonFields {
        const char* TYPE = "type";
        const char* DATA = "data";
        const char* COMMAND = "cmd";
        const char* PARAMS = "params";
        const char* SUCCESS = "success";
        const char* MESSAGE = "msg";
        const char* DATA_TYPE = "dataType";
        const char* VALUE = "value";
    }
} 