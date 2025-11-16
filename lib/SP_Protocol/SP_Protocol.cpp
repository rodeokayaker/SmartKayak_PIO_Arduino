#include "SP_Protocol.h"

namespace SP_Protocol {
    namespace MessageType {
        const char* TITLE = "type";
        const char* LOG = "log";
        const char* COMMAND = "cmd";
        const char* RESPONSE = "response";
        const char* DATA = "data";
        const char* STATUS = "status";
    }

    namespace Commands {
        const char* TITLE = "cmd";
        const char* CALIBRATE_COMPASS = "calibrate_compass";
        const char* CALIBRATE_LOADS = "calibrate_loads";
        const char* CALIBRATE_IMU = "calibrate_imu";
        const char* CALIBRATE_BLADE_ANGLE = "calibrate_blade_angle";
        const char* SEND_SPECS = "send_specs";
        const char* SEND_PADDLE_ORIENTATION = "send_paddle_orientation";
        const char* START_PAIR = "start_pair";
        const char* SHUTDOWN = "shutdown";
        const char* TARE_LOADS = "tare_loads";
        const char* SET_MAGNETOMETER_CALIBRATION = "set_m_calib";
        const char* SET_MAGNETOMETER_OFFSET = "set_m_offset";
        const char* SET_SPECS = "set_specs";
        const char* SEND_CALIBRATION_DATA = "send_calibration_data";
        namespace Params {
            const char* TITLE = "params";
            const char* BLADE_SIDE = "blade_side";
        }
        
        namespace Responses {
            const char* TITLE = "success";
            const char* SUCCESS = "Success";
            const char* ERROR = "Error";
            const char* MESSAGE = "msg";
        }
    }

    namespace DataTypes {
        const char* TITLE = "dataType";
        const char* VALUE = "value";
        const char* SPECS = "specs";
        const char* BLADE_ORIENTATION = "blade_orientation";
        const char* STATUS = "status";
        const char* MAGNETOMETER_CALIBRATION = "m_calib";
        const char* MAGNETOMETER_OFFSET = "m_offset";
        const char* MAGNETOMETER_CALIBRATION_STATUS = "m_calib_status";


        namespace Specs {

            const char* PADDLE_ID = "paddleID";
            const char* PADDLE_TYPE = "paddleType";
            const char* PADDLE_MODEL = "paddleModel";
            const char* LENGTH = "length";
            const char* BLADE_WEIGHT = "bWei";
            const char* BLADE_CENTER = "bCen";
            const char* BLADE_MOMENT_INERTIA = "bMIn";
            const char* IMU_FREQUENCY = "imuFrequency";
            const char* IMU_DISTANCE = "imuDistance";
            const char* HAS_LEFT_BLADE = "hasLeftBlade";
            const char* HAS_RIGHT_BLADE = "hasRightBlade";
            const char* FIRMWARE_VERSION = "firmwareVersion";
            const char* AXIS_DIRECTION = "axisDirection";
        }

        namespace BladeOrientation {
            const char* RIGHT_BLADE_ANGLE = "rightBladeAngle";
            const char* LEFT_BLADE_ANGLE = "leftBladeAngle";
            const char* RIGHT_BLADE_VECTOR_X = "rightBladeVectorX";
            const char* RIGHT_BLADE_VECTOR_Y = "rightBladeVectorY";
            const char* RIGHT_BLADE_VECTOR_Z = "rightBladeVectorZ";
            const char* LEFT_BLADE_VECTOR_X = "leftBladeVectorX";
            const char* LEFT_BLADE_VECTOR_Y = "leftBladeVectorY";
            const char* LEFT_BLADE_VECTOR_Z = "leftBladeVectorZ";
        }
        
        
        namespace StatusData {
            const char* BATTERY = "battery";
            const char* TEMPERATURE = "temperature";
        }

        namespace MagnetometerCalibration {
            const char* OFFSET_X = "ox";
            const char* OFFSET_Y = "oy";
            const char* OFFSET_Z = "oz";
            const char* SOFT_IRON_0_0 = "si00";
            const char* SOFT_IRON_0_1 = "si01";
            const char* SOFT_IRON_0_2 = "si02";
            const char* SOFT_IRON_1_0 = "si10";
            const char* SOFT_IRON_1_1 = "si11";
            const char* SOFT_IRON_1_2 = "si12";
            const char* SOFT_IRON_2_0 = "si20";
            const char* SOFT_IRON_2_1 = "si21";
            const char* SOFT_IRON_2_2 = "si22";
            const char* STATUS = "sts";
        }
    }

    namespace Status {
        const char* TITLE = "status";
        const char* DEVICE_ID = "deviceID";
        const char* FIRMWARE_VERSION = "firmwareVersion";
        const char* HARDWARE_VERSION = "hardwareVersion";
        const char* BATTERY_LEVEL = "batteryLevel";
        const char* TEMPERATURE = "temperature";
        const char* LOGGING_ACTIVE = "loggingActive";
        const char* LOGGING_INTERVAL = "loggingInterval";
        const char* FILTER_TYPE = "filterType";
        const char* FILTER_CUTOFF = "filterCutoff";
        const char* FILTER_ORDER = "filterOrder";
        const char* BLADE_SIDE = "bladeSide";
        const char* CALIBRATION_STATUS = "calibrationStatus";
    }

    namespace LogMessages {
        const char* MESSAGE = "msg";
    }
    
} 