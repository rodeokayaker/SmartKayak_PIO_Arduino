/**
 * @file SP_Protocol.h
 * @brief Определение протокола обмена сообщениями SmartPaddle
 */

#ifndef SP_PROTOCOL_H
#define SP_PROTOCOL_H

namespace SP_Protocol {
    /**
     * @brief Типы сообщений протокола
     */
    namespace MessageType {
        extern const char* LOG;        ///< Лог сообщение
        extern const char* COMMAND;    ///< Команда
        extern const char* RESPONSE;   ///< Ответ на команду
        extern const char* DATA;       ///< Данные датчиков
        extern const char* STATUS;     ///< Статус устройства
    }

    /**
     * @brief Поля JSON
     */
    namespace JsonFields {
        extern const char* TYPE;
        extern const char* DATA;
        extern const char* COMMAND;
        extern const char* PARAMS;
        extern const char* SUCCESS;
        extern const char* MESSAGE;
        extern const char* DATA_TYPE;
        extern const char* VALUE;
    }

    /**
     * @brief Команды протокола
     */
    namespace Commands {
        extern const char* CALIBRATE_COMPASS;      ///< Калибровка компаса
        extern const char* CALIBRATE_LOADS;        ///< Калибровка датчиков нагрузки
        extern const char* CALIBRATE_IMU;          ///< Калибровка IMU
        extern const char* CALIBRATE_BLADE_ANGLE;   ///< Калибровка угла лопасти
        extern const char* SEND_SPECS;             ///< Отправка спецификаций
        extern const char* START_PAIR;              ///< Старт пары
        extern const char* SHUTDOWN;                ///< Выключение
        extern const char* SET_MAGNETOMETER_CALIBRATION; ///< Калибровка магнитометра
        extern const char* SET_MAGNETOMETER_OFFSET; ///< Калибровка смещения магнитометра
        extern const char* TARE_LOADS;              ///< Калибровка нагрузки

        /**
         * @brief Параметры команд
         */
        namespace Params {
            extern const char* BLADE_SIDE;         ///< Сторона лопасти
            extern const char* OFFSET_X;           ///< X смещение магнитометра
            extern const char* OFFSET_Y;           ///< Y смещение магнитометра
            extern const char* OFFSET_Z;           ///< Z смещение магнитометра
            extern const char* SOFT_IRON_0_0;      ///< 0,0 коэффициент мягкого железа
            extern const char* SOFT_IRON_0_1;      ///< 0,1 коэффициент мягкого железа
            extern const char* SOFT_IRON_0_2;      ///< 0,2 коэффициент мягкого железа
            extern const char* SOFT_IRON_1_0;      ///< 1,0 коэффициент мягкого железа
            extern const char* SOFT_IRON_1_1;      ///< 1,1 коэффициент мягкого железа
            extern const char* SOFT_IRON_1_2;      ///< 1,2 коэффициент мягкого железа
            extern const char* SOFT_IRON_2_0;      ///< 2,0 коэффициент мягкого железа
            extern const char* SOFT_IRON_2_1;      ///< 2,1 коэффициент мягкого железа
            extern const char* SOFT_IRON_2_2;      ///< 2,2 коэффициент мягкого железа
        }

        /**
         * @brief Сообщения команд
         */
        namespace Messages {
            extern const char* SUCCESS;
            extern const char* ERROR;
        }
    }

    /**
     * @brief Типы данных протокола
     */
    namespace DataTypes {
        extern const char* SPECS;                ///< Спецификации
        extern const char* IMU;                   ///< Данные IMU
        extern const char* LOAD;                  ///< Данные нагрузки
        extern const char* ORIENTATION;           ///< Ориентация
        extern const char* BLADE_ORIENTATION;     ///< Ориентация лопасти
        extern const char* STATUS;                ///< Статус устройства
        extern const char* MAGNETOMETER_CALIBRATION; ///< Калибровка магнитометра
        extern const char* MAGNETOMETER_CALIBRATION_STATUS;
        extern const char* MAGNETOMETER_OFFSET;

        /**
         * @brief Поля для спецификаций
         */
        namespace Specs {
            extern const char* PADDLE_ID;
            extern const char* PADDLE_TYPE;
            extern const char* PADDLE_MODEL;
            extern const char* LENGTH;
            extern const char* IMU_DISTANCE;
            extern const char* IMU_FREQUENCY;
            extern const char* HAS_LEFT_BLADE;
            extern const char* HAS_RIGHT_BLADE;
            extern const char* FIRMWARE_VERSION;
        }

        /**
         * @brief Поля для данных ориентации лопасти
         */
        namespace BladeOrientation {
            extern const char* Y_AXIS_DIRECTION;
            extern const char* RIGHT_BLADE_ANGLE;
            extern const char* LEFT_BLADE_ANGLE;
            extern const char* RIGHT_BLADE_VECTOR_X;
            extern const char* RIGHT_BLADE_VECTOR_Y;
            extern const char* RIGHT_BLADE_VECTOR_Z;
            extern const char* LEFT_BLADE_VECTOR_X;
            extern const char* LEFT_BLADE_VECTOR_Y;
            extern const char* LEFT_BLADE_VECTOR_Z;
        }

        /**
         * @brief Поля для IMU данных
         */
        namespace IMUData {
            extern const char* ACCEL_X;
            extern const char* ACCEL_Y;
            extern const char* ACCEL_Z;
            extern const char* GYRO_X;
            extern const char* GYRO_Y;
            extern const char* GYRO_Z;
            extern const char* MAG_X;
            extern const char* MAG_Y;
            extern const char* MAG_Z;
            extern const char* QUAT_0;
            extern const char* QUAT_1;
            extern const char* QUAT_2;
            extern const char* QUAT_3;
            extern const char* MAG_X_ALT;  
            extern const char* MAG_Y_ALT;  
            extern const char* MAG_Z_ALT;  
            extern const char* TIMESTAMP;
        }

        /**
         * @brief Поля для данных нагрузки
         */
        namespace LoadData {
            extern const char* LEFT;
            extern const char* RIGHT;
            extern const char* LEFT_RAW;  
            extern const char* RIGHT_RAW;  
            extern const char* TIMESTAMP;
        }

        /**
         * @brief Поля для данных ориентации
         */
        namespace OrientationData {
            extern const char* QUAT_0;     
            extern const char* QUAT_1;     
            extern const char* QUAT_2;     
            extern const char* QUAT_3;     
            extern const char* TIMESTAMP;
        }

        /**
         * @brief Поля для данных батареи
         */
        namespace StatusData {
            extern const char* BATTERY;
            extern const char* TEMPERATURE;
        }

        /**
         * @brief Поля для калибровки магнитометра
         */
        namespace MagnetometerCalibration {
            extern const char* OFFSET_X;
            extern const char* OFFSET_Y;
            extern const char* OFFSET_Z;
            extern const char* SOFT_IRON_0_0;
            extern const char* SOFT_IRON_0_1;
            extern const char* SOFT_IRON_0_2;
            extern const char* SOFT_IRON_1_0;
            extern const char* SOFT_IRON_1_1;
            extern const char* SOFT_IRON_1_2;
            extern const char* SOFT_IRON_2_0;
            extern const char* SOFT_IRON_2_1;
            extern const char* SOFT_IRON_2_2;
            extern const char* STATUS;
        }
    }

    /**
     * @brief Уровни логирования
     */
    namespace LogLevels {
        extern const char* DEBUG;
        extern const char* INFO;
        extern const char* WARNING;
        extern const char* ERROR;
        extern const char* FATAL;
    }

    /**
     * @brief Статусы
     */
    namespace Status {
        extern const char* DEVICE_ID;
        extern const char* FIRMWARE_VERSION;
        extern const char* HARDWARE_VERSION;
        extern const char* BATTERY_LEVEL;
        extern const char* TEMPERATURE;
        extern const char* LOGGING_ACTIVE;
        extern const char* LOGGING_INTERVAL;
        extern const char* FILTER_TYPE;
        extern const char* FILTER_CUTOFF;
        extern const char* FILTER_ORDER;
        extern const char* BLADE_SIDE;
        extern const char* CALIBRATION_STATUS;
    }
}

#endif // SP_PROTOCOL_H 