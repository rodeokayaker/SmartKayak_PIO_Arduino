/**
 * @file SP_CoordinateTransform.h
 * @brief Преобразование координат IMU для различных ориентаций датчика
 * 
 * Позволяет работать с данными IMU так, как будто датчик всегда ориентирован
 * осью Y вдоль шафта, независимо от реальной ориентации.
 */

#pragma once
#include "../Core/Types.h"

/**
 * @brief Класс для преобразования координат IMU в зависимости от ориентации датчика
 */
class SP_CoordinateTransform {
public:
    /**
     * @brief Преобразовать данные IMU в стандартную систему координат (Y вдоль шафта)
     * @param data Ссылка на данные IMU (изменяются на месте)
     * @param axisDirection Реальная ориентация датчика
     * 
     * После преобразования можно работать с данными так, как будто ось Y всегда вдоль шафта.
     */
    static void transformIMUData(IMUData& data, AxisDirection axisDirection) {
        switch (axisDirection) {
            case X_AXIS_RIGHT:
            case X_AXIS_LEFT:
                // Ось X вдоль шафта → поворот на 90° вокруг Z
                rotateAroundZ_90deg(data);
                
                // Если направление влево, инвертируем ось вдоль шафта
//                if (axisDirection == X_AXIS_LEFT) {
//                    invertYAxis(data);
//                }
                break;
                
            case Z_AXIS_RIGHT:
            case Z_AXIS_LEFT:
                // Ось Z вдоль шафта → поворот на 90° вокруг X
                rotateAroundX_90deg(data);
                
                // Если направление влево, инвертируем ось вдоль шафта
//                if (axisDirection == Z_AXIS_LEFT) {
//                    invertYAxis(data);
//                }
                break;
                
            case Y_AXIS_LEFT:
                // Ось Y вдоль шафта влево → инвертируем Y
 //                invertYAxis(data);
                break;
                
            case Y_AXIS_RIGHT:
            default:
                // Уже в правильной ориентации
                break;
        }
    }

    /**
     * @brief Преобразовать данные ориентации (кватернион) в стандартную систему координат
     * @param data Ссылка на данные ориентации (изменяются на месте)
     * @param axisDirection Реальная ориентация датчика
     * 
     * Аналогично transformIMUData, но только для кватерниона ориентации.
     */
    static void transformOrientationData(OrientationData& data, AxisDirection axisDirection) {
        switch (axisDirection) {
            case X_AXIS_RIGHT:
            case X_AXIS_LEFT:
                // Ось X вдоль шафта → поворот на 90° вокруг Z
                rotateQuaternionAroundZ_90deg(data.q0, data.q1, data.q2, data.q3);
                
                // Если направление влево, инвертируем ось вдоль шафта
//                if (axisDirection == X_AXIS_LEFT) {
//                    invertQuaternionYAxis(data.q0, data.q1, data.q2, data.q3);
//                }
                break;
                
            case Z_AXIS_RIGHT:
            case Z_AXIS_LEFT:
                // Ось Z вдоль шафта → поворот на 90° вокруг X
                rotateQuaternionAroundX_90deg(data.q0, data.q1, data.q2, data.q3);
                
                // Если направление влево, инвертируем ось вдоль шафта
//                if (axisDirection == Z_AXIS_LEFT) {
//                    invertQuaternionYAxis(data.q0, data.q1, data.q2, data.q3);
//                }
                break;
                
            case Y_AXIS_LEFT:
                // Ось Y вдоль шафта влево → инвертируем Y
                // invertQuaternionYAxis(data.q0, data.q1, data.q2, data.q3);
                break;
                
            case Y_AXIS_RIGHT:
            default:
                // Уже в правильной ориентации
                break;
        }
    }

    /**
     * @brief Преобразовать данные ориентации лопастей в стандартную систему координат
     * @param data Ссылка на данные ориентации лопастей (изменяются на месте)
     * @param axisDirection Реальная ориентация датчика
     * 
     * Преобразует векторы нормали к лопастям в стандартную систему координат.
     */
    static void transformBladeOrientation(BladeOrientation& data, AxisDirection axisDirection) {
        switch (axisDirection) {
            case X_AXIS_RIGHT:
            case X_AXIS_LEFT:
                // Ось X вдоль шафта → поворот на 90° вокруг Z
                rotateVectorAroundZ_90deg(data.rightBladeVector);
                rotateVectorAroundZ_90deg(data.leftBladeVector);
                
                // Если направление влево, инвертируем ось вдоль шафта
//                if (axisDirection == X_AXIS_LEFT) {
//                    invertVectorYAxis(data.rightBladeVector);
//                    invertVectorYAxis(data.leftBladeVector);
//                }
                break;
                
            case Z_AXIS_RIGHT:
            case Z_AXIS_LEFT:
                // Ось Z вдоль шафта → поворот на 90° вокруг X
                rotateVectorAroundX_90deg(data.rightBladeVector);
                rotateVectorAroundX_90deg(data.leftBladeVector);
                
                // Если направление влево, инвертируем ось вдоль шафта
//                if (axisDirection == Z_AXIS_LEFT) {
//                    invertVectorYAxis(data.rightBladeVector);
//                    invertVectorYAxis(data.leftBladeVector);
//                }
                break;
                
            case Y_AXIS_LEFT:
                // Ось Y вдоль шафта влево → инвертируем Y
                // invertVectorYAxis(data.rightBladeVector);
                // invertVectorYAxis(data.leftBladeVector);
                break;
                
            case Y_AXIS_RIGHT:
            default:
                // Уже в правильной ориентации
                break;
        }
    }

private:
    /**
     * @brief Поворот на 90° вокруг оси Z (для X_AXIS → Y_AXIS)
     * Преобразование: новый_x = -старый_y, новый_y = старый_x, новый_z = старый_z
     */
    static void rotateAroundZ_90deg(IMUData& data) {
        // Акселерометр
        float temp = data.ax;
        data.ax = -data.ay;
        data.ay = temp;
        // data.az без изменений
        
        // Гироскоп
        temp = data.gx;
        data.gx = -data.gy;
        data.gy = temp;
        // data.gz без изменений
        
        // Магнитометр (float)
        temp = data.mx;
        data.mx = -data.my;
        data.my = temp;
        // data.mz без изменений
        
        // Магнитометр (int16_t)
        int16_t temp_int = data.mag_x;
        data.mag_x = -data.mag_y;
        data.mag_y = temp_int;
        // data.mag_z без изменений
        
        // Кватернион: умножение на q_rotation = [cos(45°), 0, 0, sin(45°)]
        // Для поворота на 90° вокруг Z
        float q0_old = data.q0;
        float q1_old = data.q1;
        float q2_old = data.q2;
        float q3_old = data.q3;
        
        const float cos45 = 0.7071067811865476f;  // cos(45°) = sin(45°) = √2/2
        
        data.q0 = cos45 * q0_old - cos45 * q3_old;
        data.q1 = cos45 * q1_old + cos45 * q2_old;
        data.q2 = cos45 * q2_old - cos45 * q1_old;
        data.q3 = cos45 * q3_old + cos45 * q0_old;
    }
    
    /**
     * @brief Поворот на 90° вокруг оси X (для Z_AXIS → Y_AXIS)
     * Преобразование: новый_x = старый_x, новый_y = -старый_z, новый_z = старый_y
     */
    static void rotateAroundX_90deg(IMUData& data) {
        // Акселерометр
        float temp = data.ay;
        data.ay = -data.az;
        data.az = temp;
        // data.ax без изменений
        
        // Гироскоп
        temp = data.gy;
        data.gy = -data.gz;
        data.gz = temp;
        // data.gx без изменений
        
        // Магнитометр (float)
        temp = data.my;
        data.my = -data.mz;
        data.mz = temp;
        // data.mx без изменений
        
        // Магнитометр (int16_t)
        int16_t temp_int = data.mag_y;
        data.mag_y = -data.mag_z;
        data.mag_z = temp_int;
        // data.mag_x без изменений
        
        // Кватернион: умножение на q_rotation = [cos(45°), sin(45°), 0, 0]
        // Для поворота на 90° вокруг X
        float q0_old = data.q0;
        float q1_old = data.q1;
        float q2_old = data.q2;
        float q3_old = data.q3;
        
        const float cos45 = 0.7071067811865476f;
        
        data.q0 = cos45 * q0_old - cos45 * q1_old;
        data.q1 = cos45 * q1_old + cos45 * q0_old;
        data.q2 = cos45 * q2_old + cos45 * q3_old;
        data.q3 = cos45 * q3_old - cos45 * q2_old;
    }
    
    /**
     * @brief Инвертировать ось Y (для LEFT направлений)
     * Преобразование: новый_y = -старый_y
     */
    static void invertYAxis(IMUData& data) {
        // Инвертируем ось Y для всех датчиков
        data.ay = -data.ay;
        data.gy = -data.gy;
        data.my = -data.my;
        data.mag_y = -data.mag_y;
        
        // Для кватерниона: инвертируем компоненту вращения вокруг Y
        data.q2 = -data.q2;
    }
    
    // ============================================
    // Вспомогательные методы для кватернионов
    // ============================================
    
    /**
     * @brief Поворот кватерниона на 90° вокруг оси Z
     */
    static void rotateQuaternionAroundZ_90deg(float& q0, float& q1, float& q2, float& q3) {
        float q0_old = q0;
        float q1_old = q1;
        float q2_old = q2;
        float q3_old = q3;
        
        const float cos45 = 0.7071067811865476f;  // cos(45°) = sin(45°)
        
        q0 = cos45 * q0_old - cos45 * q3_old;
        q1 = cos45 * q1_old + cos45 * q2_old;
        q2 = cos45 * q2_old - cos45 * q1_old;
        q3 = cos45 * q3_old + cos45 * q0_old;
    }
    
    /**
     * @brief Поворот кватерниона на 90° вокруг оси X
     */
    static void rotateQuaternionAroundX_90deg(float& q0, float& q1, float& q2, float& q3) {
        float q0_old = q0;
        float q1_old = q1;
        float q2_old = q2;
        float q3_old = q3;
        
        const float cos45 = 0.7071067811865476f;
        
        q0 = cos45 * q0_old - cos45 * q1_old;
        q1 = cos45 * q1_old + cos45 * q0_old;
        q2 = cos45 * q2_old + cos45 * q3_old;
        q3 = cos45 * q3_old - cos45 * q2_old;
    }
    
    /**
     * @brief Инвертировать ось Y в кватернионе
     */
    static void invertQuaternionYAxis(float& q0, float& q1, float& q2, float& q3) {
        // Инвертируем компоненту вращения вокруг Y
        q2 = -q2;
    }
    
    // ============================================
    // Вспомогательные методы для векторов
    // ============================================
    
    /**
     * @brief Поворот вектора на 90° вокруг оси Z
     * @param vec Массив из 3 элементов [x, y, z]
     */
    static void rotateVectorAroundZ_90deg(float* vec) {
        float temp = vec[0];
        vec[0] = -vec[1];  // новый_x = -старый_y
        vec[1] = temp;     // новый_y = старый_x
        // vec[2] без изменений
    }
    
    /**
     * @brief Поворот вектора на 90° вокруг оси X
     * @param vec Массив из 3 элементов [x, y, z]
     */
    static void rotateVectorAroundX_90deg(float* vec) {
        float temp = vec[1];
        vec[1] = -vec[2];  // новый_y = -старый_z
        vec[2] = temp;     // новый_z = старый_y
        // vec[0] без изменений
    }
    
    /**
     * @brief Инвертировать ось Y в векторе
     * @param vec Массив из 3 элементов [x, y, z]
     */
    static void invertVectorYAxis(float* vec) {
        vec[1] = -vec[1];  // новый_y = -старый_y
    }
};
