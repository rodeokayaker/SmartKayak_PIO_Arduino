/**
 * @file SP_CommandParams.h
 * @brief Вспомогательный класс для удобного доступа к параметрам команд
 */

#pragma once
#include "SP_Message.h"
#include "SP_Protocol.h"
#include "../Core/Types.h"

/**
 * @brief Класс для типобезопасного извлечения параметров из команд
 * 
 * Упрощает работу с параметрами команд, предоставляя удобные методы
 * для извлечения значений с поддержкой значений по умолчанию.
 */
class SP_CommandParams {
private:
    JsonObject& params;
    
public:
    /**
     * @brief Конструктор
     * @param cmd Указатель на команду
     */
    SP_CommandParams(SP_Command* cmd) : params(cmd->params) {}
    
    /**
     * @brief Получить параметр с типом и значением по умолчанию
     * @tparam T Тип параметра
     * @param name Имя параметра
     * @param defaultValue Значение по умолчанию
     * @return Значение параметра или значение по умолчанию
     */
    template<typename T>
    T get(const char* name, T defaultValue = T()) const {
        return params[name].is<T>() ? params[name].as<T>() : defaultValue;
    }
    
    /**
     * @brief Проверить наличие параметра
     * @param name Имя параметра
     * @return true если параметр существует
     */
    bool has(const char* name) const {
        return !params[name].isNull();
    }
    
    /**
     * @brief Получить сторону лопасти
     * @param defaultSide Значение по умолчанию
     * @return Сторона лопасти
     */
    BladeSideType getBladeSide(BladeSideType defaultSide = ALL_BLADES) const {
        return (BladeSideType)get<int>(
            SP_Protocol::Commands::Params::BLADE_SIDE, 
            (int)defaultSide
        );
    }
    
    /**
     * @brief Извлечь смещение магнитометра
     * @param offset Массив из 3 элементов для X, Y, Z
     */
    void getMagnetometerOffset(float* offset) const {
        offset[0] = get<float>(SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_X, 0.0f);
        offset[1] = get<float>(SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_Y, 0.0f);
        offset[2] = get<float>(SP_Protocol::DataTypes::MagnetometerCalibration::OFFSET_Z, 0.0f);
    }
    
    /**
     * @brief Извлечь матрицу soft iron калибровки магнитометра
     * @param softIron Массив из 6 элементов для диагонали и недиагональных элементов
     */
    void getMagnetometerSoftIron(float* softIron) const {
        // Диагональные элементы (масштаб)
        softIron[0] = get<float>(SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_0_0, 1.0f);
        softIron[1] = get<float>(SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_1_1, 1.0f);
        softIron[2] = get<float>(SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_2_2, 1.0f);
        // Недиагональные элементы
        softIron[3] = get<float>(SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_0_1, 0.0f);
        softIron[4] = get<float>(SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_0_2, 0.0f);
        softIron[5] = get<float>(SP_Protocol::DataTypes::MagnetometerCalibration::SOFT_IRON_1_2, 0.0f);
    }
    
    /**
     * @brief Получить полную калибровку магнитометра
     * @param offset Массив из 3 элементов для смещения
     * @param softIron Массив из 6 элементов для soft iron
     */
    void getMagnetometerCalibration(float* offset, float* softIron) const {
        getMagnetometerOffset(offset);
        getMagnetometerSoftIron(softIron);
    }
    
    /**
     * @brief Прямой доступ к JsonObject параметров
     * @return Ссылка на JsonObject
     */
    JsonObject& getJsonObject() {
        return params;
    }
};

/**
 * @brief Класс для удобного доступа к данным в SP_Data
 */
class SP_DataValue {
private:
    JsonObject& value;
    
public:
    /**
     * @brief Конструктор
     * @param data Указатель на данные
     */
    SP_DataValue(SP_Data* data) : value(data->value) {}
    
    /**
     * @brief Получить значение с типом и значением по умолчанию
     */
    template<typename T>
    T get(const char* name, T defaultValue = T()) const {
        return value[name].is<T>() ? value[name].as<T>() : defaultValue;
    }
    
    /**
     * @brief Проверить наличие поля
     */
    bool has(const char* name) const {
        return !value[name].isNull();
    }
    
    /**
     * @brief Прямой доступ к JsonObject значения
     */
    JsonObject& getJsonObject() {
        return value;
    }
};


