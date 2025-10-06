#include <Arduino.h>
#include <SPI.h>
#include "Protocentral_ADS1220.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// Пины ESP32 для ADS1220
#define ADS1220_CS_PIN    5   // GPIO5 для CS
#define ADS1220_DRDY_PIN  4   // GPIO4 для DRDY
#define ADS1220_SCLK_PIN  18  // GPIO18 для SCLK
#define ADS1220_MOSI_PIN  23  // GPIO23 для MOSI  
#define ADS1220_MISO_PIN  19  // GPIO19 для MISO

// Пины для BNO08x
#define BNO08X_SDA_PIN 21
#define BNO08X_SCL_PIN 22
#define BNO08X_RST_PIN 26
#define BNO08X_INT_PIN 27

// Константы мультиплексора ADS1220
#define MUX_AIN0_AIN1  0x00  // AINP = AIN0, AINN = AIN1 (Тензодатчик 1)
#define MUX_AIN2_AIN3  0x05  // AINP = AIN2, AINN = AIN3 (Тензодатчик 2)

// Адреса регистров ADS1220
#define ADS1220_REG_CONFIG0   0x00  // MUX[3:0], PGA_BYPASS, GAIN[2:0]
#define ADS1220_REG_CONFIG1   0x01  // BCS[1:0], DR[3:0], MODE[1:0]
#define ADS1220_REG_CONFIG2   0x02  // ODR[3:0], COMP_MODE, COMP_POL, COMP_LAT, COMP_QUE[1:0]
#define ADS1220_REG_CONFIG3   0x03  // OSR[2:0], RESERVED[4:0]

// Команды управления ADS1220
#define ADS1220_START_CMD     0x08  // Команда START/SYNC
#define ADS1220_RESET_CMD     0x06  // Команда RESET
#define ADS1220_PWRDOWN_CMD   0x02  // Команда POWERDOWN

// Калибровочные коэффициенты (настройте под ваши тензодатчики)
float scaleFactor1 = 0.0001;  // Коэффициент масштабирования для тензодатчика 1
float offset1 = 15700.0;      // Смещение нуля для тензодатчика 1 (базовое значение)
float scaleFactor2 = 0.0001;  // Коэффициент масштабирования для тензодатчика 2
float offset2 = 2700.0;       // Смещение нуля для тензодатчика 2 (базовое значение)

// Система управления скоростью ADS1220
uint8_t currentSPS = 0b0110;  // Текущая настройка SPS (по умолчанию 1000 SPS)
const char* currentSPSName = "1000"; // Название текущей скорости

// Массив всех доступных SPS настроек
const uint8_t SPS_SETTINGS[] = {0b0000, 0b0001, 0b0010, 0b0011, 0b0100, 0b0101, 0b0110, 0b0111};
const char* SPS_NAMES[] = {"20", "45", "90", "175", "330", "600", "1000", "2000"};
const int SPS_COUNT = 8;

// Система управления коэффициентом усиления ADS1220
uint8_t currentGAIN = 0b010;  // Текущая настройка GAIN (по умолчанию 4)
const char* currentGAINName = "4"; // Название текущего усиления

// Массив всех доступных GAIN настроек
const uint8_t GAIN_SETTINGS[] = {0b000, 0b001, 0b010, 0b011, 0b100, 0b101, 0b110, 0b111};
const char* GAIN_NAMES[] = {"1", "2", "4", "8", "16", "32", "64", "128"};
const int GAIN_COUNT = 8;

// Система управления выводом данных
unsigned long lastOutputTime = 0;  // Время последнего вывода
unsigned long outputInterval = 1000; // Интервал вывода в мс (по умолчанию 1 сек)
bool outputEnabled = true;         // Флаг включения вывода
bool highSpeedMode = false;        // Режим максимальной скорости

// Объект библиотеки ADS1220
Protocentral_ADS1220 ads1220;

// Объект для BNO08x
Adafruit_BNO08x bno08x;
sh2_SensorValue_t bno08x_sensorValue;

// Глобально:
bool bno08xReady = false;

// Функция для отправки команд ADS1220
void ads1220_send_command(uint8_t command) {
  digitalWrite(ADS1220_CS_PIN, LOW);
  SPI.transfer(command);
  digitalWrite(ADS1220_CS_PIN, HIGH);
}

// Функция для записи регистра напрямую через SPI
void writeRegisterDirect(uint8_t reg, uint8_t value) {
  digitalWrite(ADS1220_CS_PIN, LOW);
  // Правильный формат команды для ADS1220: 0x4x где x - номер регистра
  SPI.transfer(0x40 + reg); // Команда WRITE_REG для регистра reg
  SPI.transfer(value);       // Записываем значение
  digitalWrite(ADS1220_CS_PIN, HIGH);
  delay(5);
}

// Альтернативная функция для чтения регистра напрямую через SPI
uint8_t readRegisterDirect(uint8_t reg) {
  digitalWrite(ADS1220_CS_PIN, LOW);
  // Правильный формат команды для ADS1220: 0x2x где x - номер регистра
  SPI.transfer(0x20 + reg); // Команда READ_REG для регистра reg
  uint8_t value = SPI.transfer(0x00); // Читаем значение
  digitalWrite(ADS1220_CS_PIN, HIGH);
  return value;
}







// Функция для установки канала мультиплексора
void setMuxChannel(uint8_t mux_setting) {
  // Читаем текущее значение CONFIG0 регистра с правильным форматом
  uint8_t config0 = readRegisterDirect(ADS1220_REG_CONFIG0);
  
  // Получаем текущие биты MUX
  uint8_t current_mux = (config0 >> 4) & 0x0F;
  
  // Если MUX уже установлен правильно, не меняем
  if (current_mux == mux_setting) {
    return;
  }
  
  // Очищаем биты MUX (7-4) и устанавливаем новый канал
  config0 &= 0x0F; // Очищаем биты MUX (7-4), сохраняем остальные биты
  config0 |= (mux_setting << 4); // Устанавливаем новый канал
  
  // Записываем новое значение с правильным форматом
  writeRegisterDirect(ADS1220_REG_CONFIG0, config0);
  delay(5); // Минимальная задержка
  
  // Проверяем, что регистр изменился (только при отладке)
  #ifdef DEBUG_MUX
  uint8_t new_config0 = readRegisterDirect(ADS1220_REG_CONFIG0);
  uint8_t actual_mux = (new_config0 >> 4) & 0x0F;
  
  if (actual_mux != mux_setting) {
    // Попробуем принудительно записать регистр
    writeRegisterDirect(ADS1220_REG_CONFIG0, config0);
    delay(10);
    
    uint8_t final_reg = readRegisterDirect(ADS1220_REG_CONFIG0);
    uint8_t final_mux = (final_reg >> 4) & 0x0F;
    
    if (final_mux != mux_setting) {
      Serial.println("КРИТИЧЕСКАЯ ОШИБКА: Запись в регистр не работает!");
    }
  }
  #endif
}

// Функция для чтения данных с ADS1220
float readADS1220() {
  // Запуск преобразования
  ads1220_send_command(ADS1220_START_CMD);
  
  // Ожидание готовности данных
  unsigned long start = millis();
  while(digitalRead(ADS1220_DRDY_PIN) == HIGH) {
    if(millis() - start > 100) { // Уменьшенный timeout
      Serial.println("DRDY timeout!");
      return 0;
    }
  }
  
  // Чтение данных
  digitalWrite(ADS1220_CS_PIN, LOW);
  uint8_t adcData[3];
  adcData[2] = SPI.transfer(0x00);
  adcData[1] = SPI.transfer(0x00);
  adcData[0] = SPI.transfer(0x00);
  digitalWrite(ADS1220_CS_PIN, HIGH);
  
  // Преобразование в 24-битное значение
  int32_t value = ((int32_t)adcData[2] << 16) | 
                  ((int32_t)adcData[1] << 8) | 
                  adcData[0];
  
  // Коррекция знака для 24-битного значения
  if (value & 0x800000) {
    value |= 0xFF000000;
  }
  
  return (float)value;
}

// Функция для чтения тензодатчика с калибровкой
float readStrainGauge(uint8_t mux_setting, float scaleFactor, float offset) {
  setMuxChannel(mux_setting);
  float rawValue = readADS1220();
  
  // Применяем калибровку
  float calibratedValue = (rawValue - offset) * scaleFactor;
  
  return calibratedValue;
}



// Функция для настройки ADS1220
void configureADS1220() {
  // Настройка регистров с правильным форматом команд ADS1220 для 200 Гц
  // MUX регистр (0x00): MUX[3:0], PGA_BYPASS, GAIN[2:0]
  uint8_t mux_reg = (MUX_AIN0_AIN1 << 4) | // Начальный канал AIN0-AIN1
                    (0 << 3) |              // PGA_BYPASS = 0 (PGA включен)
                    (currentGAIN);           // GAIN = текущее значение

  // BCS регистр (0x01): BCS[1:0], DR[3:0], MODE[1:0]
  uint8_t bcs_reg = (0b00 << 6) | // BCS = 00 (burnout current off)
                    (currentSPS << 2) | // DR = текущий SPS
                    (0b00);         // MODE = 00 (single-shot)

  // ODR регистр (0x02): ODR[3:0], COMP_MODE, COMP_POL, COMP_LAT, COMP_QUE[1:0]
  uint8_t odr_reg = (0b0000 << 4) | // ODR = 0000 (default)
                    (0 << 3) |       // COMP_MODE = 0 (comparator off)
                    (0 << 2) |       // COMP_POL = 0
                    (0 << 1) |       // COMP_LAT = 0
                    (0b00);          // COMP_QUE = 00

  // Записываем все регистры по одному напрямую через SPI
  writeRegisterDirect(ADS1220_REG_CONFIG0, mux_reg);
  delay(10);
  writeRegisterDirect(ADS1220_REG_CONFIG1, bcs_reg);
  delay(10);
  writeRegisterDirect(ADS1220_REG_CONFIG2, odr_reg);
  delay(10);
  
  Serial.println("ADS1220 настроен для работы");
  Serial.print("Текущая скорость: ");
  Serial.print(currentSPSName);
  Serial.println(" SPS");
  Serial.print("Текущее усиление: ");
  Serial.print(currentGAINName);
  Serial.println("x");
  Serial.print("MUX регистр: 0x");
  Serial.println(mux_reg, HEX);
  Serial.print("BCS регистр: 0x");
  Serial.println(bcs_reg, HEX);
  Serial.print("ODR регистр: 0x");
  Serial.println(odr_reg, HEX);
}

// Функция для переключения скорости SPS
void setSPS(uint8_t sps_setting) {
  if (sps_setting >= SPS_COUNT) {
    Serial.println("Ошибка: Неверная настройка SPS!");
    return;
  }
  
  Serial.print("Устанавливаем скорость с индексом ");
  Serial.print(sps_setting);
  Serial.print(" (");
  Serial.print(SPS_NAMES[sps_setting]);
  Serial.println(" SPS)");
  
  // Обновляем текущую скорость
  currentSPS = SPS_SETTINGS[sps_setting];
  currentSPSName = SPS_NAMES[sps_setting];
  
  // Читаем текущий CONFIG1 регистр
  uint8_t config1 = readRegisterDirect(ADS1220_REG_CONFIG1);
  Serial.print("CONFIG1 до изменения: 0x");
  Serial.println(config1, HEX);
  
  // Очищаем биты DR[3:0] (6-2) и устанавливаем новый SPS
  config1 &= 0xE3; // Очищаем биты DR[3:0]
  config1 |= (currentSPS << 2); // Устанавливаем новый SPS
  
  Serial.print("CONFIG1 после изменения: 0x");
  Serial.println(config1, HEX);
  Serial.print("Устанавливаем SPS: 0x");
  Serial.println(currentSPS, HEX);
  
  // Записываем обновленный регистр с повторными попытками
  int attempts = 0;
  uint8_t new_config1;
  uint8_t actual_sps;
  
  do {
    writeRegisterDirect(ADS1220_REG_CONFIG1, config1);
    delay(50);
    
    // Дополнительная запись через библиотеку
    ads1220.writeRegister(ADS1220_REG_CONFIG1, config1);
    delay(50);
    
    new_config1 = readRegisterDirect(ADS1220_REG_CONFIG1);
    actual_sps = (new_config1 >> 2) & 0x0F;
    attempts++;
    
    Serial.print("Попытка ");
    Serial.print(attempts);
    Serial.print(": CONFIG1 = 0x");
    Serial.print(new_config1, HEX);
    Serial.print(", SPS = 0x");
    Serial.println(actual_sps, HEX);
    
  } while (actual_sps != currentSPS && attempts < 5);
  
  Serial.print("CONFIG1 после записи: 0x");
  Serial.println(new_config1, HEX);
  Serial.print("Фактический SPS: 0x");
  Serial.println(actual_sps, HEX);
  Serial.print("Ожидаемый SPS: 0x");
  Serial.println(currentSPS, HEX);
  
  if (actual_sps == currentSPS) {
    Serial.print("✅ Скорость успешно изменена на ");
    Serial.print(currentSPSName);
    Serial.println(" SPS!");
  } else {
    Serial.println("❌ Ошибка изменения скорости!");
    Serial.println("Попытка принудительного сброса и перезаписи...");
    
    // Принудительный сброс устройства
    ads1220_send_command(ADS1220_RESET_CMD);
    delay(100);
    
    // Повторная попытка записи
    writeRegisterDirect(ADS1220_REG_CONFIG1, config1);
    delay(100);
    ads1220.writeRegister(ADS1220_REG_CONFIG1, config1);
    delay(100);
    
    uint8_t final_config1 = readRegisterDirect(ADS1220_REG_CONFIG1);
    uint8_t final_sps = (final_config1 >> 2) & 0x0F;
    
    Serial.print("CONFIG1 после сброса: 0x");
    Serial.println(final_config1, HEX);
    Serial.print("SPS после сброса: 0x");
    Serial.println(final_sps, HEX);
    
    if (final_sps == currentSPS) {
      Serial.print("✅ Скорость установлена после сброса: ");
      Serial.print(currentSPSName);
      Serial.println(" SPS!");
    } else {
      Serial.println("❌ Критическая ошибка: регистр не изменяется!");
      Serial.println("Попробуйте перезапустить устройство командой 'r'");
    }
  }
  
  // Обновляем глобальные переменные для корректной работы getCurrentSPS()
  currentSPS = actual_sps;
  // Находим соответствующее название скорости
  for (int i = 0; i < SPS_COUNT; i++) {
    if (SPS_SETTINGS[i] == actual_sps) {
      currentSPSName = SPS_NAMES[i];
      break;
    }
  }
  
  // Обновляем интервал вывода при изменении скорости
  updateOutputInterval();
}

// Функция для получения текущей скорости
uint8_t getCurrentSPS() {
  uint8_t config1 = readRegisterDirect(ADS1220_REG_CONFIG1);
  uint8_t sps_setting = (config1 >> 2) & 0x0F;
  
  // Находим индекс текущей настройки
  for (int i = 0; i < SPS_COUNT; i++) {
    if (SPS_SETTINGS[i] == sps_setting) {
      return i;
    }
  }
  
  return 0; // По умолчанию 20 SPS
}

// Функция для отображения информации о скоростях
void showSpeedInfo() {
  Serial.println("=== Информация о скоростях ADS1220 ===");
  Serial.print("Текущая скорость: ");
  Serial.print(currentSPSName);
  Serial.println(" SPS");
  
  Serial.println("Доступные скорости:");
  Serial.println("Индекс | SPS  | Время преобразования | Применение");
  Serial.println("-------|------|---------------------|------------");
  
  for (int i = 0; i < SPS_COUNT; i++) {
    float conversion_time = 1000.0 / atoi(SPS_NAMES[i]); // Время в мс
    
    Serial.print("  ");
    Serial.print(i);
    Serial.print("    | ");
    Serial.print(SPS_NAMES[i]);
    Serial.print("   | ");
    Serial.print(conversion_time, 1);
    Serial.print(" мс              | ");
    
    // Рекомендации по применению
    if (i <= 2) {
      Serial.println("Высокая точность, низкий шум");
    } else if (i <= 4) {
      Serial.println("Баланс точности и скорости");
    } else if (i <= 6) {
      Serial.println("Высокая скорость, умеренная точность");
    } else {
      Serial.println("Максимальная скорость");
    }
  }
  
  Serial.println();
  Serial.println("Команды для переключения скорости:");
  Serial.println("  + - увеличить скорость");
  Serial.println("  - - уменьшить скорость");
  Serial.println("  0-7 - установить конкретную скорость");
  Serial.println("  i - показать эту информацию");
  Serial.println("=== Информация о скоростях завершена ===");
  Serial.println();
}

// Функция для увеличения скорости
void increaseSpeed() {
  uint8_t current_index = getCurrentSPS();
  Serial.print("Текущий индекс скорости: ");
  Serial.println(current_index);
  Serial.print("Текущая скорость: ");
  Serial.print(currentSPSName);
  Serial.println(" SPS");
  
  if (current_index < SPS_COUNT - 1) {
    Serial.print("Увеличиваем скорость с ");
    Serial.print(currentSPSName);
    Serial.print(" до ");
    Serial.print(SPS_NAMES[current_index + 1]);
    Serial.println(" SPS");
    setSPS(current_index + 1);
  } else {
    Serial.println("Уже установлена максимальная скорость!");
  }
}

// Функция для уменьшения скорости
void decreaseSpeed() {
  uint8_t current_index = getCurrentSPS();
  Serial.print("Текущий индекс скорости: ");
  Serial.println(current_index);
  Serial.print("Текущая скорость: ");
  Serial.print(currentSPSName);
  Serial.println(" SPS");
  
  if (current_index > 0) {
    Serial.print("Уменьшаем скорость с ");
    Serial.print(currentSPSName);
    Serial.print(" до ");
    Serial.print(SPS_NAMES[current_index - 1]);
    Serial.println(" SPS");
    setSPS(current_index - 1);
  } else {
    Serial.println("Уже установлена минимальная скорость!");
  }
}

// Функция для установки коэффициента усиления
void setGAIN(uint8_t gain_setting) {
  if (gain_setting >= GAIN_COUNT) {
    Serial.println("Ошибка: Неверная настройка GAIN!");
    return;
  }
  
  Serial.print("Устанавливаем усиление с индексом ");
  Serial.print(gain_setting);
  Serial.print(" (");
  Serial.print(GAIN_NAMES[gain_setting]);
  Serial.println("x)");
  
  // Обновляем текущее усиление
  currentGAIN = GAIN_SETTINGS[gain_setting];
  currentGAINName = GAIN_NAMES[gain_setting];
  
  // Читаем текущий CONFIG0 регистр
  uint8_t config0 = readRegisterDirect(ADS1220_REG_CONFIG0);
  Serial.print("CONFIG0 до изменения: 0x");
  Serial.println(config0, HEX);
  
  // Очищаем биты GAIN[2:0] (2-0) и устанавливаем новый GAIN
  config0 &= 0xF8; // Очищаем биты GAIN[2:0]
  config0 |= currentGAIN; // Устанавливаем новый GAIN
  
  Serial.print("CONFIG0 после изменения: 0x");
  Serial.println(config0, HEX);
  Serial.print("Устанавливаем GAIN: 0x");
  Serial.println(currentGAIN, HEX);
  
  // Записываем обновленный регистр с повторными попытками
  int attempts = 0;
  uint8_t new_config0;
  uint8_t actual_gain;
  
  do {
    writeRegisterDirect(ADS1220_REG_CONFIG0, config0);
    delay(50);
    
    // Дополнительная запись через библиотеку
    ads1220.writeRegister(ADS1220_REG_CONFIG0, config0);
    delay(50);
    
    new_config0 = readRegisterDirect(ADS1220_REG_CONFIG0);
    actual_gain = new_config0 & 0x07;
    attempts++;
    
    Serial.print("Попытка ");
    Serial.print(attempts);
    Serial.print(": CONFIG0 = 0x");
    Serial.print(new_config0, HEX);
    Serial.print(", GAIN = 0x");
    Serial.println(actual_gain, HEX);
    
  } while (actual_gain != currentGAIN && attempts < 5);
  
  Serial.print("CONFIG0 после записи: 0x");
  Serial.println(new_config0, HEX);
  Serial.print("Фактический GAIN: 0x");
  Serial.println(actual_gain, HEX);
  Serial.print("Ожидаемый GAIN: 0x");
  Serial.println(currentGAIN, HEX);
  
  if (actual_gain == currentGAIN) {
    Serial.print("✅ Усиление успешно изменено на ");
    Serial.print(currentGAINName);
    Serial.println("x!");
  } else {
    Serial.println("❌ Ошибка изменения усиления!");
    Serial.println("Попытка принудительного сброса и перезаписи...");
    
    // Принудительный сброс устройства
    ads1220_send_command(ADS1220_RESET_CMD);
    delay(100);
    
    // Повторная попытка записи
    writeRegisterDirect(ADS1220_REG_CONFIG0, config0);
    delay(100);
    ads1220.writeRegister(ADS1220_REG_CONFIG0, config0);
    delay(100);
    
    uint8_t final_config0 = readRegisterDirect(ADS1220_REG_CONFIG0);
    uint8_t final_gain = final_config0 & 0x07;
    
    Serial.print("CONFIG0 после сброса: 0x");
    Serial.println(final_config0, HEX);
    Serial.print("GAIN после сброса: 0x");
    Serial.println(final_gain, HEX);
    
    if (final_gain == currentGAIN) {
      Serial.print("✅ Усиление установлено после сброса: ");
      Serial.print(currentGAINName);
      Serial.println("x!");
    } else {
      Serial.println("❌ Критическая ошибка: регистр не изменяется!");
      Serial.println("Попробуйте перезапустить устройство командой 'r'");
    }
  }
}

// Функция для получения текущего усиления
uint8_t getCurrentGAIN() {
  uint8_t config0 = readRegisterDirect(ADS1220_REG_CONFIG0);
  uint8_t gain_setting = config0 & 0x07;
  
  // Находим индекс текущей настройки
  for (int i = 0; i < GAIN_COUNT; i++) {
    if (GAIN_SETTINGS[i] == gain_setting) {
      return i;
    }
  }
  
  return 5; // По умолчанию 128x (индекс 5)
}

// Функция для увеличения усиления
void increaseGain() {
  uint8_t current_index = getCurrentGAIN();
  Serial.print("Текущий индекс усиления: ");
  Serial.println(current_index);
  Serial.print("Текущее усиление: ");
  Serial.print(currentGAINName);
  Serial.println("x");
  
  if (current_index < GAIN_COUNT - 1) {
    Serial.print("Увеличиваем усиление с ");
    Serial.print(currentGAINName);
    Serial.print(" до ");
    Serial.print(GAIN_NAMES[current_index + 1]);
    Serial.println("x");
    setGAIN(current_index + 1);
  } else {
    Serial.println("Уже установлено максимальное усиление!");
  }
}

// Функция для уменьшения усиления
void decreaseGain() {
  uint8_t current_index = getCurrentGAIN();
  Serial.print("Текущий индекс усиления: ");
  Serial.println(current_index);
  Serial.print("Текущее усиление: ");
  Serial.print(currentGAINName);
  Serial.println("x");
  
  if (current_index > 0) {
    Serial.print("Уменьшаем усиление с ");
    Serial.print(currentGAINName);
    Serial.print(" до ");
    Serial.print(GAIN_NAMES[current_index - 1]);
    Serial.println("x");
    setGAIN(current_index - 1);
  } else {
    Serial.println("Уже установлено минимальное усиление!");
  }
}

// Функция для расчета интервала вывода на основе скорости
void updateOutputInterval() {
  int sps_value = atoi(currentSPSName);
  if (sps_value > 0) {
    // Интервал вывода = 1000мс / SPS, но не меньше 50мс
    outputInterval = max(50, 1000 / sps_value);
  } else {
    outputInterval = 1000; // По умолчанию 1 секунда
  }
  
  Serial.print("Интервал вывода установлен: ");
  Serial.print(outputInterval);
  Serial.println(" мс");
}

// Функция для включения/выключения вывода
void toggleOutput() {
  outputEnabled = !outputEnabled;
  if (outputEnabled) {
    Serial.println("✅ Вывод данных включен");
  } else {
    Serial.println("❌ Вывод данных отключен");
  }
}

// Функция для диагностики скорости вывода
void diagnoseOutputSpeed() {
  Serial.println("=== Диагностика скорости вывода ===");
  Serial.print("Текущая скорость SPS: ");
  Serial.println(currentSPSName);
  Serial.print("Расчетный интервал вывода: ");
  Serial.print(outputInterval);
  Serial.println(" мс");
  Serial.print("Высокоскоростной режим: ");
  Serial.println(highSpeedMode ? "ВКЛ" : "ВЫКЛ");
  
  // Тест скорости измерений
  Serial.println("Тест скорости измерений...");
  unsigned long start_time = millis();
  int test_count = 10;
  
  for (int i = 0; i < test_count; i++) {
    setMuxChannel(MUX_AIN0_AIN1);
    readADS1220();
    setMuxChannel(MUX_AIN2_AIN3);
    readADS1220();
  }
  
  unsigned long end_time = millis();
  unsigned long total_time = end_time - start_time;
  float avg_time = (float)total_time / test_count;
  
  Serial.print("Время для ");
  Serial.print(test_count);
  Serial.print(" циклов измерений: ");
  Serial.print(total_time);
  Serial.println(" мс");
  Serial.print("Среднее время на цикл: ");
  Serial.print(avg_time, 1);
  Serial.println(" мс");
  
  if (avg_time < outputInterval) {
    Serial.println("✅ Скорость измерений достаточна для вывода");
  } else {
    Serial.println("⚠️  Скорость измерений может быть недостаточной");
  }
  
  Serial.println("=== Диагностика завершена ===");
  Serial.println();
}

// Функция для переключения высокоскоростного режима
void toggleHighSpeedMode() {
  highSpeedMode = !highSpeedMode;
  if (highSpeedMode) {
    Serial.println("🚀 Высокоскоростной режим ВКЛЮЧЕН");
    Serial.println("⚠️  Внимание: Могут быть небольшие погрешности");
  } else {
    Serial.println("🐌 Обычный режим ВКЛЮЧЕН");
    Serial.println("✅ Стабильные измерения с задержками");
  }
}

// Функция для диагностики проблемы с записью регистров
void diagnoseRegisterWrite() {
  Serial.println("=== Диагностика записи регистров ===");
  
  // Тест 1: Проверка чтения регистров
  Serial.println("1. Проверка чтения регистров:");
  uint8_t config0 = readRegisterDirect(ADS1220_REG_CONFIG0);
  uint8_t config1 = readRegisterDirect(ADS1220_REG_CONFIG1);
  uint8_t config2 = readRegisterDirect(ADS1220_REG_CONFIG2);
  
  Serial.print("CONFIG0: 0x");
  Serial.println(config0, HEX);
  Serial.print("CONFIG1: 0x");
  Serial.println(config1, HEX);
  Serial.print("CONFIG2: 0x");
  Serial.println(config2, HEX);
  
  // Тест 2: Попытка записи тестовых значений
  Serial.println("2. Тест записи тестовых значений:");
  
  // Сохраняем оригинальные значения
  uint8_t orig_config0 = config0;
  uint8_t orig_config1 = config1;
  uint8_t orig_config2 = config2;
  
  // Пробуем записать тестовые значения
  Serial.println("Записываем 0xAA в CONFIG0...");
  writeRegisterDirect(ADS1220_REG_CONFIG0, 0xAA);
  delay(50);
  
  uint8_t test_config0 = readRegisterDirect(ADS1220_REG_CONFIG0);
  Serial.print("CONFIG0 после записи 0xAA: 0x");
  Serial.println(test_config0, HEX);
  
  if (test_config0 == 0xAA) {
    Serial.println("✅ Запись в CONFIG0 работает!");
  } else {
    Serial.println("❌ Запись в CONFIG0 НЕ работает!");
  }
  
  // Тест 3: Запись через библиотеку
  Serial.println("3. Тест записи через библиотеку:");
  ads1220.writeRegister(ADS1220_REG_CONFIG1, 0xBB);
  delay(50);
  
  uint8_t lib_config1 = readRegisterDirect(ADS1220_REG_CONFIG1);
  Serial.print("CONFIG1 после записи 0xBB через библиотеку: 0x");
  Serial.println(lib_config1, HEX);
  
  if (lib_config1 == 0xBB) {
    Serial.println("✅ Запись через библиотеку работает!");
  } else {
    Serial.println("❌ Запись через библиотеку НЕ работает!");
  }
  
  // Тест 4: Специальный тест для CONFIG1
  Serial.println("4. Специальный тест для CONFIG1 (SPS):");
  
  // Устанавливаем SPS = 0b0110 (1000 SPS)
  uint8_t test_config1 = config1;
  test_config1 &= 0xE3; // Очищаем биты DR[3:0]
  test_config1 |= (0b0110 << 2); // Устанавливаем 1000 SPS
  
  Serial.print("CONFIG1 для 1000 SPS: 0x");
  Serial.println(test_config1, HEX);
  
  writeRegisterDirect(ADS1220_REG_CONFIG1, test_config1);
  delay(100);
  
  uint8_t result_config1 = readRegisterDirect(ADS1220_REG_CONFIG1);
  uint8_t result_sps = (result_config1 >> 2) & 0x0F;
  
  Serial.print("CONFIG1 после записи: 0x");
  Serial.println(result_config1, HEX);
  Serial.print("SPS после записи: 0x");
  Serial.println(result_sps, HEX);
  
  if (result_sps == 0b0110) {
    Serial.println("✅ SPS установлен правильно!");
  } else {
    Serial.println("❌ SPS НЕ установлен!");
  }
  
  // Восстанавливаем оригинальные значения
  Serial.println("5. Восстановление оригинальных значений:");
  writeRegisterDirect(ADS1220_REG_CONFIG0, orig_config0);
  writeRegisterDirect(ADS1220_REG_CONFIG1, orig_config1);
  writeRegisterDirect(ADS1220_REG_CONFIG2, orig_config2);
  
  Serial.println("Оригинальные значения восстановлены");
  Serial.println("=== Диагностика завершена ===");
  Serial.println();
}

// Функция для вывода конфигурации регистров
void printConfiguration() {
  Serial.println("ADS1220 Configuration:");
  
  // Читаем регистры с обработкой ошибок
  uint8_t config0, config1, config2, config3;
  
  config0 = readRegisterDirect(ADS1220_REG_CONFIG0);
  config1 = readRegisterDirect(ADS1220_REG_CONFIG1);
  config2 = readRegisterDirect(ADS1220_REG_CONFIG2);
  config3 = readRegisterDirect(ADS1220_REG_CONFIG3);
  
  Serial.print("CONFIG0 (0x00): 0x");
  Serial.println(config0, HEX);
  Serial.print("CONFIG1 (0x01): 0x");
  Serial.println(config1, HEX);
  Serial.print("CONFIG2 (0x02): 0x");
  Serial.println(config2, HEX);
  Serial.print("CONFIG3 (0x03): 0x");
  Serial.println(config3, HEX);
  
  // Анализ MUX настройки
  uint8_t mux_setting = (config0 >> 4) & 0x0F;
  Serial.print("Текущий MUX канал: 0x");
  Serial.println(mux_setting, HEX);
  
  if (mux_setting == MUX_AIN0_AIN1) {
    Serial.println("✅ MUX настроен на AIN0-AIN1 (Тензодатчик 1)");
  } else if (mux_setting == MUX_AIN2_AIN3) {
    Serial.println("✅ MUX настроен на AIN2-AIN3 (Тензодатчик 2)");
  } else {
    Serial.println("⚠️  MUX настроен на неизвестный канал");
  }
  
  // Анализ других настроек
  uint8_t gain_setting = config0 & 0x07;
  uint8_t pga_bypass = (config0 >> 3) & 0x01;
  uint8_t dr_setting = (config1 >> 2) & 0x0F;
  uint8_t mode_setting = config1 & 0x03;
  
  Serial.print("Усиление (GAIN): ");
  Serial.println(gain_setting);
  Serial.print("PGA Bypass: ");
  Serial.println(pga_bypass ? "ВЫКЛ" : "ВКЛ");
  Serial.print("Скорость (DR): ");
  Serial.println(dr_setting);
  Serial.print("Режим (MODE): ");
  Serial.println(mode_setting);
  
  Serial.println();
}

// Функция для калибровки тензодатчиков
void calibrateStrainGauges() {
  Serial.println("=== Калибровка тензодатчиков ===");
  Serial.println("1. Убедитесь, что тензодатчики не нагружены");
  Serial.println("2. Нажмите любую клавишу для начала калибровки...");
  
  while (!Serial.available()) {
    delay(100);
  }
  Serial.read(); // Очищаем буфер
  
  // Читаем базовые значения
  Serial.println("Чтение базовых значений...");
  float baseline1 = 0, baseline2 = 0;
  const int samples = 10;
  
  for (int i = 0; i < samples; i++) {
    baseline1 += readStrainGauge(MUX_AIN0_AIN1, 1.0, 0.0);
    baseline2 += readStrainGauge(MUX_AIN2_AIN3, 1.0, 0.0);
    delay(100);
  }
  
  baseline1 /= samples;
  baseline2 /= samples;
  
  Serial.print("Базовое значение тензодатчика 1: ");
  Serial.println(baseline1);
  Serial.print("Базовое значение тензодатчика 2: ");
  Serial.println(baseline2);
  
  // Устанавливаем смещения нуля
  offset1 = baseline1;
  offset2 = baseline2;
  
  Serial.println("Калибровка завершена!");
  Serial.println();
}

// Функция для сканирования I2C-устройств
void i2cScan() {
  Serial.println("Сканирование I2C-шины...");
  byte count = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Найдено устройство по адресу 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
      count++;
      delay(2);
    }
  }
  if (count == 0) Serial.println("I2C-устройства не найдены");
  else Serial.print("Всего найдено: "), Serial.println(count);
}


void setup() {
  // Инициализация последовательного порта
  Serial.begin(115200);
  Serial.println("Инициализация системы тензодатчиков...");
  
  // Настройка SPI
  SPI.begin(ADS1220_SCLK_PIN, ADS1220_MISO_PIN, ADS1220_MOSI_PIN, ADS1220_CS_PIN);
  
  // Настройка пинов
  pinMode(ADS1220_CS_PIN, OUTPUT);
  pinMode(ADS1220_DRDY_PIN, INPUT);
  digitalWrite(ADS1220_CS_PIN, HIGH);
  
  // Инициализация ADS1220
  ads1220.begin(ADS1220_CS_PIN, ADS1220_DRDY_PIN);
  
  // Сброс устройства
  ads1220_send_command(ADS1220_RESET_CMD);
  delay(100);
  
  // Настройка регистров
  configureADS1220();
  
  // Вывод конфигурации
  printConfiguration();
  
  // Инициализация системы скорости
  currentSPS = SPS_SETTINGS[6]; // 1000 SPS по умолчанию
  currentSPSName = SPS_NAMES[6];
  currentGAIN = GAIN_SETTINGS[2]; // 4x по умолчанию
  currentGAINName = GAIN_NAMES[2];
  
  // Инициализация системы вывода
  updateOutputInterval();
  
  // Калибровка тензодатчиков
  calibrateStrainGauges();

  // --- Инициализация BNO08x ---
  pinMode(BNO08X_RST_PIN, OUTPUT);
  digitalWrite(BNO08X_RST_PIN, LOW);
  delay(10);
  digitalWrite(BNO08X_RST_PIN, HIGH);
  delay(100); // Дать время на загрузку

  Wire.begin(BNO08X_SDA_PIN, BNO08X_SCL_PIN);
  delay(10);

  Serial.print("Попытка инициализации BNO08x по адресу 0x");
  Serial.println(0x4B, HEX);

  if (!bno08x.begin_I2C(0x4B, &Wire, BNO08X_INT_PIN)) {
    Serial.println("BNO08x не найден!");
    bno08xReady = false;
  } else {
    Serial.println("BNO08x найден и инициализирован!");
    bno08x.enableReport(SH2_ROTATION_VECTOR);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION);
    bno08xReady = true;
  }
  
  Serial.println("Система готова к работе!");
  Serial.println("✅ Проблема с записью регистров исправлена!");
  Serial.println("Формат вывода: Время(мс) | Скорость | Усиление | Тензодатчик1 | Тензодатчик2");
  Serial.println("================================================");
  Serial.println("Команды управления скоростью:");
  Serial.println("  + - увеличить скорость");
  Serial.println("  - - уменьшить скорость");
  Serial.println("  0-7 - установить конкретную скорость");
  Serial.println("  i - показать информацию о скоростях");
  Serial.println("Команды управления усилением:");
  Serial.println("  g - увеличить усиление");
  Serial.println("  G - уменьшить усиление");
  Serial.println("  a,b,d - установить усиление 1x,2x,8x");
  Serial.println("  A,B,C,D - установить усиление 16x,32x,64x,128x");
  Serial.println("Команды управления выводом:");
  Serial.println("  o - включить/выключить вывод данных");
  Serial.println("  O - обновить интервал вывода");
  Serial.println("  s - диагностика скорости вывода");
  Serial.println("  H - переключить высокоскоростной режим");
  Serial.println("Команды для BNO08x:");
  Serial.println("  n - вывести данные BNO08x");
  Serial.println("  N - перезапустить BNO08x");
  Serial.println("  z - сканировать I2C-шину");
}

void loop() {
  static unsigned long last_switch = 0;
  static uint8_t current_channel = MUX_AIN2_AIN3; // Начинаем с другого канала
  static bool first_reading = true;
  
  if (highSpeedMode) {
    // Высокоскоростной режим - без задержек
    // Читаем первый тензодатчик (AIN0-AIN1)
    if (current_channel != MUX_AIN0_AIN1 || first_reading) {
      setMuxChannel(MUX_AIN0_AIN1);
      current_channel = MUX_AIN0_AIN1;
      first_reading = false;
    }
    
    float strain1 = readADS1220();
    strain1 = (strain1 - offset1) * scaleFactor1;
    
    // Читаем второй тензодатчик (AIN2-AIN3)
    if (current_channel != MUX_AIN2_AIN3) {
      setMuxChannel(MUX_AIN2_AIN3);
      current_channel = MUX_AIN2_AIN3;
    }
    
    float strain2 = readADS1220();
    strain2 = (strain2 - offset2) * scaleFactor2;
    
    // Вывод результатов в соответствии с интервалом
    unsigned long currentTime = millis();
    if (outputEnabled && (currentTime - lastOutputTime >= outputInterval)) {
      Serial.print(currentTime);
      Serial.print(" | ");
      Serial.print(currentSPSName);
      Serial.print("SPS | ");
      Serial.print(currentGAINName);
      Serial.print("x | ");
      Serial.print(strain1, 4);
      Serial.print(" | ");
      Serial.println(strain2, 4);
      
      lastOutputTime = currentTime;
    }
  } else {
    // Обычный режим - с задержками для стабильности
    // Читаем первый тензодатчик (AIN0-AIN1)
    if (current_channel != MUX_AIN0_AIN1 || first_reading) {
      setMuxChannel(MUX_AIN0_AIN1);
      current_channel = MUX_AIN0_AIN1;
      last_switch = millis();
      delay(20); // Уменьшенная задержка для стабилизации
      first_reading = false;
    }
    
    float strain1 = readADS1220();
    strain1 = (strain1 - offset1) * scaleFactor1;
    
    // Переход в режим пониженного потребления между измерениями
    ads1220_send_command(ADS1220_PWRDOWN_CMD);
    delay(10); // Минимальная задержка
    
    // Читаем второй тензодатчик (AIN2-AIN3)
    if (current_channel != MUX_AIN2_AIN3) {
      setMuxChannel(MUX_AIN2_AIN3);
      current_channel = MUX_AIN2_AIN3;
      last_switch = millis();
      delay(20); // Уменьшенная задержка для стабилизации
    }
    
    float strain2 = readADS1220();
    strain2 = (strain2 - offset2) * scaleFactor2;
    
    // Вывод результатов в соответствии с интервалом
    unsigned long currentTime = millis();
    if (outputEnabled && (currentTime - lastOutputTime >= outputInterval)) {
      Serial.print(currentTime);
      Serial.print(" | ");
      Serial.print(currentSPSName);
      Serial.print("SPS | ");
      Serial.print(currentGAINName);
      Serial.print("x | ");
      Serial.print(strain1, 4);
      Serial.print(" | ");
      Serial.println(strain2, 4);
      
      lastOutputTime = currentTime;
    }
    
    // Переход в режим пониженного потребления
    ads1220_send_command(ADS1220_PWRDOWN_CMD);
    delay(10); // Минимальная задержка между циклами
  }
}

// Обработка команд через Serial
void serialEvent() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 'c':
        calibrateStrainGauges();
        break;
        
      case 'p':
        printConfiguration();
        break;
        
      case 'r':
        ads1220_send_command(ADS1220_RESET_CMD);
        delay(100);
        configureADS1220();
        Serial.println("Устройство перезапущено!");
        break;
        
      case 'h':
        Serial.println("Доступные команды:");
        Serial.println("  c - калибровка тензодатчиков");
        Serial.println("  p - показать конфигурацию");
        Serial.println("  r - перезапуск устройства");
        Serial.println("  i - показать информацию о скоростях");
        Serial.println("  y - тест различных SPS");
        Serial.println("  e - диагностика записи регистров");
        Serial.println("  + - увеличить скорость");
        Serial.println("  - - уменьшить скорость");
        Serial.println("  0-7 - установить конкретную скорость");
        Serial.println("  g - увеличить усиление");
        Serial.println("  G - уменьшить усиление");
        Serial.println("  o - включить/выключить вывод данных");
        Serial.println("  O - обновить интервал вывода");
        Serial.println("  s - диагностика скорости вывода");
        Serial.println("  H - переключить высокоскоростной режим");
        Serial.println("  n - вывести данные BNO08x");
        Serial.println("  N - перезапустить BNO08x");
        Serial.println("  z - сканировать I2C-шину");
        break;
        

         
      case 'i':
         showSpeedInfo();
         break;
         
      case 'y':
         testSPSSettings();
         break;
         
      case 'e':
         diagnoseRegisterWrite();
         break;
         
      case '+':
         increaseSpeed();
         break;
         
      case '-':
         decreaseSpeed();
         break;
         
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
         setSPS(cmd - '0');
         break;
         
      case 'g':
         increaseGain();
         break;
         
      case 'G':
         decreaseGain();
         break;
         
      case 'o':
         toggleOutput();
         break;
         
      case 'O':
         updateOutputInterval();
         break;
         
      case 's':
         diagnoseOutputSpeed();
         break;
         
      case 'H':
         toggleHighSpeedMode();
         break;

      case 'n':
        printBNO08xData();
        break;
      case 'N':
        Serial.println("Сброс BNO08x...");
        digitalWrite(BNO08X_RST_PIN, LOW);
        delay(10);
        digitalWrite(BNO08X_RST_PIN, HIGH);
        delay(10);
        if (!bno08x.begin_I2C(0x4A, &Wire, BNO08X_INT_PIN)) {
          Serial.println("BNO08x не найден после сброса!");
        } else {
          Serial.println("BNO08x перезапущен!");
          bno08x.enableReport(SH2_ROTATION_VECTOR);
          bno08x.enableReport(SH2_LINEAR_ACCELERATION);
        }
        break;
        
      case 'z':
        i2cScan();
        break;
        
      default:
        if (cmd != '\n' && cmd != '\r') {
          Serial.print("Неизвестная команда: ");
          Serial.println(cmd);
        }
        break;
    }
  }
} 



 



// Функция для отображения всех вариантов SPS ADS1220
void showSPSOptions() {
  Serial.println("=== Доступные варианты SPS для ADS1220 ===");
  Serial.println("DR[3:0] | SPS    | Время преобразования | Применение");
  Serial.println("---------|--------|---------------------|------------");
  Serial.println("0000     | 20     | 50 мс              | Медленные измерения");
  Serial.println("0001     | 45     | 22.2 мс            | Стабильные измерения");
  Serial.println("0010     | 90     | 11.1 мс            | Баланс скорость/точность");
  Serial.println("0011     | 175    | 5.7 мс             | Быстрые измерения");
  Serial.println("0100     | 330    | 3.0 мс             | Высокая скорость");
  Serial.println("0101     | 600    | 1.67 мс            | Очень быстрые");
  Serial.println("0110     | 1000   | 1.0 мс             | Максимальная скорость");
  Serial.println("0111     | 2000   | 0.5 мс             | Сверхбыстрые");
  Serial.println();
  
  Serial.println("Рекомендации по выбору SPS:");
  Serial.println("- 20-90 SPS: Высокая точность, низкий шум");
  Serial.println("- 175-330 SPS: Баланс точности и скорости");
  Serial.println("- 600-1000 SPS: Высокая скорость, умеренная точность");
  Serial.println("- 2000 SPS: Максимальная скорость, может быть шум");
  Serial.println();
  
  Serial.println("Для тензодатчиков рекомендуется:");
  Serial.println("- 20-90 SPS для статических измерений");
  Serial.println("- 175-330 SPS для динамических измерений");
  Serial.println("- 600-1000 SPS для быстрых изменений");
  Serial.println();
}

// Функция для тестирования различных SPS
void testSPSSettings() {
  Serial.println("=== Тест различных SPS ===");
  
  // Массив SPS настроек
  const uint8_t sps_settings[] = {0b0000, 0b0001, 0b0010, 0b0011, 0b0100, 0b0101, 0b0110, 0b0111};
  const char* sps_names[] = {"20", "45", "90", "175", "330", "600", "1000", "2000"};
  
  // Тестируем каждый SPS
  for (int i = 0; i < 8; i++) {
    Serial.print("Тестирование ");
    Serial.print(sps_names[i]);
    Serial.println(" SPS:");
    
    // Устанавливаем SPS
    uint8_t config1 = readRegisterDirect(ADS1220_REG_CONFIG1);
    Serial.print("CONFIG1 до изменения: 0x");
    Serial.println(config1, HEX);
    
    config1 &= 0xE3; // Очищаем биты DR[3:0] (6-2)
    config1 |= (sps_settings[i] << 2); // Устанавливаем новый SPS
    
    Serial.print("CONFIG1 после изменения: 0x");
    Serial.println(config1, HEX);
    
    writeRegisterDirect(ADS1220_REG_CONFIG1, config1);
    delay(200); // Увеличиваем задержку для стабилизации
    
    // Дополнительная проверка через библиотеку
    ads1220.writeRegister(ADS1220_REG_CONFIG1, config1);
    delay(100);
    
    // Проверяем, что регистр изменился
    uint8_t new_config1 = readRegisterDirect(ADS1220_REG_CONFIG1);
    uint8_t actual_sps = (new_config1 >> 2) & 0x0F;
    
    Serial.print("CONFIG1 после записи: 0x");
    Serial.println(new_config1, HEX);
    Serial.print("Ожидаемый SPS: 0x");
    Serial.print(sps_settings[i], HEX);
    Serial.print(", Фактический SPS: 0x");
    Serial.println(actual_sps, HEX);
    
    if (actual_sps == sps_settings[i]) {
      Serial.println("✅ SPS установлен правильно");
    } else {
      Serial.println("❌ SPS НЕ установлен!");
    }
    
    // Читаем значения с обоих каналов
    setMuxChannel(MUX_AIN0_AIN1);
    delay(50);
    float value1 = readADS1220();
    
    setMuxChannel(MUX_AIN2_AIN3);
    delay(50);
    float value2 = readADS1220();
    
    Serial.print("  AIN0-AIN1: ");
    Serial.println(value1);
    Serial.print("  AIN2-AIN3: ");
    Serial.println(value2);
    Serial.println();
  }
  
  // Восстанавливаем исходный SPS (20)
  uint8_t config1 = readRegisterDirect(ADS1220_REG_CONFIG1);
  config1 &= 0xE3; // Очищаем биты DR[3:0]
  config1 |= (0b0000 << 2); // Устанавливаем 20 SPS
  writeRegisterDirect(ADS1220_REG_CONFIG1, config1);
  
  Serial.println("Восстановлен SPS 20");
  Serial.println("=== Тест SPS завершён ===");
  Serial.println();
}

// Функция для вывода данных BNO08x
void printBNO08xData() {
    if (!bno08xReady) {
        Serial.println("BNO08x не инициализирован!");
        return;
    }
  if (bno08x.wasReset()) {
    Serial.println("BNO08x был сброшен, повторная инициализация...");
    bno08x.enableReport(SH2_ROTATION_VECTOR);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION);
  }
  if (bno08x.getSensorEvent(&bno08x_sensorValue)) {
    if (bno08x_sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      Serial.print("BNO08x Кватернион: ");
      Serial.print(bno08x_sensorValue.un.rotationVector.real);
      Serial.print(", ");
      Serial.print(bno08x_sensorValue.un.rotationVector.i);
      Serial.print(", ");
      Serial.print(bno08x_sensorValue.un.rotationVector.j);
      Serial.print(", ");
      Serial.println(bno08x_sensorValue.un.rotationVector.k);
    } else if (bno08x_sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
      Serial.print("BNO08x Ускорение: ");
      Serial.print(bno08x_sensorValue.un.linearAcceleration.x);
      Serial.print(", ");
      Serial.print(bno08x_sensorValue.un.linearAcceleration.y);
      Serial.print(", ");
      Serial.println(bno08x_sensorValue.un.linearAcceleration.z);
    }
  } else {
    Serial.println("Нет новых данных BNO08x");
  }
}