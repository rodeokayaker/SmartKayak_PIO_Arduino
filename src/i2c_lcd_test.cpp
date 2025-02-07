#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

#define I2C_SDA 21
#define I2C_SCL 22
#define LCD_ADDRESS 0x26
#define LCD_COLUMNS 16
#define LCD_ROWS 2
#define LCD2_ADDRESS 0x27
#define LCD2_COLUMNS 20
#define LCD2_ROWS 4
#define WIRE_SPEED 400000

// Создаем объекты для каждого экрана
hd44780_I2Cexp lcd(LCD_ADDRESS);
hd44780_I2Cexp lcd2(LCD2_ADDRESS);

void setup() {
  // Инициализация I2C с повышенной скоростью
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(WIRE_SPEED);
  
  Serial.begin(115200);
  Serial.println("LCD Test Starting...");

  // Инициализация первого LCD
  int status = lcd.begin(LCD_COLUMNS, LCD_ROWS);
  if(status) {
    Serial.printf("LCD 1 initialization failed: %d\n", status);
  }
  
  // Инициализация второго LCD
  status = lcd2.begin(LCD2_COLUMNS, LCD2_ROWS);
  if(status) {
    Serial.printf("LCD 2 initialization failed: %d\n", status);
  }

  // Вывод текста на первый LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("HELLO! LCD1 here");
  lcd.setCursor(0, 1);
  lcd.print("I2C LCD Test");

  // Вывод текста на второй LCD
  lcd2.clear();
  lcd2.setCursor(0, 0);
  lcd2.print("HELLO! LCD2 here");
  lcd2.setCursor(0, 1);
  lcd2.print("'2'nd row text");
  lcd2.setCursor(0, 2);
  lcd2.print("                    ");
  lcd2.setCursor(0, 3);
  lcd2.print("'4'th row. 1234567");
}

void loop() {
  static int counter = 0;
  char buf[20];
  uint32_t time = millis();
  // Обновление счетчика на первом LCD
  lcd.setCursor(0, 0);
  snprintf(buf, sizeof(buf), "%5d Counter: %d", counter, counter);
  lcd.print(buf);
  
  lcd.setCursor(0, 1);
  snprintf(buf, sizeof(buf), "%5d Counter: %d", counter, counter);
  lcd.print(buf);
  
  // Обновление счетчика на втором LCD
  lcd2.setCursor(0, 3);
  snprintf(buf, sizeof(buf), "%5d Counter: %d", counter, counter);
  lcd2.print(buf);
  lcd2.setCursor(0, 0);
  snprintf(buf, sizeof(buf), "%5d Counter: %d", counter, counter);
  lcd2.print(buf);

  Serial.printf("Time: %d\n", millis() - time);
  counter++;
  delay(100);
} 