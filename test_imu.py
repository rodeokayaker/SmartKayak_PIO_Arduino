#!/usr/bin/env python3
"""
Скрипт для тестирования и диагностики IMU BNO08X
"""

import serial
import time
import sys

def test_imu():
    try:
        # Попробуем найти доступные порты
        ports = ['/dev/cu.usbmodem101', '/dev/cu.usbserial-0001', '/dev/ttyUSB0', '/dev/ttyACM0']
        port = None
        
        for p in ports:
            try:
                ser = serial.Serial(p, 115200, timeout=1)
                ser.close()
                port = p
                break
            except:
                continue
        
        if not port:
            print("❌ Не удалось найти доступный COM порт")
            return False
            
        print(f"✓ Найден порт: {port}")
        
        # Открываем соединение
        ser = serial.Serial(port, 115200, timeout=2)
        time.sleep(2)  # Ждем стабилизации
        
        print("🔍 Тестирование IMU BNO08X...")
        
        # Отправляем команды для тестирования IMU
        commands = [
            "help",
            "imu_status",
            "imu_reset",
            "imu_scan",
            "imu_test"
        ]
        
        for cmd in commands:
            print(f"Отправляем команду: {cmd}")
            ser.write((cmd + '\n').encode())
            time.sleep(1)
            
            # Читаем ответ
            response_lines = []
            while ser.in_waiting > 0:
                response = ser.readline().decode().strip()
                if response:
                    response_lines.append(response)
                    print(f"Ответ: {response}")
            
            # Проверяем на ошибки I2C
            for line in response_lines:
                if "I2C connection failed" in line:
                    print(f"⚠️ Обнаружена ошибка I2C: {line}")
                elif "BNO08X found" in line:
                    print(f"✅ IMU найден: {line}")
                elif "✓" in line:
                    print(f"✅ Успех: {line}")
        
        ser.close()
        print("✅ Тест IMU завершен")
        return True
        
    except Exception as e:
        print(f"❌ Ошибка: {e}")
        return False

if __name__ == "__main__":
    print("🧭 Тестирование IMU BNO08X...")
    if test_imu():
        print("✅ Готово!")
    else:
        print("❌ Тест не удался")
        sys.exit(1)
