#!/usr/bin/env python3
"""
Скрипт для очистки NVS памяти ESP32
Использование: python clear_nvs.py
"""

import serial
import time
import sys

def clear_nvs():
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
        
        # Отправляем команды для очистки
        commands = [
            "help",
            "clear_nvs",
            "reset"
        ]
        
        for cmd in commands:
            print(f"Отправляем команду: {cmd}")
            ser.write((cmd + '\n').encode())
            time.sleep(1)
            
            # Читаем ответ
            while ser.in_waiting > 0:
                response = ser.readline().decode().strip()
                if response:
                    print(f"Ответ: {response}")
        
        ser.close()
        print("✓ NVS очищен, устройство перезагружено")
        return True
        
    except Exception as e:
        print(f"❌ Ошибка: {e}")
        return False

if __name__ == "__main__":
    print("🔧 Очистка NVS памяти ESP32...")
    if clear_nvs():
        print("✅ Готово!")
    else:
        print("❌ Не удалось очистить NVS")
        sys.exit(1)
