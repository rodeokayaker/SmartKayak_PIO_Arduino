#!/usr/bin/env python3
"""
Скрипт для тестирования состояния памяти ESP32
"""

import serial
import time
import sys

def test_memory():
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
        
        print("🔍 Тестирование состояния памяти...")
        
        # Отправляем команды для проверки памяти
        commands = [
            "help",
            "memory",
            "status"
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
            
            # Проверяем на ошибки
            for line in response_lines:
                if "panic" in line.lower() or "fault" in line.lower():
                    print(f"⚠️ Обнаружена ошибка: {line}")
        
        ser.close()
        print("✅ Тест завершен")
        return True
        
    except Exception as e:
        print(f"❌ Ошибка: {e}")
        return False

if __name__ == "__main__":
    print("🧠 Тестирование памяти ESP32...")
    if test_memory():
        print("✅ Готово!")
    else:
        print("❌ Тест не удался")
        sys.exit(1)
