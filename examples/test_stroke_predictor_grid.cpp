/**
 * Пример использования StrokePredictorGrid
 * 
 * Демонстрирует:
 * - Создание предиктора
 * - Обучение на синтетических данных
 * - Предсказание силы
 * - Вывод статистики
 */

#include <Arduino.h>
#include "StrokePredictorGrid.h"
#include "SP_Quaternion.h"
#include "SP_Vector.h"

// Глобальные переменные для тестирования
StrokePredictorGrid predictor;
SP_Math::Quaternion kayakQuat;
SP_Math::Quaternion paddleQuat;
SP_Math::Vector angularVelocity;

// Счетчики для статистики
uint32_t learnCount = 0;
uint32_t predictCount = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== StrokePredictorGrid Test ===\n");
    
    // Настройка конфигурации
    predictor.config.gridStep = 10.0f;
    predictor.config.alphaEMA = 0.2f;
    predictor.config.forceThreshold = 400.0f;
    predictor.config.predictionTime = 200.0f;
    predictor.learningMethod = LearningMethod::METHOD_B;
    
    Serial.println("Configuration:");
    Serial.printf("  Grid step: %.1f degrees\n", predictor.config.gridStep);
    Serial.printf("  Alpha EMA: %.2f\n", predictor.config.alphaEMA);
    Serial.printf("  Force threshold: %.1f\n", predictor.config.forceThreshold);
    Serial.printf("  Learning method: METHOD_B\n\n");
    
    // Инициализация кватернионов (каяк горизонтально, весло вертикально)
    kayakQuat = SP_Math::Quaternion(1, 0, 0, 0); // Единичный кватернион
    paddleQuat = SP_Math::Quaternion(1, 0, 0, 0);
    angularVelocity = SP_Math::Vector(0, 0, 0);
    
    Serial.println("Initialization complete!\n");
}

void simulateStroke() {
    // Симулируем один гребок
    // Весло движется от вертикального положения вперед, с постепенным увеличением силы
    
    Serial.println("=== Simulating Stroke ===");
    
    // Параметры гребка
    float strokeDuration = 1000.0f; // 1 секунда
    float timeStep = 50.0f; // 50мс между семплами
    int numSteps = (int)(strokeDuration / timeStep);
    
    for (int i = 0; i < numSteps; i++) {
        float progress = (float)i / numSteps;
        
        // Симулируем вращение весла
        // Вращаем вокруг оси X (наклон вперед)
        float angle = progress * 60.0f; // 60 градусов наклона
        SP_Math::Vector axis(1, 0, 0);
        paddleQuat = SP_Math::Quaternion::fromAxisAngle(axis, angle * 0.0174533f);
        
        // Симулируем угловую скорость
        float omega_x = 1.0f; // рад/с
        angularVelocity = SP_Math::Vector(omega_x, 0, 0);
        
        // Симулируем силу (увеличивается в середине гребка)
        float force = 0.0f;
        if (progress > 0.2f && progress < 0.8f) {
            force = 1000.0f * sin((progress - 0.2f) / 0.6f * 3.14159f);
        }
        
        // Обучаем предиктор
        predictor.learn(paddleQuat, kayakQuat, angularVelocity, force, true);
        learnCount++;
        
        // Предсказываем через 200мс
        if (i > 5) { // Начинаем предсказывать после нескольких семплов
            float confidence = 0.0f;
            float predictedForce = predictor.predict(
                paddleQuat, kayakQuat, angularVelocity,
                predictor.config.predictionTime,
                true, // forward
                true, // use acceleration
                &confidence
            );
            predictCount++;
            
            if (i % 5 == 0) { // Выводим каждый 5-й семпл
                Serial.printf("Step %d: angle=%.1f°, force=%.1f, predicted=%.1f, conf=%.2f\n",
                             i, angle, force, predictedForce, confidence);
            }
        }
        
        delay(10); // Небольшая задержка для симуляции
    }
    
    Serial.println("\n");
}

void testPrediction() {
    Serial.println("=== Testing Predictions ===");
    
    // Тестируем предсказание в разных положениях
    // Ожидаемые значения основаны на sin-кривой из симуляции
    struct TestCase {
        float angle;
        const char* description;
    };
    
    TestCase tests[] = {
        {0.0f, "Vertical (no force expected)"},
        {15.0f, "Early stroke"},
        {30.0f, "Mid stroke"},
        {45.0f, "Late stroke"},
        {60.0f, "End stroke"},
    };
    
    Serial.println("Testing without time offset (current position):");
    for (const auto& test : tests) {
        SP_Math::Vector axis(1, 0, 0);
        SP_Math::Quaternion testPaddleQuat = SP_Math::Quaternion::fromAxisAngle(
            axis, test.angle * 0.0174533f
        );
        
        // Предсказание для текущего положения (deltaT = 0)
        float confidence = 0.0f;
        float predictedForce = predictor.predict(
            testPaddleQuat, kayakQuat, SP_Math::Vector(0, 0, 0),
            0.0f, true, false, &confidence
        );
        
        Serial.printf("  %.1f° (%s): force=%.1f, conf=%.2f\n",
                     test.angle, test.description, predictedForce, confidence);
    }
    
    Serial.println("\nTesting with 200ms prediction:");
    for (const auto& test : tests) {
        SP_Math::Vector axis(1, 0, 0);
        SP_Math::Quaternion testPaddleQuat = SP_Math::Quaternion::fromAxisAngle(
            axis, test.angle * 0.0174533f
        );
        
        // Предсказание через 200мс с угловой скоростью
        float confidence = 0.0f;
        float predictedForce = predictor.predict(
            testPaddleQuat, kayakQuat, SP_Math::Vector(1, 0, 0),
            200.0f, true, true, &confidence
        );
        
        Serial.printf("  %.1f° (%s): force=%.1f, conf=%.2f\n",
                     test.angle, test.description, predictedForce, confidence);
    }
    
    Serial.println("\n");
}

void loop() {
    static int testPhase = 0;
    static unsigned long lastTestTime = 0;
    
    unsigned long currentTime = millis();
    
    // Выполняем тесты с задержкой
    if (currentTime - lastTestTime > 3000) {
        lastTestTime = currentTime;
        
        switch (testPhase) {
            case 0:
                Serial.println("Phase 1: Stroke 1-3 (Initial learning)");
                for (int i = 0; i < 3; i++) {
                    Serial.printf("  Stroke %d\n", i + 1);
                    simulateStroke();
                }
                predictor.printGridStatistics();
                testPhase++;
                break;
                
            case 1:
                Serial.println("Phase 2: Stroke 4-6 (Building confidence)");
                for (int i = 0; i < 3; i++) {
                    Serial.printf("  Stroke %d\n", i + 4);
                    simulateStroke();
                }
                predictor.printGridStatistics();
                testPhase++;
                break;
                
            case 2:
                Serial.println("Phase 3: Stroke 7-10 (Refinement)");
                for (int i = 0; i < 4; i++) {
                    Serial.printf("  Stroke %d\n", i + 7);
                    simulateStroke();
                }
                predictor.printGridStatistics();
                testPhase++;
                break;
                
            case 3:
                testPrediction();
                testPhase++;
                break;
                
            case 4:
                // Финальная статистика
                Serial.println("\n=== Final Statistics ===");
                Serial.printf("Total strokes: 10\n");
                Serial.printf("Total learn calls: %u\n", learnCount);
                Serial.printf("Total predict calls: %u\n", predictCount);
                Serial.printf("Memory used: ~%u KB\n", 
                             predictor.getFilledCellsCount() * 24 / 1024);
                predictor.printGridStatistics();
                
                Serial.println("\n=== Test Complete ===");
                Serial.println("Commands:");
                Serial.println("  'r' - reset and run again");
                Serial.println("  's' - show statistics");
                Serial.println("  'a' - switch to learning method A");
                Serial.println("  'b' - switch to learning method B");
                testPhase++;
                break;
                
            default:
                // Ожидание команд
                if (Serial.available() > 0) {
                    char cmd = Serial.read();
                    if (cmd == 'r' || cmd == 'R') {
                        Serial.println("\nResetting...\n");
                        predictor.clearGrid();
                        learnCount = 0;
                        predictCount = 0;
                        testPhase = 0;
                    } else if (cmd == 's' || cmd == 'S') {
                        predictor.printGridStatistics();
                    } else if (cmd == 'a' || cmd == 'A') {
                        predictor.learningMethod = LearningMethod::METHOD_A;
                        Serial.println("Switched to Learning Method A (error correction)");
                    } else if (cmd == 'b' || cmd == 'B') {
                        predictor.learningMethod = LearningMethod::METHOD_B;
                        Serial.println("Switched to Learning Method B (direct weighted EMA)");
                    }
                }
                break;
        }
    }
    
    delay(100);
}

