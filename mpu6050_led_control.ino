#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <Wire.h>
#include <MPU6050.h>

#define LED_PIN 2  // LED connected to pin 2 on Arduino Nano

MPU6050 mpu;  // MPU6050 sensor object

QueueHandle_t uartQueue;  // Queue for UART commands
int ledFrequency = 5;     // Default LED blink rate in Hz
bool ledOn = false;       // LED state

// RTOS Task Prototypes
void TaskLEDControl(void *pvParameters);
void TaskUARTCommunication(void *pvParameters);
void TaskSensorRead(void *pvParameters);

void setup() {
    Serial.begin(115200);       // Initialize UART
    pinMode(LED_PIN, OUTPUT);   // Set LED as output

    // Initialize MPU6050
    Wire.begin();
    mpu.initialize();
    
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);  // Stop execution if sensor fails
    }
    Serial.println("MPU6050 connected!");

    // Create Queue for UART commands
    uartQueue = xQueueCreate(5, sizeof(char[10]));

    // Create RTOS Tasks
    xTaskCreate(TaskLEDControl, "LED Control", 128, NULL, 1, NULL);
    xTaskCreate(TaskUARTCommunication, "UART Comm", 128, NULL, 1, NULL);
    xTaskCreate(TaskSensorRead, "Sensor Read", 128, NULL, 1, NULL);
}

void loop() {
    // Empty since RTOS manages tasks
}

// **LED Control Task**
void TaskLEDControl(void *pvParameters) {
    while (1) {
        if (ledOn) {
            digitalWrite(LED_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(500 / ledFrequency));
            digitalWrite(LED_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(500 / ledFrequency));
        } else {
            digitalWrite(LED_PIN, LOW);
        }
    }
}

// **UART Communication Task**
void TaskUARTCommunication(void *pvParameters) {
    char inputBuffer[10];
    while (1) {
        if (Serial.available()) {
            int len = Serial.readBytesUntil('\n', inputBuffer, sizeof(inputBuffer) - 1);
            inputBuffer[len] = '\0';  // Null terminate

            if (strcmp(inputBuffer, "LED_10Hz") == 0) {
                ledFrequency = 10;
                ledOn = true;
            } else if (strcmp(inputBuffer, "LED_5Hz") == 0) {
                ledFrequency = 5;
                ledOn = true;
            } else if (strcmp(inputBuffer, "LED_ON") == 0) {
                ledOn = true;
            } else if (strcmp(inputBuffer, "LED_OFF") == 0) {
                ledOn = false;
            }
        }
    }
}

// **MPU6050 Sensor Read Task**
void TaskSensorRead(void *pvParameters) {
    int16_t ax, ay, az, gx, gy, gz;

    while (1) {
        // Read sensor data
        mpu.getAcceleration(&ax, &ay, &az);
        mpu.getRotation(&gx, &gy, &gz);

        // Print values over UART
        Serial.print("ACCEL: ");
        Serial.print(ax); Serial.print(", ");
        Serial.print(ay); Serial.print(", ");
        Serial.print(az);
        Serial.print(" | GYRO: ");
        Serial.print(gx); Serial.print(", ");
        Serial.print(gy); Serial.print(", ");
        Serial.println(gz);

        vTaskDelay(pdMS_TO_TICKS(1000));  // Send data every 1 second
    }
}
