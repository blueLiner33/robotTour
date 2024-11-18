//what works right now
#include <Wire.h>
#include "Adafruit_BNO08x_RVC.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include <Arduino.h>

// Motor encoders
#define ENCODER_A1 12
#define ENCODER_B1 11
#define ENCODER_A2 6
#define ENCODER_B2 7

volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;
volatile unsigned long lastTime1 = 0;
volatile unsigned long lastTime2 = 0;

#define GEAR_RATIO (244904.0 / 12000.0)
#define CPR 14
#define COUNTS_PER_REV (CPR * GEAR_RATIO)

float RPM1 = 0;
float RPM2 = 0;
float degreesPerCount = 360.0 / COUNTS_PER_REV;

// Create rvc object
Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();

unsigned long lastPollTime = 0;  // Time tracking for sensor polling
unsigned long pollInterval = 200; // Poll every 500ms (adjust as needed)
unsigned long retryTimeout = 100; // Timeout for retrying RVC read (in ms)

void encoderISR1() {
  encoderCount1++;
  lastTime1 = micros(); 
}

void encoderISR2() {
  encoderCount2++;
  lastTime2 = micros(); 
}

void setup() {
  Serial.begin(115200);          // USB serial
  Serial1.begin(115200);         // Use Serial1 for the BNO08x sensor
  Serial2.setTX(8);              // Set TX pin for Serial2 (UART1)
  Serial2.setRX(9);              // Set RX pin for Serial2 (UART1)
  Serial2.begin(115200);         // Initialize Serial2 for secondary device communication

  if (!rvc.begin(&Serial1)) {
    Serial.println("Could not find BNO08x!");
    while (1) delay(10);
  }

  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);
  pinMode(ENCODER_A2, INPUT_PULLUP);
  pinMode(ENCODER_B2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoderISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), encoderISR2, RISING);
}

void loop() {
  unsigned long currentTime1 = micros();
  unsigned long currentTime2 = micros();

  // Calculate RPM for motor 1
  if (currentTime1 != lastTime1) {  // Avoid division by zero
    RPM1 = 60000000.0 / (currentTime1 - lastTime1);
  } else {
    RPM1 = 0; // No new pulses since last reading
  }

  // Calculate RPM for motor 2
  if (currentTime2 != lastTime2) {
    RPM2 = 60000000.0 / (currentTime2 - lastTime2);
  } else {
    RPM2 = 0; // No new pulses since last reading
  }

  // Check if it's time to poll the BNO08x sensor
  unsigned long currentMillis = millis();
  if (currentMillis - lastPollTime >= pollInterval) {
    lastPollTime = currentMillis; // Update last poll time

    BNO08x_RVC_Data heading;
    bool success = false;
    unsigned long startTime = millis();

    // Retry loop for BNO08x read
    while (!success && (millis() - startTime < retryTimeout)) {
      if (rvc.read(&heading)) {
        success = true;

        // Print all data in a single line as CSV
        Serial2.print(RPM1);                  // Motor 1 RPM
        Serial2.print(",");
        Serial2.print(degreesPerCount * encoderCount1);  // Motor 1 degrees
        Serial2.print(",");
        Serial2.print(RPM2);                  // Motor 2 RPM
        Serial2.print(",");
        Serial2.print(degreesPerCount * encoderCount2);  // Motor 2 degrees

        // Print BNO08x sensor data
        Serial2.print(",");
        Serial2.print(heading.yaw);          // Yaw
        Serial2.print(",");
        Serial2.print(heading.x_accel);      // X acceleration
        Serial2.print(",");
        Serial2.print(heading.y_accel);      // Y acceleration
        Serial2.print(",");
        Serial2.println(heading.z_accel);    // Z acceleration

        // Print the same data to USB serial (for monitoring)
        Serial.print(RPM1);                  // Motor 1 RPM
        Serial.print(",");
        Serial.print(degreesPerCount * encoderCount1);  // Motor 1 degrees
        Serial.print(",");
        Serial.print(RPM2);                  // Motor 2 RPM
        Serial.print(",");
        Serial.print(degreesPerCount * encoderCount2);  // Motor 2 degrees

        // Print BNO08x sensor data
        Serial.print(",");
        Serial.print(heading.yaw);          // Yaw
        Serial.print(",");
        Serial.print(heading.x_accel);      // X acceleration
        Serial.print(",");
        Serial.print(heading.y_accel);      // Y acceleration
        Serial.print(",");
        Serial.println(heading.z_accel);    // Z acceleration
      } else {
        // If failed, give some feedback
        Serial.println("Retrying BNO08x read...");
      }
    }

    if (!success) {
      Serial.println("RVC read failed after retries.");
      Serial2.println("RVC read failed after retries.");
    }
  }

  // Delay to avoid overloading the loop
  delay(100);  // Adjust to slow down loop if necessary
}
