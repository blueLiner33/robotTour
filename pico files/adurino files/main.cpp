#include <Wire.h>
#include <SPI.h>
#include "Adafruit_BNO08x.h"
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

// Create BNO08x object for SPI
Adafruit_BNO08x bno08x = Adafruit_BNO08x();

// Define SPI pins
#define BNO08X_CS 10   // Chip select
#define BNO08X_INT 9   // Interrupt pin
#define BNO08X_RST 8   // Reset pin (optional)

// Polling interval and retry rate
unsigned long lastPollTime = 0;
unsigned long pollInterval = 100;
unsigned long retryTimeout = 100;

void encoderISR1() {
  encoderCount1++;
  lastTime1 = micros();
}

void encoderISR2() {
  encoderCount2++;
  lastTime2 = micros();
}

void setup() {
  // Initialize Serial1 for output
  Serial1.begin(115200);

  // Initialize SPI for BNO08x
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_RST, BNO08X_INT)) {
    while (1) {
      Serial1.println("BNO08x SPI init failed");
      delay(10);
    }
  }

  // Initialize motor encoders
  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);
  pinMode(ENCODER_A2, INPUT_PULLUP);
  pinMode(ENCODER_B2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoderISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), encoderISR2, RISING);

  // Confirm successful BNO08x setup
  Serial1.println("BNO08x initialized with SPI");
}

void loop() {
  unsigned long currentTime1 = micros();
  unsigned long currentTime2 = micros();

  // Calculate RPM for motor 1
  if (currentTime1 != lastTime1) {
    RPM1 = 60000000.0 / (currentTime1 - lastTime1);
  } else {
    RPM1 = 0;
  }

  // Calculate RPM for motor 2
  if (currentTime2 != lastTime2) {
    RPM2 = 60000000.0 / (currentTime2 - lastTime2);
  } else {
    RPM2 = 0;
  }

  unsigned long currentMillis = millis();
  if (currentMillis - lastPollTime >= pollInterval) {
    lastPollTime = currentMillis;

    // Read BNO08x data
    Adafruit_BNO08x::AccelerometerReport accelReport;
    if (bno08x.getAccelReport(&accelReport)) {
      // Print data
      Serial1.print(RPM1); // Motor 1 RPM
      Serial1.print(",");
      Serial1.print(degreesPerCount * encoderCount1); // Motor 1 degrees
      Serial1.print(",");
      Serial1.print(RPM2); // Motor 2 RPM
      Serial1.print(",");
      Serial1.print(degreesPerCount * encoderCount2); // Motor 2 degrees

      // Print BNO08x data
      Serial1.print(",");
      Serial1.print(accelReport.x); // X acceleration
      Serial1.print(",");
      Serial1.print(accelReport.y); // Y acceleration
      Serial1.print(",");
      Serial1.println(accelReport.z); // Z acceleration
    } else {
      Serial1.println("BNO08x read failed");
    }
  }

  delay(100);
}
