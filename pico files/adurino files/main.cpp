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

  if (currentTime1 != lastTime1) {  // Avoid division by zero
    RPM1 = 60000000.0 / (currentTime1 - lastTime1);
  } else {
    RPM1 = 0; // No new pulses since last reading
  }

  if (currentTime2 != lastTime2) {
    RPM2 = 60000000.0 / (currentTime2 - lastTime2);
  } else {
    RPM2 = 0; // No new pulses since last reading
  }

  Serial2.print(RPM1);
  Serial2.print(",");
  Serial2.println(degreesPerCount * encoderCount1);

  Serial2.print(RPM2);
  Serial2.print(",");
  Serial2.println(degreesPerCount * encoderCount2);

  BNO08x_RVC_Data heading;
  if (rvc.read(&heading)) { // Proceed only if read was successful
    Serial2.print(heading.yaw);
    Serial2.print(",");
    Serial2.print(heading.x_accel);
    Serial2.print(",");
    Serial2.print(heading.y_accel);
    Serial2.print(",");
    Serial2.println(heading.z_accel);
  } else {
    Serial2.println("RVC read failed");
  }

  delay(100);
}
