#include <Wire.h>
#include "Adafruit_BNO08x_RVC.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include <Arduino.h>
// pin notes
//encoder below 
//serial 2 for other pico
//TX (Transmit): GPIO 17
//RX (Receive): GPIO 16
//Board 3V to BNO085 Vin (Red Wire). 
//Board GND to BNO085 GND (Black Wire)
//Board RX to BNO085 SDA (Blue Wire)
//Board 3V to BNO085 P0 (Purple Wire
// Create rvc object
Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();

// Motor encoders
#define ENCODER_A1 12  // Encoder 1 A pin
#define ENCODER_B1 11  // Encoder 1 B pin
#define ENCODER_A2 6   // Encoder 2 A pin
#define ENCODER_B2 7   // Encoder 2 B pin

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

void encoderISR1() {
  encoderCount1++;
  lastTime1 = micros(); 
}

void encoderISR2() {
  encoderCount2++;
  lastTime2 = micros(); 
}

UART Serial2(8, 9, 0, 0);

void setup() {
  Serial.begin(115200); 
  Serial1.begin(115200);
  Serial2.begin(115200);

  if (!rvc.begin(&Serial1)) { // Connect to the sensor over hardware serial
    Serial.println("Could not find BNO08x!");
    while (1)
      delay(10);  // Infinite loop if the sensor is not found
  }

  pinMode(ENCODER_A1, INPUT_PULLUP);  // Set Encoder A1 pin
  pinMode(ENCODER_B1, INPUT_PULLUP);  // Set Encoder B1 pin
  pinMode(ENCODER_A2, INPUT_PULLUP);  // Set Encoder A2 pin
  pinMode(ENCODER_B2, INPUT_PULLUP);  // Set Encoder B2 pin

  // Attach interrupts for Encoder 1 and Encoder 2
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoderISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), encoderISR2, RISING);
}

void loop() {
  // Calculates RPM 
  unsigned long currentTime1 = micros();
  RPM1 = 60000000.0 / (currentTime1 - lastTime1); 
  
  // Calculates RPM 
  unsigned long currentTime2 = micros();
  RPM2 = 60000000.0 / (currentTime2 - lastTime2);  

  Serial2.print(RPM1);   // Encoder 1 rpm
  Serial2.print(",");
  Serial2.println(degreesPerCount * encoderCount1);   // Encoder 1 position in degrees
  
  Serial2.print(RPM2);        // Encoder 2 rpm
  Serial2.print(",");
  Serial2.println(degreesPerCount * encoderCount2);   // Encoder 2 position in degrees
  
  BNO08x_RVC_Data heading;
  if (!rvc.read(&heading)) {
    return;
  }
  
  // Sends data
  Serial2.print(heading.yaw);
  Serial2.print(",");
  Serial2.print(heading.x_accel);
  Serial2.print(",");
  Serial2.print(heading.y_accel);
  Serial2.print(",");
  Serial2.println(heading.z_accel);
//
  delay(100);
}