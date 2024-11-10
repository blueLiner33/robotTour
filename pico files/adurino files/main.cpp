#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "quadrature_encoder.pio"
// in the format of M1:1024,M2:2048,RPM1:120,RPM2:250,X:102.4,Y:204.8,qw:0.707,qx:0.707,qy:0,qz:0

// Create BNO08x object
Adafruit_BNO08x bno08x;

// Motor encoder positions
volatile int motorPos1 = 0;  // Encoder 1 position
volatile int motorPos2 = 0;  // Encoder 2 position
float rpm1 = 0.0;            // RPM for motor 1
float rpm2 = 0.0;            // RPM for motor 2
unsigned long lastTime = 0;  // For RPM calculation
PIO pio = pio0;
uint sm1 = 0; // State machine ID for encoder 1
uint sm2 = 1; // State machine ID for encoder 2
int pinA1 = 14; // Pin for encoder A1
int pinB1 = 15; // Pin for encoder B1
int pinA2 = 16; // Pin for encoder A2
int pinB2 = 17; // Pin for encoder B2

// Assembly code for quadrature encoder (from PIO program)
const uint16_t quadrature_encoder_program_instructions[] = {
    0x2000, 0x6800, 0x7001, 0x4000, 0x7001, 0x6000, 0x7001, 0x7001,
    0x5001, 0x5800, 0x7001, 0x3001, 0x2801, 0x1001, 0x4801, 0x7001
};

void setup() {
  Serial.begin(115200); // UART communication at 115200 baud rate

  // Initialize the BNO08x sensor
  if (!bno08x.begin()) {
    Serial.println("Failed to find BNO08x sensor!");
    while (1);
  }

  // Enable rotation vector report for orientation
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector report!");
    while (1);
  }

  // Initialize PIO for encoder 1
  pio_sm_config config1 = pio_get_default_sm_config();
  pio_sm_set_consecutive_pindirs(pio, sm1, pinA1, 2, false);
  gpio_pull_up(pinA1);
  gpio_pull_up(pinB1);
  pio_add_program(pio, &quadrature_encoder_program); // Load program into PIO
  quadrature_encoder_program_init(pio, sm1, pinA1, 0);  // Set up PIO state machine for encoder 1

  // Initialize PIO for encoder 2
  pio_sm_config config2 = pio_get_default_sm_config();
  pio_sm_set_consecutive_pindirs(pio, sm2, pinA2, 2, false);
  gpio_pull_up(pinA2);
  gpio_pull_up(pinB2);
  pio_add_program(pio, &quadrature_encoder_program); // Load program into PIO
  quadrature_encoder_program_init(pio, sm2, pinA2, 0);  // Set up PIO state machine for encoder 2

  // Allow some time for the sensor to start up
  delay(100);
}

void loop() {
  unsigned long currentTime = millis();
  // Calculate RPM every 1 second
  if (currentTime - lastTime >= 1000) {  // 1 second interval
    rpm1 = (motorPos1 * 60.0) / 1000;   // RPM = (Pulses in 1s * 60) / 1 second
    rpm2 = (motorPos2 * 60.0) / 1000;
    
    motorPos1 = 0;  // Reset pulse count for the next interval
    motorPos2 = 0;
    lastTime = currentTime;
  }

  // Read the encoder counts
  motorPos1 = quadrature_encoder_get_count(pio, sm1);
  motorPos2 = quadrature_encoder_get_count(pio, sm2);

  // Read data from BNO08x
  sh2_SensorValue_t sensorValue;
  if (bno08x.getSensorEvent(&sensorValue)) {
    float qw = sensorValue.un.rotationVector.real;
    float qx = sensorValue.un.rotationVector.i;
    float qy = sensorValue.un.rotationVector.j;
    float qz = sensorValue.un.rotationVector.k;

    // Calculate X, Y position based on encoder values (example scaling)
    float x = motorPos1 * 0.1; // Example scaling of motor position for X
    float y = motorPos2 * 0.1; // Example scaling for Y

    // Send the data over UART
    Serial.print("M1:"); 
    Serial.print(motorPos1);   // Encoder 1 position
    Serial.print(",");
    Serial.print("M2:"); 
    Serial.print(motorPos2);   // Encoder 2 position
    Serial.print(",");
    Serial.print("RPM1:"); 
    Serial.print(rpm1);        // RPM for motor 1
    Serial.print(",");
    Serial.print("RPM2:"); 
    Serial.print(rpm2);        // RPM for motor 2
    Serial.print(",");
    Serial.print(x);           // X position (calculated from motor 1 position)
    Serial.print(",");
    Serial.print(y);           // Y position (calculated from motor 2 position)
    Serial.print(",");
    Serial.print(qw);          // Quaternion data from BNO08x
    Serial.print(",");
    Serial.print(qx);
    Serial.print(",");
    Serial.print(qy);
    Serial.print(",");
    Serial.println(qz);        // Quaternion data from BNO08x

    // Wait a bit before sending the next data
    delay(100);
}

// PIO program for quadrature encoder (from your original code)
PIO_PROGRAM quadrature_encoder_program = {
  .program_instructions = quadrature_encoder_program_instructions,
  .size = sizeof(quadrature_encoder_program_instructions) / sizeof(uint16_t)
};
