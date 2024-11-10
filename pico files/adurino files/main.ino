#include <Wire.h>
#include <Adafruit_BNO08x.h>
//format M:leftMotorPos,rightMotorPos,x,y,qw,qx,qy,qz

// Create BNO08x object
Adafruit_BNO08x bno08x;

// Motor encoder positions (these are placeholders for actual encoder reading logic)
int leftMotorPos = 0;
int rightMotorPos = 0;

// Assuming the motors increment their positions (you can replace this with real encoder reading logic)
void updateMotorPositions() {
  leftMotorPos += 10; // Example increment
  rightMotorPos += 10; // Example increment
}

void setup() {
  Serial.begin(115200); // Start UART communication at 115200 baud rate

  // Initialize BNO08x
  if (!bno08x.begin()) {
    Serial.println("Failed to find BNO08x sensor!");
    while (1);
  }

  // Enable the rotation vector report (used for orientation data)
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector report!");
    while (1);
  }

  // Allow some time for the sensor to start up
  delay(100);
}

void loop() {
  sh2_SensorValue_t sensorValue;
  if (bno08x.getSensorEvent(&sensorValue)) {
    // Retrieve the quaternion data from BNO08x for orientation tracking
    float qw = sensorValue.un.rotationVector.real;
    float qx = sensorValue.un.rotationVector.i;
    float qy = sensorValue.un.rotationVector.j;
    float qz = sensorValue.un.rotationVector.k;

    // Calculate X and Y based on the robot's movement
    // For simplicity, we'll use a basic assumption of movement
    float x = leftMotorPos * 0.1;  // Example X movement, scale as necessary
    float y = rightMotorPos * 0.1; // Example Y movement, scale as necessary

    // Update motor positions (you'd replace this with real sensor feedback)
    updateMotorPositions();

    // Create and send the message via UART
    Serial.print("M:");
    Serial.print(leftMotorPos);   // Left motor position
    Serial.print(",");
    Serial.print(rightMotorPos);  // Right motor position
    Serial.print(",");
    Serial.print(x);              // X position (calculated from motors)
    Serial.print(",");
    Serial.print(y);              // Y position (calculated from motors)
    Serial.print(",");
    Serial.print(qw);             // Quaternion data for orientation
    Serial.print(",");
    Serial.print(qx);             
    Serial.print(",");
    Serial.print(qy);             
    Serial.print(",");
    Serial.println(qz);           // Quaternion data for orientation

    delay(100); // Delay to prevent flooding the UART interface
  }
}
