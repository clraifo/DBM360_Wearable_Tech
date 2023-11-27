#include <Wire.h>

// Pin definitions
const int ledPin = 6; // LED strip
const int buzzerPin = 3; // Buzzer
const int motorPin = 5; // Vibration motor
const int lightSensorPin = A0; // Ambient light sensor (LDR)
const int trigPin = 9; // Ultrasonic sensor trigger
const int echoPin = 10; // Ultrasonic sensor echo

// MPU6050 I2C address (could be 0x68 or 0x69)
const int MPU_ADDR = 0x68;

// Variables to store accelerometer data
int16_t AccX, AccY, AccZ, Temp, GyroX, GyroY, GyroZ;

// Variables for ultrasonic sensor readings
long duration, distance;

// Threshold for fall or sudden stop detection
const int accelerationThreshold = 15000; // Adjust as needed

// Variables to store previous accelerometer readings
int16_t lastAccX = 0, lastAccY = 0, lastAccZ = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize pins
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize I2C communication for the MPU6050
  Wire.begin();

  // Wake up the MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU6050)
  Wire.endTransmission(true);
}

void loop() {
  // Read accelerometer data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // request a total of 14 registers

  // Store accelerometer and gyro data
  AccX = Wire.read() << 8 | Wire.read(); // ACCEL_XOUT
  AccY = Wire.read() << 8 | Wire.read(); // ACCEL_YOUT
  AccZ = Wire.read() << 8 | Wire.read(); // ACCEL_ZOUT
  Temp = Wire.read() << 8 | Wire.read(); // TEMP_OUT
  GyroX = Wire.read() << 8 | Wire.read(); // GYRO_XOUT
  GyroY = Wire.read() << 8 | Wire.read(); // GYRO_YOUT
  GyroZ = Wire.read() << 8 | Wire.read(); // GYRO_ZOUT

  // Detect a fall or sudden stop
  if (abs(AccX - lastAccX) > accelerationThreshold || 
      abs(AccY - lastAccY) > accelerationThreshold || 
      abs(AccZ - lastAccZ) > accelerationThreshold) {
      
    // Activate the buzzer and vibration motor
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(motorPin, HIGH);
    delay(2000); // Delay to avoid repeated triggers
    digitalWrite(buzzerPin, LOW);
    digitalWrite(motorPin, LOW);
  }

  // Update last accelerometer readings
  lastAccX = AccX;
  lastAccY = AccY;
  lastAccZ = AccZ;

  // Read ambient light sensor and adjust LEDs
  int lightLevel = analogRead(lightSensorPin);
  adjustLEDs(lightLevel);

  // Measure distance with ultrasonic sensor
  distance = measureDistance();
  if (distance < 50) { // Object closer than 50 cm
    digitalWrite(buzzerPin, HIGH);
  } else {
    digitalWrite(buzzerPin, LOW);
  }

  delay(100);
}

void adjustLEDs(int lightLevel) {
  // Adjust brightness of LEDs based on ambient light level
  if (lightLevel < 300) { // Low light condition
    analogWrite(ledPin, 255); // Maximum brightness
  } else {
    analogWrite(ledPin, 128); // Dimmed for brighter conditions
  }
}

long measureDistance() {
  // Trigger ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the duration of the echo pulse
  duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2; // Calculate distance in cm
  return distance;
}
