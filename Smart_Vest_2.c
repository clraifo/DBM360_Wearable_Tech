#include <Wire.h>
#include <Stepper.h>

// Pin definitions
const int ledPin = 6;    // LED strip
const int ledPin2 = 7;   // Additional LED 1
const int ledPin3 = 8;   // Additional LED 2
const int buzzerPin = 3; // Buzzer
const int motorPin = 5;  // Vibration motor (to be replaced by stepper motor)
const int lightSensorPin = A0; // Ambient light sensor (LDR)
const int trigPin = 9;   // Ultrasonic sensor trigger
const int echoPin = 10;  // Ultrasonic sensor echo

// Stepper motor control pins
const int stepperPin1 = 11; // Stepper motor pin 1
const int stepperPin2 = 12; // Stepper motor pin 2

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

// Stepper motor setup
const int stepsPerRevolution = 200; // Steps per revolution of the motor (adjust as needed)
Stepper myStepper(stepsPerRevolution, stepperPin1, stepperPin2);

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize pins
  pinMode(ledPin, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize the stepper motor
  myStepper.setSpeed(60); // Set the speed of the stepper motor

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
      
    // Activate the buzzer and stepper motor as vibration motor
    digitalWrite(buzzerPin, HIGH);
    myStepper.step(stepsPerRevolution / 2); // Rotate stepper motor 180 degrees
    delay(2000); // Delay to avoid repeated triggers
    digitalWrite(buzzerPin, LOW);
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
    analogWrite(ledPin, 255); // Maximum brightness for LED strip
    digitalWrite(ledPin2, HIGH); // Turn on additional LED 1
    digitalWrite(ledPin3, HIGH); // Turn on additional LED 2
  } else {
    analogWrite(ledPin, 128); // Dimmed for brighter conditions for LED strip
    digitalWrite(ledPin2, LOW); // Turn off additional LED 1
    digitalWrite(ledPin3, LOW); // Turn off additional LED 2
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
