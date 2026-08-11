#include <Arduino.h>

// put function declarations here:
long moniter_speed = 115200;

#include <Wire.h>

#define DIR_PIN    2
#define STEP_PIN   3
#define EN_PIN     8

#define AS5600_ADDR 0x36 // device address
#define RAW_ANGLE_MSB 0x0C // register address

// reads the raw angle
uint16_t readRawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_MSB);
  Wire.endTransmission();
  
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() >= 2) {
    uint8_t highByte = Wire.read();
    uint8_t lowByte  = Wire.read();

    return ((highByte & 0x0F) << 8) | lowByte;
  }
  return 0;
}

// moves the motor microsteps times
// clockwise when direction is true, counterclockwise when false
void stepMotor(int microSteps, bool direction) {
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);

  for (int i = 0; i < microSteps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(50);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(50);
  }
}

void setup() {
  Serial.begin(moniter_speed);
  Wire.begin();

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);
}

void loop() {
  uint16_t rawAngle = readRawAngle();
  float degrees = (rawAngle / 4096.0) * 360.0;
  
  //Serial.print("Angle: ");
  //Serial.print(degrees);
  
  stepMotor(100, true);

  // Read AS5600 angle every 50 steps
  static int stepCounter = 0;
  if (++stepCounter >= 50) {
    stepCounter = 0;
    uint16_t rawAngle = readRawAngle();
    float degrees = (rawAngle / 4096.0) * 360.0;

    Serial.print("Encoder Angle: ");
    Serial.print(degrees);
    Serial.println("°");
  }
  
  delay(5);
  
}