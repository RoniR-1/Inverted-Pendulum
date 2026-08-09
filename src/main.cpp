#include <Arduino.h>

// put function declarations here:
long moniter_speed = 115200;

#include <Wire.h>

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

void setup() {
  Serial.begin(moniter_speed);
  Wire.begin();
}

void loop() {
  uint16_t rawAngle = readRawAngle();
  float degrees = (rawAngle / 4096.0) * 360.0;
  
  Serial.print("Angle: ");
  Serial.print(degrees);
  
  delay(100);
}