#include <Arduino.h>
#include <Wire.h>

#define TOGGLE_PIN  2 // HIGH toggles the state of motor
#define CALIBAR_PIN  7 // 
#define DIR_PIN   8
#define STEP_PIN  9
#define EN_PIN    10

#define AS5600_ADDR 0x36 // device address, A4 for data, A5 for clock as per Wire.h
#define RAW_ANGLE_MSB 0x0C // register address

long moniter_speed = 115200;
volatile uint16_t hault = 0;
uint16_t motorSpeed = 200;
const float targetAngle = 143.35;
const float safeMargin = 25;
const int availableSpace = 10; // in cm

unsigned long lastInteruptTime = 0;
int differenceTimeAllowed = 50;

// multiple interupts could fire from a single physical button/switch/etc
// to prevent this, interupts can at most fire every 50 miliseconds
void handleInterupts() {
  if (millis() - lastInteruptTime > differenceTimeAllowed) {
    if (hault == 0) hault++;
    else hault = 0;
    lastInteruptTime = millis();
  }
}


// reads the raw angle from AS5600, converts it to a range between 0 and 360
float readAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_MSB);
  Wire.endTransmission();
  
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() >= 2) {
    uint8_t highByte = Wire.read();
    uint8_t lowByte  = Wire.read();

    uint16_t angle = ((highByte & 0x0F) << 8) | lowByte; // raw angle
    return (angle / 4096.0) * 360.0; // real angle from 0 to 360
  }
  return -1.0f;
}

// moves the motor microsteps times
// clockwise when direction is true, counterclockwise when false
// the lower the speed, the faster it turns
void stepMotor(int microSteps, bool direction, uint16_t speed) {
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);

  for (int i = 0; i < microSteps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(speed);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(speed);
  }
}

/*move the carriage the away from the motor until a calibar pin is HIGH*/
bool calibrate() {
  while(digitalRead(CALIBAR_PIN) != HIGH) {
    stepMotor(16, true, 300);
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
  
  attachInterrupt(TOGGLE_PIN, handleInterupts, RISING);
  if (!calibrate()) {
    Serial.println("SOMETHING WENT WRONG STOP");
    hault++;
  }
}

void loop() {
  while(hault > 0); // hault the code

  float realAngle = readAngle();
  float errorAngle = targetAngle - realAngle;
  /*Imagine a target of 0 degrees and the reading is 355 degrees.
    The error would be huge, but its actually pretty close.
    To fix this, convert to [-180,180] range.
  */
  if (errorAngle < -180) errorAngle += 360;
  if (errorAngle > 180) errorAngle -= 360;

  if (abs(errorAngle) > safeMargin) {
    digitalWrite(EN_PIN, HIGH); // Disable motor output
    return;
  } else {
    digitalWrite(EN_PIN, LOW); // Re-enable motor
  }
  
  stepMotor(16 * errorAngle, errorAngle > 0 ? true : false, motorSpeed);
}
