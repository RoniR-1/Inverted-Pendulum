#include <Arduino.h>
#include <Wire.h>

#define TOGGLE_PIN   2 
#define CALIBAR_PIN  7 
#define DIR_PIN      8
#define STEP_PIN     9   // Port B, Pin 1 on ATmega328P (Uno/Nano)
#define EN_PIN       10

#define AS5600_ADDR   0x36
#define RAW_ANGLE_MSB 0x0C
#define STATUS_REG    0x0B

long moniter_speed = 115200;
volatile uint8_t hault = 0;

// ============================================================================
//                         PHYSICAL HARDWARE LIMITS
// ============================================================================
// Locked at 1/8 Microstepping (MS1=LOW, MS2=LOW on TMC2209)
const float STEPS_PER_CM = 500.0;        // 50,000 steps per meter
const float MAX_RAIL_LENGTH_CM = 90.0; 
const long MAX_STEPS = MAX_RAIL_LENGTH_CM * STEPS_PER_CM; // 45,000 steps total
const long SAFETY_BUFFER_STEPS = 2000;   // 4 cm safety zone at each end (reduce usable range to ~82 cm)
const long MIN_SAFE_STEPS = SAFETY_BUFFER_STEPS;           // Left boundary
const long MAX_SAFE_STEPS = MAX_STEPS - SAFETY_BUFFER_STEPS; // Right boundary

// ============================================================================
//          HIGH-SPEED & HIGH-ACCELERATION TUNING PARAMETERS (m/s)
// ============================================================================
// Pendulum Target & Safety
float targetAngle   =  194.77;     // Will be updated dynamically during calibration
const float safeMargin = 25.0;    // Shut down motor if tilt > 25 deg
float deadbandAngle = 1.0f;       // 1° deadband - ensures motor responds to small errors

// PID Gains (Tuned for responsive control near equilibrium)
float Kp = 100.0;   // Strong proportional gain - respond quickly to angle
float Ki = 0.03f;  // Very small integral - only for steady-state error
float Kd = 3.5f;   // Derivative damping to prevent overshoot

// Motion Control Limits (Conservative to prevent stalling)
float speedMultiplier       = 15000.0f; // 1° error = ~3000 steps/sec (conservative)
float lowAngleBoostExponent = 1.0f;    // Linear scaling - predictable
float maxAccelPerLoop       = 1000.0f;  // Slow ramp: 500 steps/loop = 4 m/s^2 (was 2000!)
float minSpeedStepsSec      = 400.0f;  // Higher min speed to overcome static friction
float maxSpeedStepsSec      = 9500.0f; // ~16 cm/s (conservative max)
// ============================================================================

// Tracking & Timing Variables
long currentPositionSteps = 0;
float currentStepsPerSec = 0.0f;

float errorAngle = 0.0;
float lastError = 0.0;
float integral = 0.0;
const float maxIntegral = 50.0;

// Derivative filtering (low-pass filter to reject encoder jitter)
float filteredDerivative = 0.0f;
const float derivativeLPF = 0.25f; // Alpha: 0.1-0.3 range

// Cascaded PID: Position outer loop DISABLED (set to near-zero to let angle loop dominate)
long centerPositionSteps = MAX_STEPS / 2;  // 22500 steps = 45 cm center
float positionError = 0.0f;
float lastPositionError = 0.0f;
float positionIntegral = 0.0f;

float posKp = 0.00012f;   // DISABLED: Position control interferes with balance
float posKi = 0.0f;   // DISABLED
float posKd = 0.0f;   // DISABLED
const float maxPositionIntegral = 0.0f;
float targetAngleDynamic = 192.67f; // Will be static (no position modulation)

unsigned long lastLoopTime = 0;
const unsigned long sampleTimeUs = 2500; // 2.5 ms loop frame (400 Hz)

volatile unsigned long lastInteruptTime = 0;
int differenceTimeAllowed = 200;

// Direct Port I/O macros for Pin 9 (Fast step pulsing)
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  #define STEP_HIGH() (PORTB |= 0x02)  // Pin 9 HIGH
  #define STEP_LOW()  (PORTB &= ~0x02) // Pin 9 LOW
#else
  #define STEP_HIGH() digitalWrite(STEP_PIN, HIGH)
  #define STEP_LOW()  digitalWrite(STEP_PIN, LOW)
#endif

void handleInterupts() {
  if (millis() - lastInteruptTime > differenceTimeAllowed) {
    hault = (hault == 0) ? 1 : 0;
    lastInteruptTime = millis();
  }
}

uint8_t readStatus() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(STATUS_REG);
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 1);
  if (Wire.available()) return Wire.read();
  return 0;
}

float readAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_MSB);
  if (Wire.endTransmission() != 0) {
    return -1.0f; 
  }
  
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() >= 2) {
    uint8_t highByte = Wire.read();
    uint8_t lowByte  = Wire.read();
    uint16_t angle = ((highByte & 0x0F) << 8) | lowByte;
    return (angle / 4096.0) * 360.0;
  }
  return -1.0f;
}

void driveMotor(float pidOutput) {
  float absOutput = abs(pidOutput);

  // 1. Calculate target speed with scaling
  float targetSpeed = 0.0f;

  if (absOutput >= deadbandAngle) {
    bool requestedDir = (pidOutput > 0);
    float boostedOutput = pow(absOutput, lowAngleBoostExponent);
    
    targetSpeed = constrain(boostedOutput * speedMultiplier, minSpeedStepsSec, maxSpeedStepsSec);
    if (!requestedDir) {
      targetSpeed = -targetSpeed;
    }
  } else {
    targetSpeed = 0.0f; // Smooth zero-speed in deadband
  }

  // 2. Smooth acceleration ramp toward targetSpeed
  if (targetSpeed > currentStepsPerSec) {
    currentStepsPerSec += maxAccelPerLoop;
    if (currentStepsPerSec > targetSpeed) currentStepsPerSec = targetSpeed;
  } else if (targetSpeed < currentStepsPerSec) {
    currentStepsPerSec -= maxAccelPerLoop;
    if (currentStepsPerSec < targetSpeed) currentStepsPerSec = targetSpeed;
  }

  if (abs(currentStepsPerSec) < 0.5f) {
    currentStepsPerSec = 0.0f;
    return;
  }

  // 3. Direction and hard rail boundaries with power cutoff
  bool currentDir = (currentStepsPerSec > 0.0f);

  // SAFETY: Cut motor if at or past safe boundaries
  if (currentPositionSteps <= MIN_SAFE_STEPS) { 
    digitalWrite(EN_PIN, HIGH); // Cut motor power at left limit
    currentStepsPerSec = 0.0f; 
    return; 
  }
  if (currentPositionSteps >= MAX_SAFE_STEPS) { 
    digitalWrite(EN_PIN, HIGH); // Cut motor power at right limit
    currentStepsPerSec = 0.0f; 
    return; 
  }
  
  // Also prevent moving TOWARD a boundary that's close
  if (!currentDir && currentPositionSteps - (int)(abs(currentStepsPerSec) * 0.0025f) <= MIN_SAFE_STEPS) {
    digitalWrite(EN_PIN, HIGH);
    currentStepsPerSec = 0.0f;
    return;
  }
  if (currentDir && currentPositionSteps + (int)(abs(currentStepsPerSec) * 0.0025f) >= MAX_SAFE_STEPS) {
    digitalWrite(EN_PIN, HIGH);
    currentStepsPerSec = 0.0f;
    return;
  }
  
  digitalWrite(DIR_PIN, currentDir ? HIGH : LOW);

  float absSpeed = abs(currentStepsPerSec);

  // 4. EVEN PULSE DISTRIBUTION ACROSS 2.5MS WINDOW
  int stepsToTake = (int)(absSpeed * 0.0025f);
  if (stepsToTake < 1) stepsToTake = 1;

  uint16_t pulseIntervalUs = 2500 / stepsToTake; 

  // 5. High-speed pulse loop using direct port I/O
  for (int i = 0; i < stepsToTake; i++) {
    if (currentPositionSteps <= MIN_SAFE_STEPS || currentPositionSteps >= MAX_SAFE_STEPS) break;

    STEP_HIGH();
    delayMicroseconds(2); // Driver pulse width
    STEP_LOW();
    
    if (pulseIntervalUs > 2) {
      delayMicroseconds(pulseIntervalUs - 2);
    }

    if (currentDir) {
      currentPositionSteps++;
    } else {
      currentPositionSteps--;
    }
  }
}

bool calibrate() {
  Serial.println("Homing to calibration switch...");
  
  digitalWrite(DIR_PIN, LOW); 
  while(digitalRead(CALIBAR_PIN) != HIGH) {
    STEP_HIGH();
    delayMicroseconds(80);
    STEP_LOW();
    delayMicroseconds(80);
  }

  currentPositionSteps = 0;
  Serial.println("Homed at 0 cm!");

  Serial.println("Moving carriage to center (45 cm)...");
  digitalWrite(DIR_PIN, HIGH); 
  
  long targetCenterSteps = MAX_STEPS / 2;
  while (currentPositionSteps < targetCenterSteps) {
    STEP_HIGH();
    delayMicroseconds(80);
    STEP_LOW();
    delayMicroseconds(80);
    currentPositionSteps++;
  }

  Serial.println("Carriage centered at 45 cm.");
  Serial.println("=================================================");
  Serial.println(">>> ANGLE CALIBRATION WINDOW <<<");
  Serial.println("Hold pendulum perfectly upright.");
  Serial.println("Press the CALIBRATION switch or TOGGLE button to lock target angle...");
  Serial.println("=================================================");

  // Wait for switch/button release if currently pressed
  delay(500); 

  unsigned long lastPrintTime = 0;

  // Wait for user to trigger CALIBAR_PIN or TOGGLE_PIN
  while (digitalRead(CALIBAR_PIN) == LOW && digitalRead(TOGGLE_PIN) == LOW) {
    if (millis() - lastPrintTime > 200) {
      float liveAngle = readAngle();
      Serial.print("Live Raw Angle: ");
      Serial.print(liveAngle, 2);
      Serial.println(" deg (Waiting for button press...)");
      lastPrintTime = millis();
    }
    delay(10);
  }

  // Sample vertical center over 50 readings (250ms) for high accuracy
  Serial.println("Sampling zero-angle offset...");
  float sumAngle = 0.0f;
  int sampleCount = 50;
  for (int i = 0; i < sampleCount; i++) {
    sumAngle += readAngle();
    delay(5);
  }

  targetAngle = sumAngle / (float)sampleCount;
  targetAngleDynamic = targetAngle;

  Serial.print("SUCCESS! Target Vertical Angle set to: ");
  Serial.print(targetAngle, 2);
  Serial.println(" deg");
  Serial.println("Balancing loop starting in 1 second...");
  delay(1000);

  return true;
}



void setup() {
  Serial.begin(moniter_speed);
  
  Wire.begin();
  Wire.setClock(400000); // Fast I2C Bus (400kHz)
  Wire.setWireTimeout(3000, true);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(TOGGLE_PIN, INPUT);
  pinMode(CALIBAR_PIN, INPUT);

  digitalWrite(EN_PIN, LOW); // Enable driver
  
  attachInterrupt(digitalPinToInterrupt(TOGGLE_PIN), handleInterupts, RISING);

  uint8_t status = readStatus();
  if (status & 0x20) Serial.println("AS5600: Magnet detected!");
  if (status & 0x10) Serial.println("AS5600: Magnet TOO FAR");
  if (status & 0x08) Serial.println("AS5600: Magnet TOO CLOSE");

  calibrate();
  lastLoopTime = micros();
}

void loop() {
  if (hault > 0) {
    digitalWrite(EN_PIN, HIGH); 
    integral = 0;
    positionIntegral = 0;
    lastError = 0;
    lastPositionError = 0;
    filteredDerivative = 0.0f;
    currentStepsPerSec = 0.0f;
    delay(100);
    return;
  }

  // 2.5 ms high-speed sample loop (400 Hz)
  unsigned long currentTime = micros();
  if (currentTime - lastLoopTime < sampleTimeUs) return;
  
  float dt = (currentTime - lastLoopTime) / 1000000.0f;
  lastLoopTime = currentTime;

  float realAngle = readAngle();
  if (realAngle < 0) return; 

  // ========== OUTER LOOP: Position Control (Cascaded PID) ==========
  positionError = centerPositionSteps - currentPositionSteps;
  
  // Position P term
  float posP = posKp * positionError;
  
  // Position I term (anti-windup)
  positionIntegral += positionError * dt;
  positionIntegral = constrain(positionIntegral, -maxPositionIntegral, maxPositionIntegral);
  float posI = posKi * positionIntegral;
  
  // Position D term (smooth derivative)
  float posDerivative = (positionError - lastPositionError) / dt;
  float posD = posKd * posDerivative;
  lastPositionError = positionError;
  
  // Position correction to angle setpoint (max ±3 degrees adjustment)
  float positionCorrection = constrain(posP + posI + posD, -3.0f, 3.0f);
  targetAngleDynamic = targetAngle + positionCorrection;

  // ========== INNER LOOP: Angle Balance (High-speed 400 Hz) ==========
  errorAngle = targetAngleDynamic - realAngle;
  if (errorAngle < -180.0) errorAngle += 360.0;
  if (errorAngle > 180.0)  errorAngle -= 360.0;

  if (abs(errorAngle) > safeMargin) {
    digitalWrite(EN_PIN, HIGH); 
    integral = 0;
    filteredDerivative = 0.0f;
    currentStepsPerSec = 0.0f;
    return;
  } else {
    digitalWrite(EN_PIN, LOW);  
  }

  // Proportional term
  float P = Kp * errorAngle;

  // Integral term
  integral += errorAngle * dt;
  integral = constrain(integral, -maxIntegral, maxIntegral);
  float I = Ki * integral;

  // Derivative term with LOW-PASS FILTER (rejects encoder noise)
  float rawDerivative = (errorAngle - lastError) / dt;
  filteredDerivative = (derivativeLPF * rawDerivative) + ((1.0f - derivativeLPF) * filteredDerivative);
  float D = Kd * filteredDerivative;
  lastError = errorAngle;

  float pidOutput = P + I + D;

  driveMotor(pidOutput);
}