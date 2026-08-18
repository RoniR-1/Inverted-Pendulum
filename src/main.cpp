#include <ARDUINO.h>

#define TEST_PIN    9  // The pin connected to the destroyed driver (8, 9, or 10)
#define LISTEN_PIN  4  // An unused pin listening for the signal

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  pinMode(TEST_PIN, OUTPUT);
  pinMode(LISTEN_PIN, INPUT);

  Serial.println("\n--- STARTING ARDUINO PIN DIAGNOSTIC ---");

  // Test 1: Driving Pin HIGH
  digitalWrite(TEST_PIN, HIGH);
  delay(10);
  bool highPassed = (digitalRead(LISTEN_PIN) == HIGH);

  // Test 2: Driving Pin LOW
  digitalWrite(TEST_PIN, LOW);
  delay(10);
  bool lowPassed = (digitalRead(LISTEN_PIN) == LOW);

  // Results
  Serial.print("Testing Pin ");
  Serial.print(TEST_PIN);
  Serial.println(":");

  if (highPassed && lowPassed) {
    Serial.println(" RESULT: [PASS] - Pin is 100% healthy!");
  } else {
    Serial.println(" RESULT: [FAIL] - Pin was damaged by the driver short.");
  }
}

void loop() {
  // Nothing here
}