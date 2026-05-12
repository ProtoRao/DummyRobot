constexpr uint8_t SERVO_PIN = 13;
constexpr uint16_t STEP_DELAY_MS = 20;
constexpr uint16_t HOLD_DELAY_MS = 800;
constexpr uint16_t SERVO_PERIOD_US = 20000;
constexpr uint16_t MIN_PULSE_US = 1000;
constexpr uint16_t MAX_PULSE_US = 2000;

uint16_t angleToPulseUs(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, MIN_PULSE_US, MAX_PULSE_US);
}

void writeServoAngle(int angle) {
  const uint16_t pulseUs = angleToPulseUs(angle);
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulseUs);
  digitalWrite(SERVO_PIN, LOW);
  delayMicroseconds(SERVO_PERIOD_US - pulseUs);
}

void holdServoAngle(int angle, uint16_t holdMs) {
  const unsigned long endTime = millis() + holdMs;
  while (millis() < endTime) {
    writeServoAngle(angle);
  }
}

void moveServoSmooth(int fromAngle, int toAngle) {
  if (fromAngle < toAngle) {
    for (int angle = fromAngle; angle <= toAngle; ++angle) {
      holdServoAngle(angle, STEP_DELAY_MS);
    }
  } else {
    for (int angle = fromAngle; angle >= toAngle; --angle) {
      holdServoAngle(angle, STEP_DELAY_MS);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Direct servo test on pin 13 starting...");

  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
  holdServoAngle(90, 1000);
}

void loop() {
  Serial.println("Sweeping to 0 degrees.");
  moveServoSmooth(90, 0);
  delay(HOLD_DELAY_MS);

  Serial.println("Sweeping to 180 degrees.");
  moveServoSmooth(0, 180);
  delay(HOLD_DELAY_MS);

  Serial.println("Returning to center.");
  moveServoSmooth(180, 90);
  delay(1500);
}
