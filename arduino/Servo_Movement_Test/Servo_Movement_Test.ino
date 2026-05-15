#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Arduino.h"
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

constexpr uint8_t SERVO_CHANNELS[] = {12, 13, 14, 10, 8};
constexpr int16_t INITIAL_ANGLES[] = {90, 90, -90, 0, 0};
constexpr int16_t LOGICAL_MIN_ANGLES[] = {0, 0, -150, -90, -90};
constexpr int16_t LOGICAL_MAX_ANGLES[] = {180, 180, 30, 90, 90};
constexpr bool CENTERED_POSITIVE_INCREASES_PWM[] = {false, false, false, false, true};
constexpr uint8_t SERVO_COUNT = sizeof(SERVO_CHANNELS) / sizeof(SERVO_CHANNELS[0]);

constexpr uint16_t SERVO_MIN_PULSE[] = {85, 100, 140, 150, 150};
constexpr uint16_t SERVO_MAX_PULSE[] = {475, 480, 500, 640, 640};
constexpr uint16_t STANDARD_CENTER_PULSE[] = {280, 280, 380, 395, 395};
constexpr int16_t CENTERED_RANGE_DEGREES = 90;
constexpr int16_t HALF_PULSE_RANGE[] = {235, 235, 235, 245, 245};

// Replaces SMOOTH_MIN_STEP_PULSE / SMOOTH_MAX_STEP_PULSE / SMOOTH_STEP_DELAY_MS
constexpr uint16_t SMOOTH_MOVE_DURATION_MS = 3000;  // total travel time in ms
constexpr uint8_t  SMOOTH_UPDATE_INTERVAL_MS = 20; // PWM update rate (~50 Hz)

// --- Motion state ---------------------------------------------------------
struct ServoMotion {
  uint16_t startPulse;
  uint16_t targetPulse;
  uint32_t startTime;
  uint32_t duration;
  bool     active;
};

ServoMotion servoMotions[SERVO_COUNT];

// --- API ------------------------------------------------------------------
void startServoMove(uint8_t servoIndex, uint16_t targetPulse,
                    uint16_t currentPulse,
                    uint32_t duration = SMOOTH_MOVE_DURATION_MS) {
  ServoMotion& m = servoMotions[servoIndex];
  m.startPulse  = currentPulse;
  m.targetPulse = targetPulse;
  m.startTime   = millis();
  m.duration    = duration;
  m.active      = true;
}

// Returns true while any servo is still moving.
bool updateServos() {
  const uint32_t now = millis();
  bool anyActive = false;

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    ServoMotion& m = servoMotions[i];
    if (!m.active) continue;

    const uint32_t elapsed = now - m.startTime;

    if (elapsed >= m.duration) {
      pwm.setPWM(SERVO_CHANNELS[i], 0, m.targetPulse);
      m.active = false;
      Serial.println(" | DONE");
      continue;
    }

    const float t     = (float)elapsed / (float)m.duration;
    const float eased = (1.0f - cosf(t * M_PI)) * 0.5f;
    const float raw = m.startPulse + eased * ((int32_t)m.targetPulse - (int32_t)m.startPulse);
    const uint16_t pulse = (uint16_t)constrain(raw,
                              min(m.startPulse, m.targetPulse),
                              max(m.startPulse, m.targetPulse));

    pwm.setPWM(SERVO_CHANNELS[i], 0, pulse);
    anyActive = true;
  }

  return anyActive;
}

// Blocking helper — starts a move and waits for it to finish.
// Keeps the same call signature your loop() already uses.
void moveServoSmooth(uint8_t servoIndex, uint16_t targetPulse, uint16_t currentPulse) {
  startServoMove(servoIndex, targetPulse, currentPulse);
  while (updateServos()) {
    delay(SMOOTH_UPDATE_INTERVAL_MS);
  }
}

// --- Arduino entry points -------------------------------------------------
void setup() {
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  // Snap all servos to their start position immediately on boot
  // for (uint8_t i = 0; i < SERVO_COUNT; i++) 
  // {
  //   pwm.setPWM(SERVO_CHANNELS[i], 0, SERVO_MIN_PULSE[i]);
  // }
  pwm.setPWM(12, 0, SERVO_MIN_PULSE[0]);
  pwm.setPWM(13, 0, STANDARD_CENTER_PULSE[1]);
  pwm.setPWM(14, 0, STANDARD_CENTER_PULSE[2]);
  pwm.setPWM(8, 0, STANDARD_CENTER_PULSE[4]);
  delay(500); // give servos time to reach start position
}

void loop() {
  const uint8_t k = 0;
  moveServoSmooth(k, SERVO_MAX_PULSE[k], SERVO_MIN_PULSE[k]);
  delay(1000);
  moveServoSmooth(k, SERVO_MIN_PULSE[k], SERVO_MAX_PULSE[k]);
  delay(1000);
}