#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

constexpr uint8_t SERVO_CHANNELS[] = {
  12, 13, 14
};

// Tune these if your servos need a different 0..180 degree pulse range.
constexpr uint16_t SERVO_MIN_PULSE = 150;  // 0 degrees
constexpr uint16_t SERVO_MAX_PULSE = 600;  // 180 degrees

constexpr uint16_t STEP_DELAY_MS = 15;
constexpr uint16_t HOLD_DELAY_MS = 1000;

uint16_t angleToPulse(uint8_t angle) {
  return map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

void writeAllServos(uint8_t angle) {
  const uint16_t pulse = angleToPulse(angle);
  for (uint8_t i = 0; i < sizeof(SERVO_CHANNELS); ++i) {
    pwm.setPWM(SERVO_CHANNELS[i], 0, pulse);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing all servos to 0 degrees...");

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  writeAllServos(0);
  delay(1500);

  for (uint8_t angle = 0; angle <= 180; ++angle) {
    writeAllServos(angle);
    delay(STEP_DELAY_MS);
  }
  delay(HOLD_DELAY_MS);

  for (int angle = 180; angle >= 0; --angle) {
    writeAllServos(static_cast<uint8_t>(angle));
    delay(STEP_DELAY_MS);
  }
  writeAllServos(0);
  Serial.println("Sweep complete. Holding at 0 degrees.");
}

void loop() {
  writeAllServos(0);
  delay(1000);
}
