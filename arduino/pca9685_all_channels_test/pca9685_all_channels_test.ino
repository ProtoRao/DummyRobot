#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

constexpr uint8_t FIRST_CHANNEL = 0;
constexpr uint8_t LAST_CHANNEL = 15;
constexpr uint16_t SERVO_MIN_PULSE = 150;
constexpr uint16_t SERVO_MAX_PULSE = 600;
constexpr uint16_t SERVO_CENTER_PULSE = (SERVO_MIN_PULSE + SERVO_MAX_PULSE) / 2;
constexpr uint16_t STEP_DELAY_MS = 10;
constexpr uint16_t HOLD_DELAY_MS = 700;

void moveServoSmooth(uint8_t channel, uint16_t fromPulse, uint16_t toPulse) {
  if (fromPulse < toPulse) {
    for (uint16_t pulse = fromPulse; pulse <= toPulse; ++pulse) {
      pwm.setPWM(channel, 0, pulse);
      delay(STEP_DELAY_MS);
    }
  } else {
    for (int pulse = fromPulse; pulse >= static_cast<int>(toPulse); --pulse) {
      pwm.setPWM(channel, 0, pulse);
      delay(STEP_DELAY_MS);
    }
  }
}

void testChannel(uint8_t channel) {
  Serial.print("Testing PCA9685 channel ");
  Serial.println(channel);

  pwm.setPWM(channel, 0, SERVO_CENTER_PULSE);
  delay(1000);

  Serial.println("  Moving to 0 degrees.");
  moveServoSmooth(channel, SERVO_CENTER_PULSE, SERVO_MIN_PULSE);
  delay(HOLD_DELAY_MS);

  Serial.println("  Moving to 180 degrees.");
  moveServoSmooth(channel, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  delay(HOLD_DELAY_MS);

  Serial.println("  Returning to center.");
  moveServoSmooth(channel, SERVO_MAX_PULSE, SERVO_CENTER_PULSE);
  delay(1000);

  pwm.setPWM(channel, 0, 0);
}

void setup() {
  Serial.begin(115200);
  Serial.println("PCA9685 all-channel servo test starting...");

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);

  delay(10);
}

void loop() {
  for (uint8_t channel = FIRST_CHANNEL; channel <= LAST_CHANNEL; ++channel) {
    testChannel(channel);
  }

  Serial.println("All channels tested. Restarting in 3 seconds.");
  delay(3000);
}
