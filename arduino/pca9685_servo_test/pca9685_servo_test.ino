#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 default I2C address is 0x40.
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo is connected to PCA9685 channel 12.
constexpr uint8_t SERVO_CHANNEL = 12;

// Tune these values if your servo does not reach the expected angles cleanly.
// 150..600 is a conservative range for many hobby servos at 50 Hz.
constexpr uint16_t SERVO_MIN_PULSE = 150;
constexpr uint16_t SERVO_MAX_PULSE = 600;
constexpr uint16_t SERVO_CENTER_PULSE = (SERVO_MIN_PULSE + SERVO_MAX_PULSE) / 2;

constexpr uint16_t STEP_DELAY_MS = 15;
constexpr uint16_t HOLD_DELAY_MS = 800;

void moveServoSmooth(uint16_t fromPulse, uint16_t toPulse) {
  if (fromPulse < toPulse) {
    for (uint16_t pulse = fromPulse; pulse <= toPulse; ++pulse) {
      pwm.setPWM(SERVO_CHANNEL, 0, pulse);
      delay(STEP_DELAY_MS);
    }
  } else {
    for (int pulse = fromPulse; pulse >= static_cast<int>(toPulse); --pulse) {
      pwm.setPWM(SERVO_CHANNEL, 0, pulse);
      delay(STEP_DELAY_MS);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("PCA9685 servo test starting...");

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);  // Standard analog servo frequency.

  delay(10);

  Serial.println("Moving servo to center position.");
  pwm.setPWM(SERVO_CHANNEL, 0, SERVO_CENTER_PULSE);
  delay(1000);
}

void loop() {
  Serial.println("Sweeping to minimum.");
  moveServoSmooth(SERVO_CENTER_PULSE, SERVO_MIN_PULSE);
  delay(HOLD_DELAY_MS);

  Serial.println("Sweeping to maximum.");
  moveServoSmooth(SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  delay(HOLD_DELAY_MS);

  Serial.println("Returning to center.");
  moveServoSmooth(SERVO_MAX_PULSE, SERVO_CENTER_PULSE);
  delay(1500);
}
