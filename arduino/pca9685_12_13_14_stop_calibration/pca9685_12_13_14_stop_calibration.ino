#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

constexpr uint8_t CHANNELS[] = {12, 13, 14};
constexpr uint16_t TEST_PULSES[] = {295, 300, 305, 307, 310, 315, 320};
constexpr uint16_t SETTLE_DELAY_MS = 2500;

void setAllTestChannels(uint16_t pulse) {
  for (uint8_t i = 0; i < sizeof(CHANNELS); ++i) {
    pwm.setPWM(CHANNELS[i], 0, pulse);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("PCA9685 stop calibration for channels 12, 13, 14.");

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  for (uint8_t i = 0; i < sizeof(TEST_PULSES) / sizeof(TEST_PULSES[0]); ++i) {
    const uint16_t pulse = TEST_PULSES[i];
    Serial.print("Testing stop pulse: ");
    Serial.println(pulse);
    setAllTestChannels(pulse);
    delay(SETTLE_DELAY_MS);
  }

  Serial.println("Calibration sweep complete. Holding last pulse.");
}

void loop() {
}
