#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

constexpr uint8_t FIRST_CHANNEL = 0;
constexpr uint8_t LAST_CHANNEL = 15;

void setup() {
  Serial.begin(115200);
  Serial.println("PCA9685 all outputs off.");

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  for (uint8_t channel = FIRST_CHANNEL; channel <= LAST_CHANNEL; ++channel) {
    pwm.setPWM(channel, 0, 0);
  }
}

void loop() {
  delay(1000);
}
