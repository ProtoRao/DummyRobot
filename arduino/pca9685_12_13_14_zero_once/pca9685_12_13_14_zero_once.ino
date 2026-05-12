#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

constexpr uint8_t CHANNELS[] = {12, 13, 14};
constexpr uint16_t SERVO_STOP_PULSE = 307;
constexpr uint16_t STEP_DELAY_MS = 400;

void setup() {
  Serial.begin(115200);
  Serial.println("Setting channels 12, 13, 14 to stop pulse once.");

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  for (uint8_t i = 0; i < sizeof(CHANNELS); ++i) {
    const uint8_t channel = CHANNELS[i];
    Serial.print("Setting channel ");
    Serial.println(channel);
    pwm.setPWM(channel, 0, SERVO_STOP_PULSE);
    delay(STEP_DELAY_MS);
  }
}

void loop() {
}
