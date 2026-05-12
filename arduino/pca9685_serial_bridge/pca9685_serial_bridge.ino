#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

constexpr uint8_t kChannelCount = 16;
constexpr uint16_t kPulseMin = 205;   // ~1.0 ms at 50 Hz
constexpr uint16_t kPulseStop = 307;  // ~1.5 ms at 50 Hz
constexpr uint16_t kPulseMax = 410;   // ~2.0 ms at 50 Hz
constexpr uint8_t kAngleMin = 0;
constexpr uint8_t kAngleStop = 90;
constexpr uint8_t kAngleMax = 180;
constexpr uint8_t kPca9685Address = 0x40;

uint16_t channelPulseCache[kChannelCount];

uint16_t angleToPulse(uint8_t angle) {
  if (angle <= kAngleStop) {
    return map(angle, kAngleMin, kAngleStop, kPulseMin, kPulseStop);
  }
  return map(angle, kAngleStop, kAngleMax, kPulseStop, kPulseMax);
}

uint8_t pulseToAngle(uint16_t pulse) {
  pulse = constrain(pulse, kPulseMin, kPulseMax);
  if (pulse <= kPulseStop) {
    return map(pulse, kPulseMin, kPulseStop, kAngleMin, kAngleStop);
  }
  return map(pulse, kPulseStop, kPulseMax, kAngleStop, kAngleMax);
}

uint16_t readRegister16(uint8_t reg) {
  Wire.beginTransmission(kPca9685Address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(static_cast<int>(kPca9685Address), 2);

  if (Wire.available() < 2) {
    return 0;
  }

  const uint8_t low = Wire.read();
  const uint8_t high = Wire.read();
  return static_cast<uint16_t>(low | (high << 8));
}

uint16_t readChannelOffCount(uint8_t channel) {
  const uint8_t baseReg = 0x06 + 4 * channel;
  const uint16_t offCount = readRegister16(baseReg + 2);
  return offCount & 0x0FFF;
}

void refreshPulseCache() {
  for (uint8_t channel = 0; channel < kChannelCount; ++channel) {
    uint16_t pulse = readChannelOffCount(channel);
    if (pulse == 0) {
      pulse = kPulseStop;
    }
    channelPulseCache[channel] = pulse;
  }
}

void setChannelAngle(uint8_t channel, uint8_t angle) {
  const uint16_t pulse = angleToPulse(angle);
  pwm.setPWM(channel, 0, pulse);
  channelPulseCache[channel] = pulse;
}

void printChannelState(uint8_t channel) {
  const uint16_t pulse = channelPulseCache[channel];
  Serial.print("STATE ");
  Serial.print(channel);
  Serial.print(' ');
  Serial.print(pulseToAngle(pulse));
  Serial.print(' ');
  Serial.println(pulse);
}

void printAllStates() {
  for (uint8_t channel = 0; channel < kChannelCount; ++channel) {
    printChannelState(channel);
  }
  Serial.println("DONE");
}

void handleSetCommand(char *args) {
  int channel = -1;
  int angle = -1;
  if (sscanf(args, "%d %d", &channel, &angle) != 2) {
    Serial.println("ERR bad_set");
    return;
  }

  if (channel < 0 || channel >= kChannelCount || angle < kAngleMin || angle > kAngleMax) {
    Serial.println("ERR range");
    return;
  }

  setChannelAngle(static_cast<uint8_t>(channel), static_cast<uint8_t>(angle));
  Serial.print("OK ");
  Serial.print(channel);
  Serial.print(' ');
  Serial.print(angle);
  Serial.print(' ');
  Serial.println(channelPulseCache[channel]);
}

void handlePulseCommand(char *args) {
  int channel = -1;
  int pulse = -1;
  if (sscanf(args, "%d %d", &channel, &pulse) != 2) {
    Serial.println("ERR bad_pulse");
    return;
  }

  if (channel < 0 || channel >= kChannelCount || pulse < 0 || pulse > 4095) {
    Serial.println("ERR range");
    return;
  }

  pwm.setPWM(static_cast<uint8_t>(channel), 0, static_cast<uint16_t>(pulse));
  channelPulseCache[channel] = static_cast<uint16_t>(pulse);
  Serial.print("OK ");
  Serial.print(channel);
  Serial.print(' ');
  Serial.print(pulseToAngle(channelPulseCache[channel]));
  Serial.print(' ');
  Serial.println(channelPulseCache[channel]);
}

void handleCommand(char *buffer) {
  if (strncmp(buffer, "GETALL", 6) == 0) {
    refreshPulseCache();
    printAllStates();
    return;
  }

  if (strncmp(buffer, "GET ", 4) == 0) {
    const int channel = atoi(buffer + 4);
    if (channel < 0 || channel >= kChannelCount) {
      Serial.println("ERR range");
      return;
    }
    refreshPulseCache();
    printChannelState(static_cast<uint8_t>(channel));
    return;
  }

  if (strncmp(buffer, "SET ", 4) == 0) {
    handleSetCommand(buffer + 4);
    return;
  }

  if (strncmp(buffer, "PULSE ", 6) == 0) {
    handlePulseCommand(buffer + 6);
    return;
  }

  if (strncmp(buffer, "STOP ", 5) == 0) {
    const int channel = atoi(buffer + 5);
    if (channel < 0 || channel >= kChannelCount) {
      Serial.println("ERR range");
      return;
    }
    setChannelAngle(static_cast<uint8_t>(channel), kAngleStop);
    Serial.print("OK ");
    Serial.print(channel);
    Serial.print(' ');
    Serial.print(kAngleStop);
    Serial.print(' ');
    Serial.println(channelPulseCache[channel]);
    return;
  }

  Serial.println("ERR unknown");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  refreshPulseCache();
  Serial.println("READY");
}

void loop() {
  static char buffer[32];
  static uint8_t index = 0;

  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r') {
      continue;
    }

    if (ch == '\n') {
      buffer[index] = '\0';
      if (index > 0) {
        handleCommand(buffer);
      }
      index = 0;
      continue;
    }

    if (index < sizeof(buffer) - 1) {
      buffer[index++] = ch;
    }
  }
}
