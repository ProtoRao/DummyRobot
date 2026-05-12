#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

constexpr uint8_t SERVO_CHANNELS[] = {12, 13, 14, 10, 8};
constexpr int16_t INITIAL_ANGLES[] = {90, 90, 0, 0, 0};
constexpr int16_t LOGICAL_MIN_ANGLES[] = {0, 0, -90, -90, -90};
constexpr int16_t LOGICAL_MAX_ANGLES[] = {180, 180, 90, 90, 90};
constexpr int16_t ANGLE_OFFSETS[] = {0, -5, 0, 0, 0};
constexpr int16_t STANDARD_PULSE_TRIMS[] = {-75, -12, 0, 0, 0};
constexpr uint16_t CENTER_PULSES[] = {0, 0, 500, 300, 350};
constexpr bool CENTERED_POSITIVE_INCREASES_PWM[] = {false, false, false, false, true};
constexpr uint8_t SERVO_COUNT = sizeof(SERVO_CHANNELS) / sizeof(SERVO_CHANNELS[0]);

// Tune this range if your servos need slightly different endpoints.
constexpr uint16_t SERVO_MIN_PULSE = 150;
constexpr uint16_t SERVO_MAX_PULSE = 600;
constexpr int16_t CENTERED_RANGE_DEGREES = 90;
constexpr int16_t HALF_PULSE_RANGE = (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 2;
constexpr uint8_t SMOOTH_MIN_STEP_PULSE = 1;
constexpr uint8_t SMOOTH_MAX_STEP_PULSE = 4;
constexpr uint8_t SMOOTH_STEP_DELAY_MS = 8;

int16_t currentAngles[SERVO_COUNT];
uint16_t currentPulses[SERVO_COUNT];

int16_t clampLogicalAngle(uint8_t servoIndex, int16_t angle) {
  return constrain(angle, LOGICAL_MIN_ANGLES[servoIndex], LOGICAL_MAX_ANGLES[servoIndex]);
}

int16_t logicalAngleToPhysicalAngle(uint8_t servoIndex, int16_t angle) {
  return angle;
}

uint16_t physicalAngleToPulse(int16_t angle) {
  return map(constrain(angle, 0, 180), 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

int16_t applyServoCalibration(uint8_t servoIndex, int16_t angle) {
  const int correctedAngle = static_cast<int>(angle) + ANGLE_OFFSETS[servoIndex];
  return constrain(correctedAngle, 0, 180);
}

uint16_t centeredAngleToPulse(uint8_t servoIndex, int16_t angle) {
  int centeredPulse = 0;
  if (CENTERED_POSITIVE_INCREASES_PWM[servoIndex]) {
    centeredPulse = map(
        angle,
        -CENTERED_RANGE_DEGREES,
        CENTERED_RANGE_DEGREES,
        CENTER_PULSES[servoIndex] - HALF_PULSE_RANGE,
        CENTER_PULSES[servoIndex] + HALF_PULSE_RANGE);
  } else {
    centeredPulse = map(
        angle,
        -CENTERED_RANGE_DEGREES,
        CENTERED_RANGE_DEGREES,
        CENTER_PULSES[servoIndex] + HALF_PULSE_RANGE,
        CENTER_PULSES[servoIndex] - HALF_PULSE_RANGE);
  }
  return static_cast<uint16_t>(constrain(centeredPulse, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
}

void moveServoSmooth(uint8_t servoIndex, uint16_t targetPulse) {
  const uint8_t channel = SERVO_CHANNELS[servoIndex];
  uint16_t currentPulse = currentPulses[servoIndex];

  // On first command after boot, we do not know the real physical starting pulse.
  // Set directly once, then smooth all later moves from the cached position.
  if (currentPulse == 0) {
    pwm.setPWM(channel, 0, targetPulse);
    currentPulses[servoIndex] = targetPulse;
    return;
  }

  while (currentPulse != targetPulse) {
    const uint16_t remainingDistance =
        (currentPulse < targetPulse) ? (targetPulse - currentPulse) : (currentPulse - targetPulse);
    uint16_t stepPulse = remainingDistance / 6;
    stepPulse = constrain(stepPulse, SMOOTH_MIN_STEP_PULSE, SMOOTH_MAX_STEP_PULSE);

    uint16_t nextPulse = currentPulse;
    if (currentPulse < targetPulse) {
      nextPulse = static_cast<uint16_t>(currentPulse + stepPulse);
      if (nextPulse > targetPulse) {
        nextPulse = targetPulse;
      }
    } else {
      nextPulse = static_cast<uint16_t>(currentPulse - stepPulse);
      if (nextPulse < targetPulse) {
        nextPulse = targetPulse;
      }
    }

    pwm.setPWM(channel, 0, nextPulse);
    currentPulse = nextPulse;
    delay(SMOOTH_STEP_DELAY_MS);
  }

  currentPulses[servoIndex] = targetPulse;
}

void setServoByIndex(uint8_t servoIndex, int16_t angle) {
  angle = clampLogicalAngle(servoIndex, angle);
  uint16_t finalPulse = 0;
  if (LOGICAL_MIN_ANGLES[servoIndex] == -90) {
    finalPulse = centeredAngleToPulse(servoIndex, angle);
  } else {
    const int16_t physicalAngle = logicalAngleToPhysicalAngle(servoIndex, angle);
    const int16_t correctedAngle = applyServoCalibration(servoIndex, physicalAngle);
    const int adjustedPulse = static_cast<int>(physicalAngleToPulse(correctedAngle)) + STANDARD_PULSE_TRIMS[servoIndex];
    finalPulse = static_cast<uint16_t>(constrain(adjustedPulse, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
  }
  moveServoSmooth(servoIndex, finalPulse);
  currentAngles[servoIndex] = angle;
}

void applyInitialPositions() {
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    setServoByIndex(i, INITIAL_ANGLES[i]);
    delay(80);
  }
}

void printState(uint8_t servoIndex) {
  Serial.print("STATE ");
  Serial.print(servoIndex);
  Serial.print(' ');
  Serial.print(SERVO_CHANNELS[servoIndex]);
  Serial.print(' ');
  Serial.print(currentAngles[servoIndex]);
  Serial.print(' ');
  Serial.println(currentPulses[servoIndex]);
}

void printAllStates() {
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    printState(i);
  }
  Serial.println("DONE");
}

void handleSetCommand(char *args) {
  int servoIndex = -1;
  int angle = -1;
  if (sscanf(args, "%d %d", &servoIndex, &angle) != 2) {
    Serial.println("ERR bad_set");
    return;
  }

  if (servoIndex < 0 || servoIndex >= SERVO_COUNT) {
    Serial.println("ERR range");
    return;
  }

  if (angle < LOGICAL_MIN_ANGLES[servoIndex] || angle > LOGICAL_MAX_ANGLES[servoIndex]) {
    Serial.println("ERR range");
    return;
  }

  setServoByIndex(static_cast<uint8_t>(servoIndex), angle);
  Serial.print("OK ");
  Serial.print(servoIndex);
  Serial.print(' ');
  Serial.print(SERVO_CHANNELS[servoIndex]);
  Serial.print(' ');
  Serial.print(currentAngles[servoIndex]);
  Serial.print(' ');
  Serial.println(currentPulses[servoIndex]);
}

void handleGetCommand(char *args) {
  const int servoIndex = atoi(args);
  if (servoIndex < 0 || servoIndex >= SERVO_COUNT) {
    Serial.println("ERR range");
    return;
  }
  printState(static_cast<uint8_t>(servoIndex));
}

void handleCommand(char *buffer) {
  if (strcmp(buffer, "GETALL") == 0) {
    printAllStates();
    return;
  }

  if (strcmp(buffer, "HOME") == 0) {
    applyInitialPositions();
    Serial.println("OK HOME");
    return;
  }

  if (strncmp(buffer, "GET ", 4) == 0) {
    handleGetCommand(buffer + 4);
    return;
  }

  if (strncmp(buffer, "SET ", 4) == 0) {
    handleSetCommand(buffer + 4);
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

  applyInitialPositions();
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
