#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

constexpr uint8_t SERVO_CHANNELS[] = {12, 13, 14, 10, 8};
constexpr int16_t INITIAL_ANGLES[] = {90, 90, -90, 0, 0};
constexpr int16_t LOGICAL_MIN_ANGLES[] = {0, 0, -150, -90, -90};
constexpr int16_t LOGICAL_MAX_ANGLES[] = {180, 180, 30, 90, 90};
constexpr int8_t  CENTERED_POSITIVE_INCREASES_PWM[] = {0, 0, 1, 0, 2};
constexpr uint8_t SERVO_COUNT = sizeof(SERVO_CHANNELS) / sizeof(SERVO_CHANNELS[0]);

constexpr uint16_t SERVO_MIN_PULSE[] = {85, 100, 500, 150, 140};
constexpr uint16_t SERVO_MAX_PULSE[] = {475, 480, 140, 640, 540};
constexpr uint16_t STANDARD_CENTER_PULSE[] = {280, 280, 380, 395, 34};
constexpr int16_t CENTERED_RANGE_DEGREES = 90;
constexpr int16_t HALF_PULSE_RANGE[] = {235, 235, 235, 245, 245};
constexpr uint8_t SMOOTH_MIN_STEP_PULSE = 1;
constexpr uint8_t SMOOTH_MAX_STEP_PULSE = 2;
constexpr uint8_t SMOOTH_STEP_DELAY_MS = 32;

int16_t currentAngles[SERVO_COUNT];
uint16_t currentPulses[SERVO_COUNT];
uint16_t targetPulses[SERVO_COUNT]; 
unsigned long lastStepTime = 0;

uint16_t physicalAngleToPulse(uint8_t servoIndex, int16_t angle) {
  return map(angle, LOGICAL_MIN_ANGLES[servoIndex], LOGICAL_MAX_ANGLES[servoIndex], SERVO_MIN_PULSE[servoIndex], SERVO_MAX_PULSE[servoIndex]);
}

void updateServosNonBlocking() {
  // Only step if the smoothing delay has passed 
  if (millis() - lastStepTime < SMOOTH_STEP_DELAY_MS) return;
  lastStepTime = millis();

  bool movementOccurred = false;

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    if (currentPulses[i] != targetPulses[i]) {
      const uint16_t remainingDistance = (currentPulses[i] < targetPulses[i]) ? 
                                         (targetPulses[i] - currentPulses[i]) : 
                                         (currentPulses[i] - targetPulses[i]);
      
      // Keep your original smoothing logic [cite: 12]
      uint16_t stepPulse = remainingDistance / 6;
      stepPulse = constrain(stepPulse, SMOOTH_MIN_STEP_PULSE, SMOOTH_MAX_STEP_PULSE);

      if (currentPulses[i] < targetPulses[i]) {
        currentPulses[i] += stepPulse;
        if (currentPulses[i] > targetPulses[i]) currentPulses[i] = targetPulses[i];
      } else {
        currentPulses[i] -= stepPulse;
        if (currentPulses[i] < targetPulses[i]) currentPulses[i] = targetPulses[i];
      }

      pwm.setPWM(SERVO_CHANNELS[i], 0, currentPulses[i]);
      movementOccurred = true;
    }
  }
}

void setServoByIndex(uint8_t servoIndex, int16_t angle) {
  angle = constrain(angle, LOGICAL_MIN_ANGLES[servoIndex], LOGICAL_MAX_ANGLES[servoIndex]);
  targetPulses[servoIndex] = physicalAngleToPulse(servoIndex, angle);
  currentAngles[servoIndex] = angle; // Track logical angle
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

  for(int i=0; i<SERVO_COUNT; i++) {
    targetPulses[i] = physicalAngleToPulse(i, INITIAL_ANGLES[i]);
  }

}

void loop() {
  updateServosNonBlocking(); // Always checking if servos need to step
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
