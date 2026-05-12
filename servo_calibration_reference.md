# Servo Calibration Reference

Current calibrated setup for the 5-servo PCA9685 controller.

## Servo Order

1. Servo 1 -> PCA9685 channel `12`
2. Servo 2 -> PCA9685 channel `13`
3. Servo 3 -> PCA9685 channel `14`
4. Servo 4 -> PCA9685 channel `10`
5. Servo 5 -> PCA9685 channel `8`

## Home Positions

Logical startup and `HOME` angles:

`90, 90, 0, 0, 0`

## Angle Modes

- Servo 1: `0..180`
- Servo 2: `0..180`
- Servo 3: `-90..+90`
- Servo 4: `-90..+90`
- Servo 5: `-90..+90`

## Calibration Notes

- Servo 1 / channel `12`
  Logical range: `0..180`
  Offset: `0`
  Pulse trim: `-75`
  Direction: standard `0 -> 180` mapping

- Servo 2 / channel `13`
  Logical range: `0..180`
  Offset: `-5`
  Note: logical `5 deg` is approximately physical `0 deg`
  Pulse trim: `-12`
  Reference: logical `90 deg` corresponds to pulse `350`
  Direction: standard `0 -> 180` mapping

- Servo 3 / channel `14`
  Logical range: `-90..+90`
  Center pulse at logical `0 deg`: `500`
  Direction: positive angle decreases PWM pulse width
  Negative angle increases PWM pulse width

- Servo 4 / channel `10`
  Logical range: `-90..+90`
  Center pulse at logical `0 deg`: `300`
  Direction: positive angle decreases PWM pulse width
  Negative angle increases PWM pulse width

- Servo 5 / channel `8`
  Logical range: `-90..+90`
  Center pulse at logical `0 deg`: `350`
  Direction: positive angle increases PWM pulse width
  Negative angle decreases PWM pulse width

## Firmware Constants

These values are currently reflected in:

- [pca9685_positional_serial_bridge.ino](D:/Robotics/DummyRobot/arduino/pca9685_positional_serial_bridge/pca9685_positional_serial_bridge.ino)
- [servo_position_gui.py](D:/Robotics/DummyRobot/servo_position_gui.py)

Relevant firmware settings:

```cpp
SERVO_CHANNELS = {12, 13, 14, 10, 8}
INITIAL_ANGLES = {90, 90, 0, 0, 0}
ANGLE_OFFSETS = {0, -5, 0, 0, 0}
STANDARD_PULSE_TRIMS = {-75, -12, 0, 0, 0}
CENTER_PULSES = {0, 0, 500, 300, 350}
CENTERED_POSITIVE_INCREASES_PWM = {false, false, false, false, true}
SMOOTH_MIN_STEP_PULSE = 1
SMOOTH_MAX_STEP_PULSE = 4
SMOOTH_STEP_DELAY_MS = 8
```

## Motion Smoothing

Servo moves are ramped in firmware instead of jumping straight to the target PWM.

- Pulse step size: `4`
- Step size range: `1..4`
- Delay between steps: `8 ms`

If motion is still too abrupt:

- Reduce `SMOOTH_MAX_STEP_PULSE`
- Increase `SMOOTH_STEP_DELAY_MS`

If motion becomes too slow:

- Increase `SMOOTH_MAX_STEP_PULSE`
- Reduce `SMOOTH_STEP_DELAY_MS`
