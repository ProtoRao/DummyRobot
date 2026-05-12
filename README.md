# DummyRobot

Control software and Arduino sketches for a 5-servo PCA9685-based robot arm.

## Repository Layout

- `arduino/` contains the firmware sketches and upload helper scripts.
- `servo_position_gui.py` provides a per-servo position control GUI over the serial bridge.
- `servo_control_gui.py` provides stepped angle controls for PCA9685 channels.
- `ik_position_gui.py` adds XYZ inverse-kinematics control on top of the serial bridge.
- `test_ik_position_gui.py` contains regression tests for the IK math helpers.
- `servo_calibration_reference.md` documents the active servo/channel calibration values.

## Main Firmware

The primary firmware entry point is:

- `arduino/pca9685_positional_serial_bridge/pca9685_positional_serial_bridge.ino`

It exposes serial commands for homing, reading servo state, and setting logical servo angles.

## Notes

- Generated Arduino CLI folders and Python bytecode caches are ignored.
- The older `robot_arm_3dof.py` and `robot_arm_xyz_driver.py` files were replaced by the current GUI-based workflow.
