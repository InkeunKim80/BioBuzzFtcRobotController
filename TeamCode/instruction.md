# TeleOp VT: Controller Instructions

OpMode: **TeleOp VT** (group: Competition)

## Gamepad 1 (Driver): Drive

| Input | Action |
|---|---|
| Left stick / right stick | Robot-centric mecanum drive |
| Left bumper (hold) | Slow mode: drive speed is 40% while held |

## Gamepad 2 (Operator): Intake, Shooter, and Gate

| Input | Action |
|---|---|
| A | Intake forward and shooter forward (together) |
| B | Intake off and shooter off (together) |
| X | Intake reverse and shooter reverse (together) |
| D-pad Up | Shooter power multiplier +0.01 |
| D-pad Down | Shooter power multiplier -0.01 |
| Y | Gate toggle: press once to open, press again to close |

### Shooter notes
- The shooter has two motors that run in opposite directions. Both receive the same power command.
- Power output = direction power (+1 forward, -1 reverse) x power multiplier.
- The power multiplier ranges from 0.10 to 1.00 and changes in 0.01 steps.
- The gate is not tied to the shooter state. Use Y to open or close it manually.

### Slider
The slider is not installed yet, so its controls are commented out in the code.
When the hardware is ready, uncomment the following lines in `TeleOp_VT.java`:
- Right bumper: return to start (`robot.slider.returnToStart()`)

Y is already used by the gate toggle. Assign a different button to the slider extend action before enabling it.

## Driver Station Telemetry

- Shooter State: FORWARD, REVERSE, or OFF
- Shooter RPM 1 / RPM 2: measured speed of each shooter motor (absolute value)
- Shooter Power 1 / Power 2: power sent to each shooter motor
- Power Multiplier: current shooter power limit
- Intake State and Intake Power

## Quick Start

1. Select **TeleOp VT** on the Driver Station and press INIT.
2. Press START.
3. Drive with Gamepad 1. Use the left bumper for precise driving near the field elements.
4. Set the shooter power with D-pad Up/Down on Gamepad 2 before shooting.
5. Press Y on Gamepad 2 to open the gate.
6. Press A on Gamepad 2 to start the shooter and intake together.
7. Press B on Gamepad 2 to stop both when finished, and press Y again to close the gate.
