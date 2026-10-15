# TeleOp VT: Controller Instructions

OpMode: **TeleOp VT** (group: Competition)

## Gamepad 1 (Driver): Drive and Shoot

| Input | Action |
|---|---|
| Left stick / right stick | Robot-centric mecanum drive |
| Left bumper (hold) | Slow mode: drive speed is 40% while held |
| A | Shooter forward |
| B | Shooter off |
| X | Shooter reverse |
| D-pad Up | Shooter power multiplier +0.01 |
| D-pad Down | Shooter power multiplier -0.01 |

### Shooter notes
- The shooter has two motors that run in opposite directions. Both receive the same power command.
- Power output = direction power (+1 forward, -1 reverse) x power multiplier.
- The power multiplier ranges from 0.10 to 1.00 and changes in 0.01 steps.
- The gate servo opens only in the forward state. It stays closed in off and reverse.

## Gamepad 2 (Operator): Intake and Slider

| Input | Action |
|---|---|
| A | Intake forward |
| B | Intake off |
| X | Intake reverse |

### Slider
The slider is not installed yet, so its controls are commented out in the code.
When the hardware is ready, uncomment the following lines in `TeleOp_VT.java`:
- Y: extend (`robot.slider.extend()`)
- Right bumper: return to start (`robot.slider.returnToStart()`)

## Driver Station Telemetry

- Shooter State: FORWARD, REVERSE, or OFF
- Shooter RPM 1 / RPM 2: measured speed of each shooter motor (absolute value)
- Shooter Power 1 / Power 2: power sent to each shooter motor
- Power Multiplier: current shooter power limit
- Gate Position: gate servo position (0.0 closed, 1.0 open)
- Intake State and Intake Power

## Quick Start

1. Select **TeleOp VT** on the Driver Station and press INIT.
2. Press START.
3. Use the left bumper for precise driving near the field elements.
4. Set the shooter power with D-pad Up/Down before shooting.
5. Press A on Gamepad 1 to spin the shooter up, then load game pieces with the intake on Gamepad 2.
6. Press B on Gamepad 1 to stop the shooter when finished.
