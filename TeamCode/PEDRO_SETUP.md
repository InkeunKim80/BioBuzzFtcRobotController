# Pedro Pathing Setup

This project uses Pedro Pathing for drivetrain localization, path generation,
path following, and autonomous movement.

The robot software architecture also uses NextFTC mechanisms and the Ivy
command framework.

## Current dependencies

The project currently includes Pedro Pathing components including:

```text
com.pedropathing:revhub
com.pedropathing:tuning
com.pedropathing.ivy:pedro
```

and NextFTC components including:

```text
dev.nextftc.v2:control
dev.nextftc.v2:hardware
dev.nextftc.v2:robot
```

Check `build.dependencies.gradle` and `TeamCode/build.gradle` for the exact
versions currently used by the project.

FTC SDK 12 is used as the base SDK for the 2026-2027 season.

## Project structure

Pedro-related configuration and tuning code is located under:

```text
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/
```

The main files are:

```text
pedro/
├── Constants.java
├── Tuning.java
└── procedures/
```

Robot mechanisms are organized separately using NextFTC:

```text
teamcode/
├── hardware/
├── mechanisms/
├── opmodes/
├── pedro/
├── robot/
└── util/
```

This separation allows Pedro to handle localization and path following while
the robot mechanisms remain modular.

## Configure the drivetrain and localizer

Before using autonomous path following, configure the robot-specific Pedro
constants in:

```text
pedro/Constants.java
```

The configuration must match the actual robot hardware, including:

- drivetrain geometry
- motor directions
- encoder directions
- localizer type
- odometry hardware
- drivetrain constants
- follower constants

Do not assume the template values are correct for the final robot.

## Pedro tuning

Pedro tuning procedures are registered through:

```text
pedro/Tuning.java
```

The current `Tuning.java` should be reviewed before robot tuning begins.
Only tuner procedures compatible with the Pedro dependencies currently used
by this project should be enabled.

The general tuning sequence is:

```text
Drivetrain configuration
        ↓
Localization configuration
        ↓
Localization verification
        ↓
Follower tuning
        ↓
Foresight tuning
        ↓
Autonomous path validation
```

See:

```text
TeamCode/TUNING.md
```

for the project-specific tuning procedure.

## TeleOp

The BioBuzz project uses the NextFTC drivetrain mechanism rather than the
older `BasicTeleOp` implementation.

Relevant code is under:

```text
teamcode/mechanisms/Drivetrain.java
teamcode/opmodes/teleop/
```

Both robot-centric and field-centric drive implementations may be used
depending on the selected TeleOp.

Motor names and directions must match the Control Hub robot configuration.

## Autonomous

Autonomous OpModes are located under:

```text
teamcode/opmodes/auto/
```

Pedro paths should be built using the tuned robot configuration and then
executed through the project's Pedro/NextFTC command architecture.

The Pedro Pathing Visualizer can be used to design and inspect planned paths
before deploying them to the robot.

## Localization logging

Actual robot pose can be recorded using:

```text
teamcode/util/PoseLogger.java
```

Logs are stored on the Control Hub under:

```text
/sdcard/FIRST/localization_logs/
```

and can be copied into:

```text
logs/
```

for offline analysis.

The notebook:

```text
analysis/notebooks/localization_validation.ipynb
```

can be used to visualize the measured robot trajectory.

See:

```text
TeamCode/POSE_LOGGING.md
```

for log retrieval instructions.

The long-term validation workflow is:

```text
Pedro planned path
        ↓
Expected robot trajectory
        ↓
Robot execution
        ↓
PoseLogger measurement
        ↓
Jupyter analysis
        ↓
Planned vs. actual trajectory
```

## Build

From Linux/Xavier:

```bash
./gradlew :TeamCode:assembleDebug
```

From Windows:

```powershell
.\gradlew.bat :TeamCode:assembleDebug
```

Android Studio can also be used to sync, build, and deploy the project to the
Control Hub.