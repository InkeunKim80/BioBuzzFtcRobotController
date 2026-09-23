# Pulling pose logs off the robot

`PoseLogger` writes the robot pose to a CSV file during an OpMode.

The OpMode should create a `PoseLogger` with its own name, for example:

```java
PoseLogger poseLogger = new PoseLogger("ExampleAuto");
```

The resulting file is stored on the Control Hub under:

```text
/sdcard/FIRST/localization_logs/<OpModeName>_YYYYMMDD_HHMMSS.csv
```

For example:

```text
/sdcard/FIRST/localization_logs/ExampleAuto_20260923_153000.csv
```

Each CSV contains:

```text
time_s,x_in,y_in,heading_deg
```

where:

- `time_s` is elapsed time since the logger was created.
- `x_in` is the robot X position in inches.
- `y_in` is the robot Y position in inches.
- `heading_deg` is the robot heading in degrees.

Pose samples are buffered and periodically flushed to storage. The logger should still be closed when the OpMode finishes.

## Option 1: adb pull over USB

Connect the Control Hub to the computer with USB.

Confirm that adb can see the device:

```bash
adb devices
```

Pull the localization logs into the repository:

```bash
adb pull /sdcard/FIRST/localization_logs/ ./logs/
```

## Option 2: Control Hub Wi-Fi + adb

Connect the computer to the Control Hub Wi-Fi network.

The Control Hub normally uses:

```text
192.168.43.1
```

Connect adb:

```bash
adb connect 192.168.43.1:5555
```

Then pull the logs:

```bash
adb pull /sdcard/FIRST/localization_logs/ ./logs/
```

## Option 3: Control Hub web interface

Connect to the Control Hub Wi-Fi network.

Open:

```text
http://192.168.43.1:8080
```

Then go to:

```text
Program & Manage -> Manage -> Files
```

Browse to:

```text
FIRST/localization_logs/
```

and download the desired CSV file into the repository's `logs/` directory.

## Analyze the log

The Jupyter notebook is located at:

```text
analysis/notebooks/localization_validation.ipynb
```

The notebook loads pose data from the repository's `logs/` directory and plots:

- X/Y robot trajectory
- X position versus time
- Y position versus time
- Heading versus time

The logged trajectory can later be compared with the planned Pedro Pathing trajectory to evaluate localization and path-following performance.