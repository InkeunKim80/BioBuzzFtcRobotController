## TeamCode Module

Welcome!

This module, TeamCode, is the place where you will write/paste the code for your team's
robot controller App. This module is currently empty (a clean slate) but the
process for adding OpModes is straightforward.

## Creating your own OpModes

The easiest way to create your own OpMode is to copy a Sample OpMode and make it your own.

Sample opmodes exist in the FtcRobotController module.
To locate these samples, find the FtcRobotController module in the "Project/Android" tab.

Expand the following tree elements:
 FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples

### Naming of Samples

To gain a better understanding of how the samples are organized, and how to interpret the
naming system, it will help to understand the conventions that were used during their creation.

These conventions are described (in detail) in the sample_conventions.md file in this folder.

To summarize: A range of different samples classes will reside in the java/external/samples.
The class names will follow a naming convention which indicates the purpose of each class.
The prefix of the name will be one of the following:

Basic:  	This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones examples.

Sensor:    	This is a Sample OpMode that shows how to use a specific sensor.
            It is not intended to drive a functioning robot, it is simply showing the minimal code
            required to read and display the sensor values.

Robot:	    This is a Sample OpMode that assumes a simple two-motor (differential) drive base.
            It may be used to provide a common baseline driving OpMode, or
            to demonstrate how a particular sensor or concept can be used to navigate.

Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
            These may be complex, but their operation should be explained clearly in the comments,
            or the comments should reference an external doc, guide or tutorial.
            Each OpMode should try to only demonstrate a single concept so they are easy to
            locate based on their name.  These OpModes may not produce a drivable robot.

After the prefix, other conventions will apply:

* Sensor class names are constructed as:    Sensor - Company - Type
* Robot class names are constructed as:     Robot - Mode - Action - OpModetype
* Concept class names are constructed as:   Concept - Topic - OpModetype

Once you are familiar with the range of samples available, you can choose one to be the
basis for your own robot.  In all cases, the desired sample(s) needs to be copied into
your TeamCode module to be used.

This is done inside Android Studio directly, using the following steps:

 1) Locate the desired sample class in the Project/Android tree.

 2) Right click on the sample class and select "Copy"

 3) Expand the  TeamCode/java folder

 4) Right click on the org.firstinspires.ftc.teamcode folder and select "Paste"

 5) You will be prompted for a class name for the copy.
    Choose something meaningful based on the purpose of this class.
    Start with a capital letter, and remember that there may be more similar classes later.

Once your copy has been created, you should prepare it for use on your robot.
This is done by adjusting the OpMode's name, and enabling it to be displayed on the
Driver Station's OpMode list.

Each OpMode sample class begins with several lines of code like the ones shown below:

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

The name that will appear on the driver station's "opmode list" is defined by the code:
 ``name="Template: Linear OpMode"``
You can change what appears between the quotes to better describe your opmode.
The "group=" portion of the code can be used to help organize your list of OpModes.

As shown, the current OpMode will NOT appear on the driver station's OpMode list because of the
  ``@Disabled`` annotation which has been included.
This line can simply be deleted , or commented out, to make the OpMode visible.



## ADVANCED Multi-Team App management:  Cloning the TeamCode Module

In some situations, you have multiple teams in your club and you want them to all share
a common code organization, with each being able to *see* the others code but each having
their own team module with their own code that they maintain themselves.

In this situation, you might wish to clone the TeamCode module, once for each of these teams.
Each of the clones would then appear along side each other in the Android Studio module list,
together with the FtcRobotController module (and the original TeamCode module).

Selective Team phones can then be programmed by selecting the desired Module from the pulldown list
prior to clicking to the green Run arrow.

Warning:  This is not for the inexperienced Software developer.
You will need to be comfortable with File manipulations and managing Android Studio Modules.
These changes are performed OUTSIDE of Android Studios, so close Android Studios before you do this.
 
Also.. Make a full project backup before you start this :)

To clone TeamCode, do the following:

Note: Some names start with "Team" and others start with "team".  This is intentional.

1)  Using your operating system file management tools, copy the whole "TeamCode"
    folder to a sibling folder with a corresponding new name, eg: "Team0417".

2)  In the new Team0417 folder, delete the TeamCode.iml file.

3)  the new Team0417 folder, rename the "src/main/java/org/firstinspires/ftc/teamcode" folder
    to a matching name with a lowercase 'team' eg:  "team0417".

4)  In the new Team0417/src/main folder, edit the "AndroidManifest.xml" file, change the line that contains
         package="org.firstinspires.ftc.teamcode"
    to be
         package="org.firstinspires.ftc.team0417"

5)  Add:    include ':Team0417' to the "/settings.gradle" file.
    
6)  Open up Android Studios and clean out any old files by using the menu to "Build/Clean Project"

---

## Hardware Configuration Guide

### Motor Names and Control Hub Pin Assignments

This section documents the motor configuration used in the BioBuzz FTC Robot Controller.

#### Drivetrain Motors (Mecanum Drive)

| Motor Name | Function | Control Hub Port | Direction | RPM |
|------------|----------|-----------------|-----------|-----|
| `frontLeft` | Front-Left Motor | Port 0 | FORWARD | 312 |
| `frontRight` | Front-Right Motor | Port 1 | REVERSE | 312 |
| `backLeft` | Back-Left Motor | Port 2 | FORWARD | 312 |
| `backRight` | Back-Right Motor | Port 3 | REVERSE | 312 |

**Strafe Compensation**: 1.1 (adjusted for mechanical variance in strafe movement)

**Motor Spec**: GoBILDA 312 RPM Motor

#### Intake System

| Motor Name | Function | Control Hub Port | Direction | RPM |
|------------|----------|-----------------|-----------|-----|
| `intakeMotor` | Intake Motor | Port 4 | REVERSE | 312 |

**Motor Spec**: GoBILDA 312 RPM Motor

#### Shooting System

The shooting system can be configured with two options:

##### Option 1: Single Motor Shooter

| Motor Name | Function | Control Hub Port | Direction | RPM |
|------------|----------|-----------------|-----------|-----|
| `shootMotor` | Shooting Motor | Port 5 | REVERSE | 6000 |

**Motor Spec**: GoBILDA 6000 RPM Motor (high-speed shooter)  
**Use Case**: Single flywheel shooter for basic shooting functionality

##### Option 2: Dual Motor Shooter

| Motor Name | Function | Control Hub Port | Direction | RPM |
|------------|----------|-----------------|-----------|-----|
| `shootMotor1` | Primary Shooting Motor | Port 5 | REVERSE | 6000 |
| `shootMotor2` | Secondary Shooting Motor | Port 6 | REVERSE | 6000 |

**Motor Spec**: GoBILDA 6000 RPM Motors (dual flywheel shooter)  
**Use Case**: Dual flywheel shooter for increased shooting power and accuracy

#### Slider Arm System

| Motor Name | Function | Control Hub Port | Direction | RPM |
|------------|----------|-----------------|-----------|-----|
| `sliderMotor` | Slider Arm Motor | Port 7 | FORWARD | ??? |

**Motor Spec**: GoBILDA ??? RPM Motor (adjustable arm positioning)  
**Use Case**: Linear motion for arm extension/retraction

#### Shooting Gate Servo

| Servo Name | Function | Control Hub Servo Port | Position |
|------------|----------|----------------------|----------|
| `gateServo` | Shooting Gate | Servo Port 0 | 0.0 (Closed) - 1.0 (Open) |

**Function**: 
- **Gate Opens** when shooter is in FORWARD state (shooting active)
- **Gate Closes** automatically when shooter state changes to OFF or REVERSE
- Provides ball retention and smooth release control

#### Scoop Servo

| Servo Name | Function | Control Hub Servo Port | Position |
|------------|----------|----------------------|----------|
| `scoopServo` | Intake Scoop | Servo Port 1 | 0.0 (Closed) - 1.0 (Open) |

**Function**: 
- **Scoop Opens** for ball delivery/discharge
- **Scoop Closes** for ball collection and retention
- Manual control via gamepad

---

### Control Hub Port Reference

#### Configuration - Option 1 (Single Shooter)

```
Control Hub Motor Ports (0-3): Drivetrain Motors (312 RPM)
├─ Port 0: frontLeft (FORWARD, 312 RPM)
├─ Port 1: frontRight (REVERSE, 312 RPM)
├─ Port 2: backLeft (FORWARD, 312 RPM)
└─ Port 3: backRight (REVERSE, 312 RPM)

Control Hub Motor Ports (4-7): Other Systems
├─ Port 4: intakeMotor (REVERSE, 312 RPM)
├─ Port 5: shootMotor (REVERSE, 6000 RPM)
├─ Port 6: Reserved for future mechanisms
└─ Port 7: sliderMotor (FORWARD, ???)
```

#### Configuration - Option 2 (Dual Shooter)

```
Control Hub Motor Ports (0-3): Drivetrain Motors (312 RPM)
├─ Port 0: frontLeft (FORWARD, 312 RPM)
├─ Port 1: frontRight (REVERSE, 312 RPM)
├─ Port 2: backLeft (FORWARD, 312 RPM)
└─ Port 3: backRight (REVERSE, 312 RPM)

Control Hub Motor Ports (4-7): Other Systems
├─ Port 4: intakeMotor (REVERSE, 312 RPM)
├─ Port 5: shootMotor1 (REVERSE, 6000 RPM)
├─ Port 6: shootMotor2 (REVERSE, 6000 RPM)
└─ Port 7: sliderMotor (FORWARD, ???)
```

### Important Notes

- **Motor Name Matching**: The names in this table (e.g., `frontLeft`) must **exactly** match the hardware configuration names set in the Control Hub's Driver Hub configuration.
- **Direction Setting**: Motor directions are pre-configured in the code. If a motor spins in the wrong direction, adjust the direction setting in the respective mechanism class rather than reversing the port wiring.
- **Future Expansion**: Ports 5-7 are reserved for additional mechanisms (e.g., shooter, climber).

### Troubleshooting Motor Configuration

1. If a motor doesn't respond in the app, verify the name matches the Control Hub configuration.
2. If a motor spins backward, check the `Direction` setting in the code (not the hardware wiring).
3. Use the **Test OpModes** to individually verify each motor:
   - `IntakeTest`: Test intake motor only
   - `DriveTest`: Test drivetrain motors
   - `ShootTest`: Test shooting mechanism

---

## Test OpMode Controls

### DriveTest - Drivetrain Testing

| Control | Action |
|---------|--------|
| **Left Stick (Up/Down/Left/Right)** | Move robot forward/backward/strafe |
| **Right Stick (Rotate)** | Rotate robot |
| **Left Bumper (Hold)** | Reduce speed to 40% |

**Telemetry Display**:
- Front Left Power
- Front Right Power
- Back Left Power
- Back Right Power

---

### IntakeTest - Intake Motor Testing

| Control | Action |
|---------|--------|
| **Gamepad 2 - A Button** | Forward (Intake) |
| **Gamepad 2 - B Button** | Off (Stop) |
| **Gamepad 2 - X Button** | Reverse |
| **Gamepad 2 - Y Button** | Cycle State |

**Telemetry Display**:
- Intake State (FORWARD / REVERSE / OFF)
- Intake RPM
- Intake Power (Throttle)

---

### ShootTestSingleMotor - Single Motor Shooting (Option 1)

| Control | Action |
|---------|--------|
| **Gamepad 2 - A Button** | Forward (Shoot) → **Gate Opens** |
| **Gamepad 2 - B Button** | Off (Stop) → **Gate Closes** |
| **Gamepad 2 - X Button** | Reverse → **Gate Closes** |
| **Gamepad 2 - Y Button** | Cycle State |

**Telemetry Display**:
- Shooter State (FORWARD / REVERSE / OFF)
- Shooter RPM
- Shooter Power (Throttle)
- **Gate Position** (0.0 = Closed, 1.0 = Open)
- **Gate Status** (OPEN / CLOSED)

**Motors**: shootMotor (Port 5, 6000 RPM)  
**Servo**: gateServo (Servo Port 0)  
**Configuration**: Option 1 - Single Motor Shooter with Automatic Gate Control

---

### ShootTestDualMotor - Dual Motor Shooting (Option 2)

| Control | Action |
|---------|--------|
| **Gamepad 2 - A Button** | Forward (Shoot) → **Gate Opens** |
| **Gamepad 2 - B Button** | Off (Stop) → **Gate Closes** |
| **Gamepad 2 - X Button** | Reverse → **Gate Closes** |
| **Gamepad 2 - Y Button** | Cycle State |

**Telemetry Display**:
- Shooter State (FORWARD / REVERSE / OFF)
- Shooter Motor 1 RPM
- Shooter Motor 2 RPM
- Shooter Motor 1 Power (Throttle)
- Shooter Motor 2 Power (Throttle)
- **Gate Position** (0.0 = Closed, 1.0 = Open)
- **Gate Status** (OPEN / CLOSED)

**Motors**: shootMotor1 (Port 5), shootMotor2 (Port 6) - Both 6000 RPM  
**Servo**: gateServo (Servo Port 0)  
**Configuration**: Option 2 - Dual Motor Shooter with Automatic Gate Control

---

### SliderTest - Slider Arm Position Control (PID)

| Control | Action |
|---------|--------|
| **Gamepad 2 - A Button (Press Once)** | **Extend to Position 2000** (자동 이동 후 유지) |
| **Gamepad 2 - X Button (Press Once)** | **Return to Start (Position 0)** (자동 복귀) |

**특징**:
- ✅ 버튼 한 번 누르면 자동으로 목표 위치까지 이동
- ✅ 도달 후 **PID 제어로 위치 자동 유지**
- ✅ 버튼을 계속 누르고 있을 필요 없음
- ✅ 중력이나 외부 힘에 저항하며 위치 유지

**Telemetry Display**:
- Current Position (현재 위치)
- Target Position (목표 위치)
- Position Difference (차이)
- At Target (목표 도달 여부)
- Slider RPM
- Slider Power (모터 전력)

**Motor**: sliderMotor (Port 7, ??? RPM)  
**제어 방식**: PID (P=0.01, I=0.001, D=0.005)  
**Use Case**: 정확한 위치 제어 및 자동 유지

---

### ScoopTest - Scoop Servo Testing

| Control | Action |
|---------|--------|
| **Gamepad 2 - A Button** | Open Scoop |
| **Gamepad 2 - B Button** | Close Scoop |
| **Gamepad 2 - Y Button** | Toggle Scoop |

**Telemetry Display**:
- Scoop State (OPEN / CLOSED)
- Scoop Position (0.0 = Closed, 1.0 = Open)

**Servo**: scoopServo (Servo Port 1)  
**Use Case**: Manual scoop control for ball collection and discharge

---

### LimeLightTest - LimeLight Camera Testing

| Control | Action |
|---------|--------|
| **Gamepad 1 - A Button** | LED On (Full Brightness) |
| **Gamepad 1 - B Button** | LED Off |
| **Gamepad 1 - X Button** | LED Default Mode |
| **Gamepad 1 - Y Button** | LED Blink Mode |

**Telemetry Display**:
- Target Detected (✓ YES / NO)
- Horizontal Offset (tx) - Degrees
- Vertical Offset (ty) - Degrees
- Target Area (ta) - Percentage
- LED Mode Control

**Camera**: LimeLight (Network Tables)  
**Use Case**: Target detection and tracking for vision-based gameplay"