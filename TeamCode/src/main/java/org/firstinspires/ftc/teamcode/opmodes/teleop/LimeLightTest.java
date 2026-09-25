package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import edu.wpi.first.networktables.NetworkTableInstance;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name="LimeLight Test", group = "Tests")
public class LimeLightTest extends NextOpMode {
    private static final String LIMELIGHT_NAME = "limelight";

    public LimeLightTest() {
        Scheduler.reset();
    }

    @Override
    public void start() {
        CommandGamepad gp1 = new CommandGamepad(gamepad1);

        gp1.a().onTrue(instant(() -> {
            setLEDMode(3);
            telemetry.addLine("LED Mode: On (Full)");
        }));

        gp1.b().onTrue(instant(() -> {
            setLEDMode(1);
            telemetry.addLine("LED Mode: Off");
        }));

        gp1.x().onTrue(instant(() -> {
            setLEDMode(0);
            telemetry.addLine("LED Mode: Default");
        }));

        gp1.y().onTrue(instant(() -> {
            setLEDMode(2);
            telemetry.addLine("LED Mode: Blink");
        }));
    }

    @Override
    public void periodic() {
        double tx = getLimelightValue("tx", 0.0);
        double ty = getLimelightValue("ty", 0.0);
        double ta = getLimelightValue("ta", 0.0);
        double tv = getLimelightValue("tv", 0.0);

        boolean targetDetected = tv != 0.0;

        telemetry.addData("=== LIMELIGHT STATUS ===", "");
        telemetry.addData("Target Detected", targetDetected ? "✓ YES" : "NO");

        if (targetDetected) {
            telemetry.addData("", "");
            telemetry.addData("=== TARGET INFO ===", "");
            telemetry.addData("Horizontal Offset (tx)", String.format("%.2f°", tx));
            telemetry.addData("Vertical Offset (ty)", String.format("%.2f°", ty));
            telemetry.addData("Target Area (ta)", String.format("%.2f%%", ta));
        } else {
            telemetry.addData("", "");
            telemetry.addData("STATUS", "Searching for target...");
        }

        telemetry.addData("", "");
        telemetry.addData("=== LED CONTROLS ===", "");
        telemetry.addData("A Button", "LED On (Full)");
        telemetry.addData("B Button", "LED Off");
        telemetry.addData("X Button", "LED Default");
        telemetry.addData("Y Button", "LED Blink");

        telemetry.update();
    }

    private double getLimelightValue(String key, double defaultValue) {
        try {
            return NetworkTableInstance.getInstance().getTable(LIMELIGHT_NAME).getEntry(key).getDouble(defaultValue);
        } catch (Exception e) {
            return defaultValue;
        }
    }

    private void setLEDMode(int mode) {
        try {
            NetworkTableInstance.getInstance().getTable(LIMELIGHT_NAME).getEntry("ledMode").setNumber(mode);
        } catch (Exception e) {
            telemetry.addLine("Error setting LED mode");
        }
    }
}
