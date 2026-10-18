package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name="Slider Test", group = "Tests")
public class SliderTest extends NextOpMode {
    private final Robot robot;

    public SliderTest(Robot robot) {
        super(robot);
        this.robot = robot;

        Scheduler.reset();
    }

    @Override
    public void start() {
        CommandGamepad gp2 = new CommandGamepad(gamepad2);

        gp2.a().onTrue(robot.slider.extend());
        gp2.x().onTrue(robot.slider.returnToStart());
    }

    @Override
    public void periodic() {
        double currentPos = robot.slider.getCurrentPosition();
        double targetPos = robot.slider.getTargetPosition();
        double difference = targetPos - currentPos;
        double progressPercent = (currentPos / 2000.0) * 100;
        boolean atTarget = robot.slider.isAtTarget();

        telemetry.addData("=== SLIDER POSITION ===", "");
        telemetry.addData("Current Position", String.format("%.0f units", currentPos));
        telemetry.addData("Target Position", String.format("%.0f units", targetPos));
        telemetry.addData("Remaining Distance", String.format("%.0f units", difference));
        telemetry.addData("Progress", String.format("%.1f%%", progressPercent));
        telemetry.addData("At Target", atTarget ? "✓ YES" : "NO");

        telemetry.addData("", "");
        telemetry.addData("=== MOTOR STATUS ===", "");
        telemetry.addData("Motor Power", String.format("%.2f", robot.slider.getPower()));
        telemetry.addData("Motor RPM", String.format("%.1f", robot.slider.getRPM()));

        telemetry.addData("", "");
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("A Button", "Extend to 2000");
        telemetry.addData("X Button", "Return to Start (0)");

        telemetry.update();
    }
}
