package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.drive.DriveCommands;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name="Drive Test", group = "Tests")
public class DriveTest extends NextOpMode {
    private final Robot robot;

    public DriveTest(Robot robot) {
        super(robot);
        this.robot = robot;

        Scheduler.reset();
    }

    @Override
    public void start() {
        CommandGamepad gp1 = new CommandGamepad(gamepad1);

        robot.drivetrain.startDrive(gamepad1);

        gp1.leftBumper().whileTrue(
                infinite(() -> DriveCommands.setScalar(0.4))
                        .setEnd(end -> DriveCommands.setScalar(1.0))
        );
    }

    @Override
    public void periodic() {
        telemetry.addData("Front Left Power", robot.drivetrain.frontLeft.getThrottle());
        telemetry.addData("Front Right Power", robot.drivetrain.frontRight.getThrottle());
        telemetry.addData("Back Left Power", robot.drivetrain.backLeft.getThrottle());
        telemetry.addData("Back Right Power", robot.drivetrain.backRight.getThrottle());
        telemetry.update();
    }
}
