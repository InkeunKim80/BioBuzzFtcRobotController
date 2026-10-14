package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.drive.DriveCommands;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name = "TeleOp VT", group = "Competition")
public class TeleOp_VT extends NextOpMode {
    private final Robot robot;

    public TeleOp_VT(Robot robot) {
        super(robot);
        this.robot = robot;

        Scheduler.reset();
    }

    @Override
    public void start() {
        Servo gateServo = hardwareMap.get(Servo.class, "gateServo");
        robot.shooterDual.setGateServo(gateServo);

        CommandGamepad gp1 = new CommandGamepad(gamepad1);
        CommandGamepad gp2 = new CommandGamepad(gamepad2);

        // Controller 1: drive + shoot
        robot.drivetrain.startDrive(gamepad1);

        gp1.leftBumper().whileTrue(
                infinite(() -> DriveCommands.setScalar(0.4))
                        .setEnd(end -> DriveCommands.setScalar(1.0))
        );

        gp1.a().onTrue(robot.shooterDual.forward());
        gp1.b().onTrue(robot.shooterDual.off());
        gp1.x().onTrue(robot.shooterDual.reverse());

        gp1.dpadUp().onTrue(robot.shooterDual.increasePower());
        gp1.dpadDown().onTrue(robot.shooterDual.decreasePower());

        // Controller 2: intake + slider
        gp2.a().onTrue(robot.intake.forward());
        gp2.b().onTrue(robot.intake.off());
        gp2.x().onTrue(robot.intake.reverse());

        // Slider placeholder: hardware not yet installed
        // CommandGamepad gp2 slider controls (enable when Slider hardware is ready)
        // gp2.y().onTrue(robot.slider.extend());
        // gp2.rightBumper().onTrue(robot.slider.returnToStart());
    }

    @Override
    public void periodic() {
        telemetry.addData("Shooter State", robot.shooterDual.getState());
        telemetry.addData("Shooter RPM 1", robot.shooterDual.getRPM1());
        telemetry.addData("Shooter RPM 2", robot.shooterDual.getRPM2());
        telemetry.addData("Shooter Power 1", robot.shooterDual.getPower1());
        telemetry.addData("Shooter Power 2", robot.shooterDual.getPower2());
        telemetry.addData("Power Multiplier", String.format("%.2f", robot.shooterDual.getPowerMultiplier()));
        telemetry.addData("Gate Position", robot.shooterDual.getGatePosition());
        telemetry.addData("Intake State", robot.intake.getState());
        telemetry.addData("Intake Power", robot.intake.getPower());
        // telemetry.addData("Slider Position", robot.slider.getCurrentPosition());
        // telemetry.addData("Slider Target", robot.slider.getTargetPosition());
        telemetry.update();
    }
}
