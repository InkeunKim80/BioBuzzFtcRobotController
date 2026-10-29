package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.CRServo;
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

        CommandGamepad gp1 = new CommandGamepad(gamepad1);
        CommandGamepad gp2 = new CommandGamepad(gamepad2);

        robot.gate.setServo(hardwareMap.get(CRServo.class, "gateServo"));
        robot.intakeServo.setServo(hardwareMap.get(CRServo.class, "intakeServo"));
        robot.flower.setServo(hardwareMap.get(Servo.class, "flowerServo"));

        // Controller 1: drive
        robot.drivetrain.startDrive(gamepad1);

        gp1.leftBumper().whileTrue(
                infinite(() -> DriveCommands.setScalar(0.4))
                        .setEnd(end -> DriveCommands.setScalar(1.0))
        );
        // Flower toggle on B
        
        gp1.b().onTrue(robot.flower.startSweep());
        gp1.b().onFalse(robot.flower.stopSweep());

        // Controller 2: intake + shooter + slider
        gp2.b().onTrue(off());
        gp2.a().onTrue(shoot());
        gp2.x().onTrue(intakeon());
        //gp2.y().onTrue(intakeoff());
        gp2.y().onTrue(robot.gate.run());
        gp2.y().onFalse(robot.gate.stop());

        gp2.dpadUp().onTrue(robot.shooterDual.increasePower());
        gp2.dpadDown().onTrue(robot.shooterDual.decreasePower());

        // Slider placeholder: hardware not yet installed
        // CommandGamepad gp2 slider controls (enable when Slider hardware is ready)
        // gp2.rightBumper().onTrue(robot.slider.returnToStart());

    }

    private Command off() {
        return parallel(
                robot.shooterDual.off(),
                robot.intake.off(),
                robot.intakeServo.off(),
                robot.gate.stop()
        );
    }

    private Command shoot() {
        return sequential(
                robot.shooterDual.reverse(),
                waitMs(500),
                robot.intake.reverse(),
                robot.intakeServo.forward()
        );
    }

    private Command intakeon() {
        return sequential(
                robot.shooterDual.reverse(),
                robot.intake.reverse(),
                robot.intakeServo.forward()
        );
    }
        

    @Override
    public void periodic() {
        telemetry.addData("Shooter State", robot.shooterDual.getState());
        telemetry.addData("Shooter Current RPM", String.format("%.0f", robot.shooterDual.getRPM1()));
        telemetry.addData("Shooter Target RPM", String.format("%.0f", robot.shooterDual.getTargetRpm()));
        telemetry.addData("RPM Control", robot.shooterDual.isRpmControlEnabled() ? "ON" : "OFF");
        telemetry.addData("Shooter Power", robot.shooterDual.getPower1());
        telemetry.addData("Power Multiplier", String.format("%.2f", robot.shooterDual.getPowerMultiplier()));
        telemetry.addData("Intake State", robot.intake.getState());
        telemetry.addData("Intake Power", robot.intake.getPower());
        telemetry.addData("Intake Servo Power", robot.intakeServo.getPower());
        telemetry.addData("Gate Power", String.format("%.2f", robot.gate.getPower()));
        // telemetry.addData("Slider Position", robot.slider.getCurrentPosition());
        // telemetry.addData("Slider Target", robot.slider.getTargetPosition());
        telemetry.update();
    }
}
