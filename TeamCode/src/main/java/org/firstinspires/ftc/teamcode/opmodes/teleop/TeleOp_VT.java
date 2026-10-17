package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.waitMs;
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
    private static final double GATE_STEP = 0.05;

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

        // Controller 1: drive
        robot.drivetrain.startDrive(gamepad1);

        gp1.leftBumper().whileTrue(
                infinite(() -> DriveCommands.setScalar(0.4))
                        .setEnd(end -> DriveCommands.setScalar(1.0))
        );

        // Controller 2: intake + shooter + slider
        //gp2.a().onTrue(robot.intake.forward());
        gp2.a().onTrue(robot.shooterDual.forward());
        gp2.x().onTrue(robot.intake.reverse());
        gp2.y().onTrue(robot.intake.off());
        gp2.b().onTrue(robot.shooterDual.off());
        //gp2.b().onTrue(robot.gate.toggle());

        //gp2.x().onTrue(robot.shooterDual.forward());

        gp2.dpadUp().onTrue(robot.shooterDual.increasePower());
        gp2.dpadDown().onTrue(robot.shooterDual.decreasePower());

        // Gate toggle on Y
        robot.gate.setServo(hardwareMap.get(Servo.class, "gateServo"));
        //gp2.a().onTrue(robot.gate.toggle());
        gp1.leftBumper().onTrue(robot.gate.setPosition(0.25));
        gp1.rightBumper().onTrue(robot.gate.setPosition(0.9));

        // Gate fine position control: dpad left/right
        gp2.dpadLeft().onTrue(robot.gate.adjust(-GATE_STEP));
        gp2.dpadRight().onTrue(robot.gate.adjust(GATE_STEP));

        // Flower toggle on B
        robot.flower.setServo(hardwareMap.get(CRServo.class, "flowerServo"));
        gp1.b().onTrue(robot.flower.toggle());


        // Slider placeholder: hardware not yet installed
        // CommandGamepad gp2 slider controls (enable when Slider hardware is ready)
        // gp2.rightBumper().onTrue(robot.slider.returnToStart());
    }
/*
    private Command off() {
        return sequential(
                robot.shooterDual.off(),
                robot.intake.off(),
                robot.gate.close()
        );
    }

    private Command Shoot() {
        return sequential(
                robot.shooterDual.forward(),
                robot.gate.open(),
                waitMs(500),
                robot.intake.reverse()
        );
    }

    private Command intakeon() {
        return sequential(
                robot.intake.reverse(),
                robot.shooterDual.off(),
                robot.gate.close()
        );
    }

    private Command intakeoff() {
        return sequential(
                robot.intake.off()
                //robot.shooterDual.off(),
                //robot.gate.toggle()
        );
    }
        */

    @Override
    public void periodic() {
        telemetry.addData("Shooter State", robot.shooterDual.getState());
        telemetry.addData("Shooter RPM 1", robot.shooterDual.getRPM1());
        telemetry.addData("Shooter RPM 2", robot.shooterDual.getRPM2());
        telemetry.addData("Shooter Power 1", robot.shooterDual.getPower1());
        telemetry.addData("Shooter Power 2", robot.shooterDual.getPower2());
        telemetry.addData("Power Multiplier", String.format("%.2f", robot.shooterDual.getPowerMultiplier()));
        telemetry.addData("Intake State", robot.intake.getState());
        telemetry.addData("Intake Power", robot.intake.getPower());
        telemetry.addData("Gate Position", String.format("%.2f", robot.gate.getPosition()));
        // telemetry.addData("Slider Position", robot.slider.getCurrentPosition());
        // telemetry.addData("Slider Target", robot.slider.getTargetPosition());
        telemetry.update();
    }
}
