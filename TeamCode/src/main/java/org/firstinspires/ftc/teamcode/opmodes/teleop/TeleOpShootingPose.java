package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.IndicatorLight;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name = "TeleOp Shooting Pose", group = "Competition")
public class TeleOpShootingPose extends NextOpMode {
    private final Robot robot;
    private Follower follower;
    private boolean gamepad1Locked = false;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Placeholder coordinates: replace with the real shooting positions.
    private final Pose shootingPosition1 = poseFactory.of(48, 96, 90);
    private final Pose shootingPosition2 = poseFactory.of(96, 48, 0);

    public TeleOpShootingPose(Robot robot) {
        super(robot);
        this.robot = robot;

        Scheduler.reset();
    }

    @Override
    public void disabledPeriodic() {
        if (follower == null) {
            follower = Constants.create(hardwareMap);
            if (follower != null) {
                Pose savedPose = PoseStorage.load();
                if (savedPose != null) {
                    follower.setPose(savedPose);
                }
                follower.update();
            }
        }
        telemetry.addData("Follower", follower != null ? "Initialized" : "Not initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        CommandGamepad gp1 = new CommandGamepad(gamepad1);
        CommandGamepad gp2 = new CommandGamepad(gamepad2);

        robot.gate.setServo(hardwareMap.get(CRServo.class, "gateServo"));
        robot.intakeServo.setServo(hardwareMap.get(CRServo.class, "intakeServo"));
        robot.flower.setServo(hardwareMap.get(Servo.class, "flowerServo"));
        robot.indicator.setServo(hardwareMap.get(Servo.class, "rgbLight"));

        // Controller 1: drive, shooting position correction, flower
        robot.drivetrain.startDrive(gamepad1);

        gp1.leftBumper().onTrue(instant(() -> {
            if (!gamepad1Locked) goTo(shootingPosition1);
        }));
        gp1.rightBumper().onTrue(instant(() -> {
            if (!gamepad1Locked) goTo(shootingPosition2);
        }));

        gp1.b().onTrue(instant(() -> {
            if (!gamepad1Locked) schedule(robot.flower.startSweep());
        }));
        gp1.b().onFalse(robot.flower.stopSweep());

        // Controller 2: intake + shooter
        gp2.b().onTrue(off());
        gp2.a().onTrue(shoot());
        gp2.x().onTrue(intakeon());
        gp2.y().onTrue(robot.gate.run());
        gp2.y().onFalse(robot.gate.stop());

        gp2.dpadUp().onTrue(robot.shooterDual.increasePower());
        gp2.dpadDown().onTrue(robot.shooterDual.decreasePower());
    }

    private void goTo(Pose target) {
        Pose current = follower.pose();
        schedule(sequential(
                instant(() -> robot.drivetrain.stopDrive()),
                follow(follower, line(current, target).linear(current, target)),
                robot.indicator.set(IndicatorLight.GREEN),
                instant(() -> gamepad1Locked = true),
                waitMs(3000),
                robot.indicator.set(IndicatorLight.OFF),
                instant(() -> {
                    gamepad1Locked = false;
                    robot.drivetrain.startDrive(gamepad1);
                })
        ));
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
        if (follower != null) {
            follower.update();
            telemetry.addData("X", "%.2f", follower.pose().x());
            telemetry.addData("Y", "%.2f", follower.pose().y());
            telemetry.addData("Heading", "%.1f", Math.toDegrees(follower.pose().heading()));
        }

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
        telemetry.update();
    }
}
