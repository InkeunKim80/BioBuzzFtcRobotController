package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name = "Shooting Tuning", group = "Tuning")
public class ShootingTuning extends NextOpMode {
    private static final double KP_STEP = 0.0001;
    private static final double KI_STEP = 0.00001;

    private final Robot robot;

    public ShootingTuning(Robot robot) {
        super(robot);
        this.robot = robot;

        Scheduler.reset();
    }

    @Override
    public void start() {
        CommandGamepad gp1 = new CommandGamepad(gamepad1);

        gp1.a().onTrue(robot.shooterDual.reverse());

        gp1.dpadUp().onTrue(instant(() -> robot.shooterDual.setKp(robot.shooterDual.getKp() + KP_STEP)));
        gp1.dpadDown().onTrue(instant(() -> robot.shooterDual.setKp(robot.shooterDual.getKp() - KP_STEP)));
        gp1.dpadRight().onTrue(instant(() -> robot.shooterDual.setKi(robot.shooterDual.getKi() + KI_STEP)));
        gp1.dpadLeft().onTrue(instant(() -> robot.shooterDual.setKi(robot.shooterDual.getKi() - KI_STEP)));
    }

    @Override
    public void periodic() {
        telemetry.addData("Shooter State", robot.shooterDual.getState());
        telemetry.addData("Target RPM", String.format("%.0f", robot.shooterDual.getTargetRpm()));
        telemetry.addData("Current RPM", String.format("%.0f", robot.shooterDual.getRPM1()));
        telemetry.addData("Shooter Power", String.format("%.3f", robot.shooterDual.getPower1()));
        telemetry.addData("KP", String.format("%.6f", robot.shooterDual.getKp()));
        telemetry.addData("KI", String.format("%.7f", robot.shooterDual.getKi()));
        telemetry.addData("Controls", "A reverse shooter, D-pad Up/Down KP, Left/Right KI");
        telemetry.update();
    }
}
