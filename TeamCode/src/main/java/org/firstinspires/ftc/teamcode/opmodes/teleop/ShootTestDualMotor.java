package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.ShooterDual;
import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name = "Shoot Test Dual Motor", group = "Tests")
public class ShootTestDualMotor extends NextOpMode {
    private final Robot robot;
    private final ShooterDual shooterDual;

    public ShootTestDualMotor(Robot robot) {
        super(robot);
        this.robot = robot;
        this.shooterDual = new ShooterDual();
        Scheduler.reset();
    }

    @Override
    public void start() {
        Servo gateServo = hardwareMap.get(Servo.class, "gateServo");
        shooterDual.setGateServo(gateServo);

        CommandGamepad gp2 = new CommandGamepad(gamepad2);

        gp2.a().onTrue(shooterDual.forward());
        gp2.b().onTrue(shooterDual.off());
        gp2.x().onTrue(shooterDual.reverse());

        gp2.leftBumper().onTrue(shooterDual.adjustLeftBumper());
        gp2.rightBumper().onTrue(shooterDual.adjustRightBumper());
    }

    @Override
    public void periodic() {
        shooterDual.periodic();
        telemetry.addData("Shooter State", shooterDual.getState());
        telemetry.addData("Shooter Motor 1 RPM", shooterDual.getRPM1());
        telemetry.addData("Shooter Motor 2 RPM", shooterDual.getRPM2());
        telemetry.addData("Shooter Motor 1 Power", shooterDual.getPower1());
        telemetry.addData("Shooter Motor 2 Power", shooterDual.getPower2());
        telemetry.addData("Power Multiplier", String.format("%.1f", shooterDual.getPowerMultiplier()));
        telemetry.addData("Gate Position", shooterDual.getGatePosition());
        telemetry.addData("Gate Status",
                shooterDual.getState() == ShooterDual.ShooterState.FORWARD ? "OPEN" : "CLOSED");
        telemetry.update();
    }
}