package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name="Shoot Test Single Motor", group = "Tests")
public class ShootTestSingleMotor extends NextOpMode {
    private final Robot robot;

    public ShootTestSingleMotor(Robot robot) {
        super(robot);
        this.robot = robot;

        Scheduler.reset();
    }

    @Override
    public void start() {
        Servo gateServo = hardwareMap.get(Servo.class, "gateServo");
        robot.shooter.setGateServo(gateServo);

        CommandGamepad gp2 = new CommandGamepad(gamepad2);

        gp2.a().onTrue(robot.shooter.forward());
        gp2.b().onTrue(robot.shooter.off());
        gp2.x().onTrue(robot.shooter.reverse());
        gp2.y().onTrue(robot.shooter.cycle());
    }

    @Override
    public void periodic() {
        telemetry.addData("Shooter State", robot.shooter.getState());
        telemetry.addData("Shooter RPM", robot.shooter.getRPM());
        telemetry.addData("Shooter Power", robot.shooter.getPower());
        telemetry.addData("Gate Position", robot.shooter.getGatePosition());
        telemetry.addData("Gate Status", robot.shooter.getState() == robot.shooter.ShooterState.FORWARD ? "OPEN" : "CLOSED");
        telemetry.update();
    }
}
