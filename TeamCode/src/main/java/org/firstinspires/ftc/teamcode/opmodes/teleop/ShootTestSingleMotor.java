package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name = "Shoot Test Single Motor", group = "Tests")
public class ShootTestSingleMotor extends NextOpMode {
    private final Robot robot;

    public ShootTestSingleMotor(Robot robot) {
        super(robot);
        this.robot = robot;

        Scheduler.reset();
    }

    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;
    private static final double TRIGGER_THRESHOLD = 0.1;

    @Override
    public void start() {
        Servo gateServo = hardwareMap.get(Servo.class, "gateServo");
        robot.shooter.setGateServo(gateServo);

        CommandGamepad gp2 = new CommandGamepad(gamepad2);

        gp2.a().onTrue(robot.shooter.forward());
        gp2.b().onTrue(robot.shooter.off());
        gp2.x().onTrue(robot.shooter.reverse());
    }

    @Override
    public void periodic() {
        robot.shooter.periodic();

        // LB/RB: Click to increment/decrement by 0.01 (once per press)
        if (gamepad2.left_bumper && !prevLeftBumper) {
            robot.shooter.setPowerMultiplier(robot.shooter.getPowerMultiplier() - 0.01);
        }
        prevLeftBumper = gamepad2.left_bumper;

        if (gamepad2.right_bumper && !prevRightBumper) {
            robot.shooter.setPowerMultiplier(robot.shooter.getPowerMultiplier() + 0.01);
        }
        prevRightBumper = gamepad2.right_bumper;

        // LT/RT: Hold to continuously adjust power multiplier
        if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            robot.shooter.setPowerMultiplier(robot.shooter.getPowerMultiplier() - 0.02);
        }
        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            robot.shooter.setPowerMultiplier(robot.shooter.getPowerMultiplier() + 0.02);
        }

        telemetry.addData("Shooter State", robot.shooter.getState());
        telemetry.addData("Shooter Power", robot.shooter.getPower());
        telemetry.addData("Power Multiplier", String.format("%.2f", robot.shooter.getPowerMultiplier()));
        telemetry.addData("LB Pressed", gamepad2.left_bumper);
        telemetry.addData("RB Pressed", gamepad2.right_bumper);
        telemetry.addData("LT Value", String.format("%.2f", gamepad2.left_trigger));
        telemetry.addData("RT Value", String.format("%.2f", gamepad2.right_trigger));
        telemetry.addData("Gate Position", robot.shooter.getGatePosition());
        telemetry.addData("Gate Status",
                robot.shooter.getState() == Shooter.ShooterState.FORWARD ? "OPEN" : "CLOSED");
        telemetry.update();
    }
}