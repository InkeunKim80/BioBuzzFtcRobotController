package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.commands.Commands.instant;

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

    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;
    private static final double TRIGGER_THRESHOLD = 0.1;

    @Override
    public void start() {
        Servo gateServo = hardwareMap.get(Servo.class, "gateServo");
        shooterDual.setGateServo(gateServo);

        CommandGamepad gp2 = new CommandGamepad(gamepad2);

        gp2.a().onTrue(shooterDual.forward());
        gp2.b().onTrue(shooterDual.off());
        gp2.x().onTrue(shooterDual.reverse());

        gp2.leftStickButton().onTrue(shooterDual.enableRPMControl());
        gp2.rightStickButton().onTrue(shooterDual.enablePowerControl());
    }

    @Override
    public void periodic() {
        shooterDual.periodic();

        // LB/RB: Click to increment/decrement by 0.01 (once per press)
        if (gamepad2.left_bumper && !prevLeftBumper) {
            shooterDual.setPowerMultiplier(shooterDual.getPowerMultiplier() - 0.01);
        }
        prevLeftBumper = gamepad2.left_bumper;

        if (gamepad2.right_bumper && !prevRightBumper) {
            shooterDual.setPowerMultiplier(shooterDual.getPowerMultiplier() + 0.01);
        }
        prevRightBumper = gamepad2.right_bumper;

        // LT/RT: Hold to continuously adjust power multiplier
        if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            shooterDual.setPowerMultiplier(shooterDual.getPowerMultiplier() - 0.02);
        }
        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            shooterDual.setPowerMultiplier(shooterDual.getPowerMultiplier() + 0.02);
        }

        telemetry.addData("Shooter State", shooterDual.getState());
        telemetry.addData("Shooter Motor 1 RPM", shooterDual.getRPM1());
        telemetry.addData("Shooter Motor 2 RPM", shooterDual.getRPM2());
        telemetry.addData("Shooter Motor 1 Power", shooterDual.getPower1());
        telemetry.addData("Shooter Motor 2 Power", shooterDual.getPower2());
        telemetry.addData("Power Multiplier", String.format("%.2f", shooterDual.getPowerMultiplier()));
        telemetry.addData("Control Mode", shooterDual.isRPMControlEnabled() ? "RPM Control" : "Power Control");
        if (shooterDual.isRPMControlEnabled()) {
            telemetry.addData("Target RPM", String.format("%.0f", shooterDual.getTargetRPM()));
        }
        telemetry.addData("LB Pressed", gamepad2.left_bumper);
        telemetry.addData("RB Pressed", gamepad2.right_bumper);
        telemetry.addData("LT Value", String.format("%.2f", gamepad2.left_trigger));
        telemetry.addData("RT Value", String.format("%.2f", gamepad2.right_trigger));
        telemetry.addData("Gate Position", shooterDual.getGatePosition());
        telemetry.addData("Gate Status",
                shooterDual.getState() == ShooterDual.ShooterState.FORWARD ? "OPEN" : "CLOSED");
        telemetry.update();
    }
}