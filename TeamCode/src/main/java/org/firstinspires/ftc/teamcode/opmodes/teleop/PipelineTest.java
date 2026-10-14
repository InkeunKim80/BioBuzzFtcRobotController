package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.ShooterDual;
import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name = "Pipeline Test", group = "Tests")
public class PipelineTest extends NextOpMode {
    private final Robot robot;
    private final ShooterDual shooterDual;
    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;
    private static final double TRIGGER_THRESHOLD = 0.1;

    public PipelineTest(Robot robot) {
        super(robot);
        this.robot = robot;
        this.shooterDual = new ShooterDual();
        Scheduler.reset();
    }

    @Override
    public void start() {
        Servo gateServo = hardwareMap.get(Servo.class, "gateServo");
        shooterDual.setGateServo(gateServo);

        CommandGamepad gp1 = new CommandGamepad(gamepad1);
        gp1.a().onTrue(robot.intake.forward());
        gp1.b().onTrue(robot.intake.off());
        gp1.x().onTrue(robot.intake.reverse());

        CommandGamepad gp2 = new CommandGamepad(gamepad2);
        gp2.a().onTrue(shooterDual.forward());
        gp2.b().onTrue(shooterDual.off());
        gp2.x().onTrue(shooterDual.reverse());
    }

    @Override
    public void periodic() {
        shooterDual.periodic();

        if (gamepad2.left_bumper && !prevLeftBumper) {
            shooterDual.setPowerMultiplier(shooterDual.getPowerMultiplier() - 0.01);
        }
        prevLeftBumper = gamepad2.left_bumper;

        if (gamepad2.right_bumper && !prevRightBumper) {
            shooterDual.setPowerMultiplier(shooterDual.getPowerMultiplier() + 0.01);
        }
        prevRightBumper = gamepad2.right_bumper;

        if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            shooterDual.setPowerMultiplier(shooterDual.getPowerMultiplier() - 0.02);
        }
        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            shooterDual.setPowerMultiplier(shooterDual.getPowerMultiplier() + 0.02);
        }

        telemetry.addData("Intake State", robot.intake.getState());
        telemetry.addData("Intake RPM", robot.intake.getRPM());
        telemetry.addData("Intake Power", robot.intake.getPower());
        telemetry.addData("Shooter State", shooterDual.getState());
        telemetry.addData("Shooter Motor 1 RPM", shooterDual.getRPM1());
        telemetry.addData("Shooter Motor 2 RPM", shooterDual.getRPM2());
        telemetry.addData("Shooter Motor 1 Power", shooterDual.getPower1());
        telemetry.addData("Shooter Motor 2 Power", shooterDual.getPower2());
        telemetry.addData("Power Multiplier", String.format("%.2f", shooterDual.getPowerMultiplier()));
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