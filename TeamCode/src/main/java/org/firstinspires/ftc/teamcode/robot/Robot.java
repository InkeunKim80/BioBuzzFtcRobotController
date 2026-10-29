package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.mechanisms.Flower;
import org.firstinspires.ftc.teamcode.mechanisms.Gate;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeServo;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
// import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.ShooterDual;
import org.firstinspires.ftc.teamcode.mechanisms.Slider;
import org.firstinspires.ftc.teamcode.mechanisms.Scoop;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public class Robot implements NextRobot {
    public final Drivetrain drivetrain = new Drivetrain();
    public final Intake intake = new Intake();
    public final IntakeServo intakeServo = new IntakeServo();
    public final ShooterDual shooterDual = new ShooterDual();
    // public final Shooter shooter = new Shooter();
    public final Gate gate = new Gate();
    public final Flower flower = new Flower();
    public final Slider slider = new Slider();
    public final Scoop scoop = new Scoop();

    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(drivetrain, intake, intakeServo, shooterDual, gate, flower, slider, scoop);
    }
}
