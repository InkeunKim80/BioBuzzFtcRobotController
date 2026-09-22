package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public class Robot implements NextRobot {
    public final Drivetrain drivetrain = new Drivetrain();
    public final Intake intake = new Intake();

    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(drivetrain, intake);
    }
}
