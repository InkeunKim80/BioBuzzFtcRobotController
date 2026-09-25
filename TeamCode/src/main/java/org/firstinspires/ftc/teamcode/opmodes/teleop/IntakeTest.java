package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name="Intake Test", group = "Tests")
public class IntakeTest extends NextOpMode {
    private final Robot robot;

    public IntakeTest(Robot robot) {
        super(robot);
        this.robot = robot;

        Scheduler.reset();
    }

    @Override
    public void start() {
        CommandGamepad gp2 = new CommandGamepad(gamepad2);

        gp2.a().onTrue(robot.intake.forward());
        gp2.b().onTrue(robot.intake.off());
        gp2.x().onTrue(robot.intake.reverse());
        gp2.y().onTrue(robot.intake.cycle());
    }

    @Override
    public void periodic() {
        telemetry.addData("Intake State", robot.intake.getState());
        telemetry.addData("Intake RPM", robot.intake.getRPM());
        telemetry.addData("Intake Power", robot.intake.getPower());
        telemetry.update();
    }
}
