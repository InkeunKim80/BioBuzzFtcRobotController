package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name="Scoop Test", group = "Tests")
public class ScoopTest extends NextOpMode {
    private final Robot robot;

    public ScoopTest(Robot robot) {
        super(robot);
        this.robot = robot;

        Scheduler.reset();
    }

    @Override
    public void start() {
        Servo scoopServo = hardwareMap.get(Servo.class, "scoopServo");
        robot.scoop.setScoopServo(scoopServo);

        CommandGamepad gp2 = new CommandGamepad(gamepad2);

        gp2.a().onTrue(robot.scoop.open());
        gp2.b().onTrue(robot.scoop.close());
        gp2.y().onTrue(robot.scoop.toggle());
    }

    @Override
    public void periodic() {
        telemetry.addData("Scoop State", robot.scoop.getState());
        telemetry.addData("Scoop Position", robot.scoop.getPosition());
        telemetry.update();
    }
}
