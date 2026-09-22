package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.drive.DriveCommands;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name="Example TeleOp", group = "Examples")
public class ExampleTeleOp extends NextOpMode {
    private final Robot robot;

    public ExampleTeleOp(Robot robot) {
        super(robot);
        this.robot = robot;

        Scheduler.reset();
    }

    // Called repeatedly while the OpMode is in the INIT phase.
    //@Override
    //public void disabledPeriodic() {  }

    // Called exactly once after the PLAY button is pressed.
    @Override
    public void start() {
        CommandGamepad gp1 = new CommandGamepad(gamepad1);
        CommandGamepad gp2 = new CommandGamepad(gamepad2);

        robot.drivetrain.startDrive(gamepad1);

        gp1.leftBumper().whileTrue(
                infinite(() -> DriveCommands.setScalar(0.4))
                        .setEnd(end -> DriveCommands.setScalar(1.0))
        );

        gp2.a().onTrue(robot.intake.forward());
        gp2.b().onTrue(robot.intake.off());
        gp2.x().onTrue(robot.intake.reverse());
        gp2.y().onTrue(robot.intake.cycle());
    }

    // Called repeatedly while the OpMode is actively running.
    @Override
    public void periodic() {
        telemetry.update();
    }

    // Called exactly once when the OpMode finishes execution.
    //@Override
    //public void end() { }
}
