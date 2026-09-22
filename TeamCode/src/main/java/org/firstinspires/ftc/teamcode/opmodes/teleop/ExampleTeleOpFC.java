package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

@Disabled
@NextTeleop(name="Example TeleOp FC", group = "Examples")
public class ExampleTeleOpFC extends NextOpMode {
    private final Robot robot;
    private Follower follower;

    public ExampleTeleOpFC(Robot robot) {
        super(robot);
        this.robot = robot;

        Scheduler.reset();
    }

    // Called repeatedly while the OpMode is in the INIT phase.
    @Override
    public void disabledPeriodic() {
        if (follower == null) {
            follower = Constants.create(hardwareMap)
                    .withLogger(log -> telemetry.addData("Follower", log.toString()));

            // Set the robot's actual starting pose here.
            // follower.setPose(startPose);

            follower.update();
        }
    }

    // Called exactly once after the PLAY button is pressed.
    @Override
    public void start() {
        robot.drivetrain.startDrive(gamepad1);

        robot.drivetrain.startDriveFieldCentric(
                gamepad1,
                () -> follower.pose().heading()
        );
    }

    // Called repeatedly while the OpMode is actively running.
    @Override
    public void periodic() {
        telemetry.update();
    }

    // Called exactly once when the OpMode finishes execution.
    @Override
    public void end() {
    }

}