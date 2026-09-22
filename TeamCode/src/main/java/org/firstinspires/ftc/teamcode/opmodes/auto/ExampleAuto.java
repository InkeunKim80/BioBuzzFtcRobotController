package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextAutonomous;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Disabled
@NextAutonomous(name = "Example Auto", group = "Examples")
public class ExampleAuto extends NextOpMode {

    private final Robot robot;
    private final Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Pedro coordinates are in inches.
    // see https://pedropathing.com/docs/pathing/guide/pose-creation for pose creation
    private final Pose startPose = poseFactory.of(0, 0, 0);

    private final Pose endPose = poseFactory.of(24, 0, 0);

    public ExampleAuto(Robot robot) {
        super(robot);
        this.robot = robot;

        follower = Constants.create(hardwareMap)
                .withLogger(log -> telemetry.addData("Follower", log.toString()));

        follower.setPose(startPose);
        follower.update();
    }

    // see https://pedropathing.com/docs/pathing/guide/path-creation for path creation
    private Path forwardPath() {
        return line(startPose, endPose).constant(startPose);
    }


    // see https://pedropathing.com/docs/ivy/utilities-and-decorators for additional utility commands
    private Command autoRoutine() {
        return sequential(
                follow(follower, forwardPath()),
                robot.intake.forward(),
                waitMs(2000),
                robot.intake.off()
                );
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void periodic() {
        follower.update();

        telemetry.addData("X", "%.2f", follower.pose().x());
        telemetry.addData("Y", "%.2f", follower.pose().y());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
    }
}