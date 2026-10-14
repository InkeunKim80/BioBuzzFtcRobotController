package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Disabled
@NextAutonomous(name = "Example Auton Pedro", group = "Examples")
public class ExampleAutonPedro extends NextOpMode {

    private final Robot robot;
    private final Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Pedro coordinates are in inches.
    private final Pose startPose = poseFactory.of(0, 0, 0);

    private final Pose endPose = poseFactory.of(24, 0, 0);

    public ExampleAutonPedro(Robot robot) {
        super(robot);
        this.robot = robot;

        follower = Constants.create(hardwareMap)
                .withLogger(log -> telemetry.addData("Follower", log.toString()));

        follower.setPose(startPose);
        follower.update();
    }

    private Path forwardPath() {
        return line(startPose, endPose).constant(startPose);
    }

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
