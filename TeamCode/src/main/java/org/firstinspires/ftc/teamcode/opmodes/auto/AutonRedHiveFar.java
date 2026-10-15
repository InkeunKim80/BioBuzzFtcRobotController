package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@NextAutonomous(name = "Auton Red Hive Far", group = "Examples")
public class AutonRedHiveFar extends NextOpMode {

    private static final int POLLEN_COUNT = 4;
    private static final long SHOOTER_SPIN_UP_MS = 1500;
    private static final long SHOT_MS = 600;

    private final Robot robot;
    private final Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Pedro coordinates are in inches.
    private final Pose startPose = poseFactory.of(32, 8, 90);

    private final Pose endPose = poseFactory.of(17, 120, 270);

    public AutonRedHiveFar(Robot robot) {
        super(robot);
        this.robot = robot;

        follower = Constants.create(hardwareMap)
                .withLogger(log -> telemetry.addData("Follower", log.toString()));

        follower.setPose(startPose);
        follower.update();
    }

    private Path forwardPath() {
        return line(startPose, endPose).linear(startPose, endPose);
    }

    private Command autoRoutine() {
        return sequential(
                robot.shooterDual.reverse(),
                waitMs(SHOOTER_SPIN_UP_MS),
                robot.intake.reverse(),
                waitMs(SHOT_MS * POLLEN_COUNT),
                robot.intake.off(),
                robot.shooterDual.off(),
                follow(follower, forwardPath())
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
