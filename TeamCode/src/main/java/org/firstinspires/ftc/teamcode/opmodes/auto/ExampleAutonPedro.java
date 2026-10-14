package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.AtomicPath;
import com.pedropathing.paths.CompoundPath;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Line;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.opmode.NextOpMode;

@Autonomous(name = "Example Auton Pedro", group = "Examples")
public class ExampleAutonPedro extends NextOpMode {
    private final Robot robot;
    private Follower follower;
    private Path mainPath;

    public ExampleAutonPedro(Robot robot) {
        super(robot);
        this.robot = robot;

        Scheduler.reset();
    }

    @Override
    public void disabledPeriodic() {
        if (follower == null) {
            follower = Constants.create(hardwareMap);
            if (follower != null) {
                follower.setPose(new Pose(0, 0, 0));
                mainPath = buildPath();
                follower.update();
                telemetry.addData("Follower Status", "Initialized");
            } else {
                telemetry.addData("Follower Status", "Failed to initialize (Constants.create returned null)");
            }
        }
        telemetry.update();
    }

    @Override
    public void start() {
        if (follower != null && mainPath != null) {
            follower.follow(mainPath);
        }
    }

    @Override
    public void periodic() {
        if (follower != null) {
            follower.update();

            Pose pose = follower.pose();
            telemetry.addData("X", String.format("%.2f", pose.x()));
            telemetry.addData("Y", String.format("%.2f", pose.y()));
            telemetry.addData("Heading (deg)", String.format("%.2f", Math.toDegrees(pose.heading())));
            telemetry.addData("Following", follower.isBusy());
        }
        telemetry.update();
    }

    private Path buildPath() {
        Path forward = new AtomicPath(new Line(new Pose(0, 0), new Pose(24, 0))).constant(0);
        Path strafe = new AtomicPath(new Line(new Pose(24, 0), new Pose(24, 12))).constant(0);
        return new CompoundPath(forward, strafe);
    }
}
