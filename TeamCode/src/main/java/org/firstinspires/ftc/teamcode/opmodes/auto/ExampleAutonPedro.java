package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.opmode.NextOpMode;

@Autonomous(name="Example Auton Pedro", group = "Examples")
public class ExampleAutonPedro extends NextOpMode {
    private final Robot robot;
    private Follower follower;
    private PathChain mainPath;

    public ExampleAutonPedro(Robot robot) {
        super(robot);
        this.robot = robot;

        Scheduler.reset();
    }

    // Build example path during INIT phase
    @Override
    public void disabledPeriodic() {
        if (follower == null) {
            // Initialize Pedro Pathing follower
            follower = Constants.create(hardwareMap);

            if (follower != null) {
                // Set starting pose (adjust based on your field positioning)
                // Pose(x, y, heading in radians)
                // follower.setPose(new Pose(0, 0, 0));

                // Build example path chain
                buildPaths();

                follower.update();
                telemetry.addData("Follower Status", "Initialized");
                telemetry.addData("Path Status", "Ready");
            } else {
                telemetry.addData("Follower Status", "Failed to initialize");
            }
        }
    }

    // Called exactly once after the PLAY button is pressed
    @Override
    public void start() {
        // Follow the path chain
        if (follower != null && mainPath != null) {
            follower.followPath(mainPath, true);
        }
    }

    // Called repeatedly while the OpMode is actively running
    @Override
    public void periodic() {
        if (follower != null) {
            follower.update();

            // Display current pose
            telemetry.addData("X Pos", String.format("%.2f", follower.pose().getX()));
            telemetry.addData("Y Pos", String.format("%.2f", follower.pose().getY()));
            telemetry.addData("Heading", String.format("%.2f°", Math.toDegrees(follower.pose().getHeading())));

            // Display path following status
            telemetry.addData("Path Busy", follower.isBusy());

            // Check if path is complete
            if (!follower.isBusy()) {
                telemetry.addData("Status", "Path Complete");
            } else {
                telemetry.addData("Status", "Following Path");
            }
        }

        telemetry.update();
    }

    // Called exactly once when the OpMode finishes execution
    @Override
    public void end() {
        if (follower != null) {
            follower.holdPoint(follower.pose());
        }
    }

    // Build example path
    private void buildPaths() {
        // Example: Move forward 24 inches, then strafe right 12 inches
        Path examplePath = new Path(new BezierLine(
                new Point(0, 0, Point.CARTESIAN),
                new Point(24, 0, Point.CARTESIAN)
        ));
        examplePath.setLinearHeadingInterpolation(0, 0);

        Path strafeRight = new Path(new BezierLine(
                new Point(24, 0, Point.CARTESIAN),
                new Point(24, 12, Point.CARTESIAN)
        ));
        strafeRight.setLinearHeadingInterpolation(0, 0);

        // Chain paths together
        mainPath = follower.pathBuilder()
                .addPath(examplePath)
                .setConstantHeadingInterpolation(0)
                .addPath(strafeRight)
                .setConstantHeadingInterpolation(0)
                .build();
    }
}
