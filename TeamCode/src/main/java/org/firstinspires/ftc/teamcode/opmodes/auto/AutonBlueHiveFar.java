package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@NextAutonomous(name = "Auton Blue Hive Far", group = "Examples")
public class AutonBlueHiveFar extends NextOpMode {

    private static final int POLLEN_COUNT = 4;
    private static final long GATE_OPEN_DELAY_MS = 500;
    private static final long SHOT_MS = 600;

    private final Robot robot;
    private final Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Pedro coordinates are in inches.
    private final Pose startPose = poseFactory.of(107, 133, 270);

    private final Pose controlPose = poseFactory.of(129, 102, 0);

    private final Pose endPose = poseFactory.of(129, 16, 90);

    public AutonBlueHiveFar(Robot robot) {
        super(robot);
        this.robot = robot;

        follower = Constants.create(hardwareMap)
                .withLogger(log -> telemetry.addData("Follower", log.toString()));

        follower.setPose(startPose);
        follower.update();
    }

    private Path forwardPath() {
        return curve(startPose, controlPose, endPose).linear(startPose, endPose);
    }

    private Command autoRoutine() {
        return sequential(
                robot.shooterDual.reverse(),
                robot.intake.reverse(),
                waitMs(GATE_OPEN_DELAY_MS),
                robot.gate.open(),
                waitMs(SHOT_MS * POLLEN_COUNT),
                robot.intake.off(),
                robot.gate.close(),
                robot.shooterDual.off(),
                follow(follower, forwardPath())
        );
    }

    @Override
    public void start() {
        robot.gate.setServo(hardwareMap.get(CRServo.class, "gateServo"));
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

    @Override
    public void end() {
        PoseStorage.save(follower.pose());
    }
}
