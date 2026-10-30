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

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@NextAutonomous(name = "Auton Red Hive Far", group = "Examples")
public class AutonRedHiveFar extends NextOpMode {

    private static final int POLLEN_COUNT = 4;
    private static final long GATE_OPEN_DELAY_MS = 500;
    private static final long SHOT_MS = 600;

    private final Robot robot;
    private final Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Pedro coordinates are in inches.
    private final Pose startPose = poseFactory.of(32, 8, 90);

    private final Pose shootPose = poseFactory.of(38, 23, 45);

    private final Pose endPose = poseFactory.of(15, 120, 270);

    public AutonRedHiveFar(Robot robot) {
        super(robot);
        this.robot = robot;

        follower = Constants.create(hardwareMap)
                .withLogger(log -> telemetry.addData("Follower", log.toString()));

        follower.setPose(startPose);
        follower.update();
    }

    private Path firstPath() {
        return line(startPose, shootPose).linear(startPose, shootPose);
    }

    private Path secondPath() {
        return line(shootPose, endPose).linear(shootPose, endPose);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, firstPath()),
                robot.shooterDual.reverse(),
                robot.intakeServo.forward(),
                robot.intake.reverse(),
                waitMs(GATE_OPEN_DELAY_MS),
                robot.gate.run(),
                waitMs(SHOT_MS * POLLEN_COUNT),
                robot.intakeServo.off(),
                robot.intake.off(),
                robot.gate.stop(),
                robot.shooterDual.off(),
                follow(follower, secondPath())
        );
    }

    @Override
    public void start() {
        robot.gate.setServo(hardwareMap.get(CRServo.class, "gateServo"));
        robot.intakeServo.setServo(hardwareMap.get(CRServo.class, "intakeServo"));
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
