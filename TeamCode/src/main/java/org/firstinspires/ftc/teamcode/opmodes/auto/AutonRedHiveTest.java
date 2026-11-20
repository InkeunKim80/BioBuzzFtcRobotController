
package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.IndicatorLight;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.HiveAprilTagDetector;

import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@NextAutonomous(name = "Auton Red Hive Test", group = "Examples")
public class AutonRedHiveTest extends NextOpMode {

    private static final int POLLEN_COUNT = 4;

    private static final long FIRST_GATE_OPEN_DELAY_MS = 1500;
    private static final long LATER_GATE_OPEN_DELAY_MS = 200;
    private static final long SHOT_MS = 1500;
    private static final long TIP_SETTLE_MS = 150;

    private static final int LIMELIGHT_PIPELINE = 0;
    private static final int FIRST_TAG_ID = 34;
    private static final int LAST_TAG_ID = 37;

    private static final double FORCE_PARK_TIME_SEC = 25.0;
    private static final long LED_ON_AFTER_PARK_MS = 3000;

    private final Robot robot;
    private final Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private Limelight3A limelight;
    private HiveAprilTagDetector detector;

    private final ElapsedTime autoTimer = new ElapsedTime();

    private Command activeRoutine;
    private boolean forceParking = false;
    private boolean parkingStarted = false;

    private final Pose startPose =
            poseFactory.of(59, 8, 90);

    private final Pose flowerPose =
            poseFactory.of(11.73, 46.62, 180);

    private final Pose parkControlPose =
            poseFactory.of(9, 15, 0);

    private final Pose alternateEndPose =
            poseFactory.of(13, 90, 270);

    public AutonRedHiveTest(Robot robot) {
        super(robot);
        this.robot = robot;

        follower = Constants.create(hardwareMap).withLogger();
        follower.setPose(startPose);
        follower.update();
    }

    // =========================================================
    // Paths
    // =========================================================

    private Path flowerPath() {
        return line(startPose, flowerPose)
                .linear(startPose, flowerPose);
    }

    private Path returnToStartPath() {
        return line(flowerPose, startPose)
                .linear(flowerPose, startPose);
    }

    private Path parkFromStartPath() {
        return curve(
                startPose,
                parkControlPose,
                alternateEndPose
        ).linear(startPose, alternateEndPose);
    }

    // Emergency parking from CURRENT robot pose.
    private Path emergencyParkPath() {
        Pose currentPose = follower.pose();

        return curve(
                currentPose,
                parkControlPose,
                alternateEndPose
        ).linear(currentPose, alternateEndPose);
    }

    // =========================================================
    // Shooting
    // =========================================================

    private Command shoot(long gateOpenDelayMs) {
        return sequential(
                robot.indicator.set(IndicatorLight.YELLOW),
                waitMs(gateOpenDelayMs),
                robot.gate.run(),
                waitMs(SHOT_MS * POLLEN_COUNT),
                robot.gate.stop(),
                robot.indicator.set(IndicatorLight.OFF)
        );
    }

    // =========================================================
    // TIP Detection
    // =========================================================

    private Command checkTip() {
        return sequential(
                waitMs(TIP_SETTLE_MS),

                instant(() -> detector.startCheck()),

                waitUntil(() ->
                        detector.update()
                                != HiveAprilTagDetector.State.CHECKING
                ),

                conditional(
                        () -> detector.getState()
                                == HiveAprilTagDetector.State.UNKNOWN,

                        sequential(
                                instant(() -> detector.startCheck()),
                                waitUntil(() ->
                                        detector.update()
                                                != HiveAprilTagDetector.State.CHECKING
                                )
                        ),

                        Command.NOOP
                )
        );
    }

    private Command shootIfTipFailed() {
        return conditional(
                () -> detector.getState()
                        == HiveAprilTagDetector.State.VISIBLE,

                shoot(LATER_GATE_OPEN_DELAY_MS),

                Command.NOOP
        );
    }

    // =========================================================
    // Normal Autonomous
    // =========================================================

    private Command autoRoutine() {
        return sequential(
                robot.shooterDual.reverse(),
                robot.intakeServo.forward(),
                robot.intake.reverse(),

                shoot(FIRST_GATE_OPEN_DELAY_MS),

                follow(follower, flowerPath()),
                follow(follower, returnToStartPath()),

                checkTip(),
                shootIfTipFailed(),

                instant(() -> parkingStarted = true),

                follow(follower, parkFromStartPath()),

                robot.shooterDual.off(),
                robot.intakeServo.off(),
                robot.intake.off(),
                robot.gate.stop(),

                robot.indicator.set(IndicatorLight.GREEN),
                waitMs(LED_ON_AFTER_PARK_MS),
                robot.indicator.set(IndicatorLight.OFF)
        );
    }

    // =========================================================
    // Forced Parking
    // =========================================================

    private void forceParkNow() {

        if (forceParking || parkingStarted) {
            return;
        }

        forceParking = true;
        parkingStarted = true;

        // Cancel whatever the normal autonomous is doing.
        if (activeRoutine != null) {
            activeRoutine.cancel();
        }

        // Build the path from the actual current pose.
        Path emergencyPath = emergencyParkPath();

        activeRoutine = sequential(

                // Stop all mechanisms before moving.
                robot.gate.stop(),
                robot.shooterDual.off(),
                robot.intakeServo.off(),
                robot.intake.off(),

                robot.indicator.set(IndicatorLight.YELLOW),

                follow(follower, emergencyPath),

                robot.indicator.set(IndicatorLight.GREEN)
        );

        schedule(activeRoutine);
    }

    // =========================================================
    // Lifecycle
    // =========================================================

    @Override
    public void disabledPeriodic() {
        telemetry.addLine("=== PLAN A: UP HIVE ===");
        telemetry.addData("Start", "(59, 8, 90 deg)");
        telemetry.addData("Flower", "(11.73, 46.62, 180 deg)");
        telemetry.addData("Park Control", "(9, 15)");
        telemetry.addData("Park", "(13, 90, 270 deg)");
        telemetry.addData("Forced Parking", "25.0 sec");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();

        autoTimer.reset();
        forceParking = false;
        parkingStarted = false;

        hardwareMap.get(CRServo.class, "gateServo");
        hardwareMap.get(CRServo.class, "intakeServo");
        hardwareMap.get(Servo.class, "rgbLight");

        limelight = hardwareMap.get(
                Limelight3A.class, "limelight"
        );

        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();

        detector = new HiveAprilTagDetector(
                limelight,
                FIRST_TAG_ID,
                LAST_TAG_ID
        );

        activeRoutine = autoRoutine();
        schedule(activeRoutine);
    }

    @Override
    public void periodic() {
        super.periodic();

        follower.update();

        if (autoTimer.seconds() >= FORCE_PARK_TIME_SEC) {
            forceParkNow();
        }

        Pose currentPose = follower.pose();

        telemetry.addLine("=== PLAN A ===");
        telemetry.addData(
                "Elapsed", "%.2f sec", autoTimer.seconds()
        );
        telemetry.addData(
                "Remaining", "%.2f sec",
                Math.max(0.0, 30.0 - autoTimer.seconds())
        );
        telemetry.addData("X", "%.2f", currentPose.getX());
        telemetry.addData("Y", "%.2f", currentPose.getY());
        telemetry.addData(
                "Heading", "%.1f deg",
                Math.toDegrees(currentPose.getHeading())
        );
        telemetry.addData("Force Parking", forceParking);
        telemetry.addData("Parking Started", parkingStarted);

        if (detector != null) {
            telemetry.addData("TIP State", detector.getState());
            telemetry.addData(
                    "Valid Frames", detector.getValidFrames()
            );
            telemetry.addData(
                    "Detected Frames", detector.getDetectedFrames()
            );
        }

        telemetry.update();
    }

    @Override
    public void end() {
        if (limelight != null) {
            limelight.stop();
        }

        PoseStorage.save(follower.pose());
        super.end();
    }
}
