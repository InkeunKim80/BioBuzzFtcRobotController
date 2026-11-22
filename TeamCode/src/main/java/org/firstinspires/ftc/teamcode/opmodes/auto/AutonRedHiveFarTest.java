
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
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@NextAutonomous(name = "Auton Red Hive Far Test", group = "Examples")
public class AutonRedHiveFarTest extends NextOpMode {

    private static final int POLLEN_COUNT = 4;

    private static final long FIRST_GATE_OPEN_DELAY_MS = 1500;
    private static final long LATER_GATE_OPEN_DELAY_MS = 200;
    private static final long SHOT_MS = 1500;

    private static final long TIP_WAIT_AFTER_DETECTION_MS = 2000;
    private static final long TIP_DETECTION_TIMEOUT_MS = 5000;

    private static final int LIMELIGHT_PIPELINE = 0;
    private static final int FIRST_TAG_ID = 30;
    private static final int LAST_TAG_ID = 33;

    private static final double FORCE_PARK_TIME_SEC = 25.0;
    private static final long LED_ON_AFTER_PARK_MS = 3000;

    private final Robot robot;
    private final Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private Limelight3A limelight;
    private HiveAprilTagDetector detector;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime tipWaitTimer = new ElapsedTime();

    private Command activeRoutine;

    private boolean partnerTipDetected = false;
    private boolean tipDetectionTimedOut = false;
    private boolean forceParking = false;
    private boolean flowerReady = false;
    private boolean parkingStarted = false;

    // =========================================================
    // Poses
    // =========================================================

    private final Pose startPose =
            poseFactory.of(60, 135, 270);

    private final Pose flowerPose =
            poseFactory.of(48, 128, 90);

    private final Pose flowerControlPose =
            poseFactory.of(62, 123, 0);

    private final Pose parkControlPose =
            poseFactory.of(54, 103, 0);

    private final Pose endPose =
            poseFactory.of(15, 120, 270);

    public AutonRedHiveFarTest(Robot robot) {
        super(robot);
        this.robot = robot;

        follower = Constants.create(hardwareMap)
                .withLogger(log -> telemetry.addData("Follower", log.toString()));
        follower.setPose(startPose);
        follower.update();
    }

    // =========================================================
    // Paths
    // =========================================================

    private Path flowerPath() {
        return curve(
                startPose,
                flowerControlPose,
                flowerPose
        ).linear(startPose, flowerPose);
    }

    private Path returnToStartPath() {
        return curve(
                flowerPose,
                flowerControlPose,
                startPose
        ).linear(flowerPose, startPose);
    }

    private Path parkPath() {
        return curve(
                startPose,
                parkControlPose,
                endPose
        ).linear(startPose, endPose);
    }

    private Path emergencyParkPath() {
        Pose currentPose = follower.pose();

        return curve(
                currentPose,
                parkControlPose,
                endPose
        ).linear(currentPose, endPose);
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
    // Partner TIP Detection
    // =========================================================

    private void updatePartnerTipDetection() {

        if (partnerTipDetected || tipDetectionTimedOut) {
            return;
        }

        if (tipWaitTimer.milliseconds()
                >= TIP_DETECTION_TIMEOUT_MS) {
            tipDetectionTimedOut = true;
            return;
        }

        HiveAprilTagDetector.State state = detector.update();

        if (state == HiveAprilTagDetector.State.VISIBLE) {
            partnerTipDetected = true;
            return;
        }

        if (state == HiveAprilTagDetector.State.NOT_VISIBLE
                || state == HiveAprilTagDetector.State.UNKNOWN) {
            detector.startCheck();
        }
    }

    private Command waitForPartnerTip() {
        return sequential(
                instant(() -> {
                    partnerTipDetected = false;
                    tipDetectionTimedOut = false;
                    tipWaitTimer.reset();
                    detector.startCheck();
                }),

                waitUntil(() -> {
                    updatePartnerTipDetection();
                    return partnerTipDetected
                            || tipDetectionTimedOut;
                })
        );
    }

    // =========================================================
    // Normal Autonomous
    // =========================================================

    private Command shootingAndCollectionRoutine() {
        return sequential(
                waitMs(TIP_WAIT_AFTER_DETECTION_MS),

                robot.shooterDual.reverse(),
                robot.intakeServo.forward(),
                robot.intake.reverse(),

                shoot(FIRST_GATE_OPEN_DELAY_MS),

                robot.flower.positionZero(),
                follow(follower, flowerPath()),
                robot.flower.positionOne(),
                follow(follower, returnToStartPath()),

                shoot(LATER_GATE_OPEN_DELAY_MS)
        );
    }

    private Command autoRoutine() {
        return sequential(
                robot.flower.positionOne(),
                waitForPartnerTip(),

                conditional(
                        () -> partnerTipDetected,
                        shootingAndCollectionRoutine(),
                        Command.NOOP
                ),

                instant(() -> parkingStarted = true),

                follow(follower, parkPath()),

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

        // Interrupt AprilTag wait, shooting, or path following.
        if (activeRoutine != null) {
            activeRoutine.cancel();
        }

        // Recalculate from current measured pose.
        Path emergencyPath = emergencyParkPath();

        activeRoutine = sequential(
                robot.flower.positionOne(),
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
        if (!flowerReady) {
            robot.flower.setServoAtOne(hardwareMap.get(Servo.class, "flowerServo"));
            flowerReady = true;
        }

        telemetry.addLine("=== PLAN B: DOWN HIVE ===");
        telemetry.addData("Start", "(60, 135, 270 deg)");
        telemetry.addData("Flower", "(48, 128, 90 deg)");
        telemetry.addData("Flower Control", "(62, 123)");
        telemetry.addData("Park Control", "(54, 103)");
        telemetry.addData("Park", "(15, 120, 270 deg)");
        telemetry.addData("AprilTag IDs", "30 - 33");
        telemetry.addData("Forced Parking", "25.0 sec");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();

        autoTimer.reset();

        partnerTipDetected = false;
        tipDetectionTimedOut = false;
        forceParking = false;
        parkingStarted = false;

        robot.gate.setServo(hardwareMap.get(Servo.class, "gateServo"));
        robot.intakeServo.setServo(hardwareMap.get(CRServo.class, "intakeServo"));
        robot.indicator.setServo(hardwareMap.get(Servo.class, "rgbLight"));

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

        telemetry.addLine("=== PLAN B: DOWN HIVE ===");
        telemetry.addData(
                "Elapsed", "%.2f sec", autoTimer.seconds()
        );
        telemetry.addData(
                "Remaining", "%.2f sec",
                Math.max(0.0, 30.0 - autoTimer.seconds())
        );

        telemetry.addData("X", "%.2f", currentPose.x());
        telemetry.addData("Y", "%.2f", currentPose.y());
        telemetry.addData(
                "Heading", "%.1f deg",
                Math.toDegrees(currentPose.heading())
        );

        telemetry.addData("Partner TIP", partnerTipDetected);
        telemetry.addData("TIP Timeout", tipDetectionTimedOut);
        telemetry.addData("Force Parking", forceParking);
        telemetry.addData("Parking Started", parkingStarted);

        if (detector != null) {
            telemetry.addData(
                    "AprilTag State", detector.getState()
            );
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
