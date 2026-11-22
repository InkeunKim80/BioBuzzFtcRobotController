
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

@NextAutonomous(name = "Auton Blue Hive Test", group = "Examples")
public class AutonBlueHiveTest extends NextOpMode {

    // =========================================================
    // Configuration
    // =========================================================

    private static final int POLLEN_COUNT = 4;

    private static final long FIRST_GATE_OPEN_DELAY_MS = 1500;
    private static final long LATER_GATE_OPEN_DELAY_MS = 200;
    private static final long SHOT_MS = 1500;
    private static final long TIP_SETTLE_MS = 150;

    private static final int LIMELIGHT_PIPELINE = 0;
    private static final int FIRST_TAG_ID = 42;
    private static final int LAST_TAG_ID = 45;

    private static final double FORCE_PARK_TIME_SEC = 25.0;
    private static final long LED_ON_AFTER_PARK_MS = 3000;

    // =========================================================
    // Robot / Follower
    // =========================================================

    private final Robot robot;
    private final Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private Limelight3A limelight;
    private HiveAprilTagDetector detector;

    private final ElapsedTime autoTimer = new ElapsedTime();

    private Command activeRoutine;

    private boolean forceParking = false;
    private boolean flowerReady = false;
    private boolean parkingStarted = false;

    // =========================================================
    // Poses
    // =========================================================

    private final Pose startPose =
            poseFactory.of(84, 134, 270);

    private final Pose flowerPose =
            poseFactory.of(126, 95, 0);

    private final Pose parkControlPose =
            poseFactory.of(129, 102, 0);

    private final Pose endPose =
            poseFactory.of(126, 50, 90);

    // =========================================================
    // Constructor
    // =========================================================

    public AutonBlueHiveTest(Robot robot) {
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
        return line(startPose, flowerPose)
                .linear(startPose, flowerPose);
    }

    private Path returnToStartPath() {
        return line(flowerPose, startPose)
                .linear(flowerPose, startPose);
    }

    private Path parkPath() {
        return curve(
                startPose,
                parkControlPose,
                endPose
        ).linear(startPose, endPose);
    }

    // Forced parking from the current robot pose.
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
                robot.flower.positionOne(),

                // Start shooter and intake.
                robot.shooterDual.reverse(),
                // robot.intakeServo.forward(),
                robot.intake.reverse(),

                // First shooting: 4 preload POLLEN.
                shoot(FIRST_GATE_OPEN_DELAY_MS),

                // Move to FLOWER and collect.
                robot.flower.positionZero(),
                follow(follower, flowerPath()),

                // Return to shooting position.
                robot.flower.positionOne(),
                follow(follower, returnToStartPath()),

                // Check whether HIVE was tipped.
                checkTip(),

                // Shoot again only when TIP failed.
                shootIfTipFailed(),

                // Normal parking.
                instant(() -> parkingStarted = true),

                follow(follower, parkPath()),

                // Stop all mechanisms.
                robot.shooterDual.off(),
                // robot.intakeServo.off(),
                robot.intake.off(),
                robot.gate.stop(),

                // Parking indicator.
                robot.indicator.set(IndicatorLight.GREEN),

                waitMs(LED_ON_AFTER_PARK_MS),

                robot.indicator.set(IndicatorLight.OFF)
        );
    }

    // =========================================================
    // Forced Parking at 25 Seconds
    // =========================================================

    private void forceParkNow() {

        if (forceParking || parkingStarted) {
            return;
        }

        forceParking = true;
        parkingStarted = true;

        // Cancel the currently executing autonomous routine.
        if (activeRoutine != null) {
            activeRoutine.cancel();
        }

        // Recalculate parking path from current position.
        Path emergencyPath = emergencyParkPath();

        activeRoutine = sequential(
                robot.flower.positionOne(),

                // Stop mechanisms immediately.
                robot.gate.stop(),
                robot.shooterDual.off(),
                // robot.intakeServo.off(),
                robot.intake.off(),

                robot.indicator.set(IndicatorLight.YELLOW),

                // Move to parking position.
                follow(follower, emergencyPath),

                robot.indicator.set(IndicatorLight.GREEN)
        );

        schedule(activeRoutine);
    }

    // =========================================================
    // Initialization Telemetry
    // =========================================================

    @Override
    public void disabledPeriodic() {
        if (!flowerReady) {
            robot.flower.setServoAtOne(hardwareMap.get(Servo.class, "flowerServo"));
            flowerReady = true;
        }


        telemetry.addLine("=== BLUE HIVE UP ===");

        telemetry.addData("Start", "(84, 134, 270 deg)");
        telemetry.addData("Flower", "(126, 95, 0 deg)");
        telemetry.addData("Park Control", "(129, 102)");
        telemetry.addData("Park", "(126, 50, 90 deg)");
        telemetry.addData("AprilTag IDs", "42 - 45");
        telemetry.addData("Forced Parking", "25.0 sec");

        telemetry.update();
    }

    // =========================================================
    // Start
    // =========================================================

    @Override
    public void start() {
        super.start();

        autoTimer.reset();

        forceParking = false;
        parkingStarted = false;

        robot.gate.setServo(hardwareMap.get(Servo.class, "gateServo"));
        // robot.intakeServo.setServo(hardwareMap.get(CRServo.class, "intakeServo"));
        robot.indicator.setServo(hardwareMap.get(Servo.class, "rgbLight"));

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
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

    // =========================================================
    // Periodic
    // =========================================================

    @Override
    public void periodic() {
        super.periodic();

        follower.update();

        if (autoTimer.seconds() >= FORCE_PARK_TIME_SEC) {
            forceParkNow();
        }

        Pose currentPose = follower.pose();

        telemetry.addLine("=== BLUE HIVE UP ===");

        telemetry.addData(
                "Elapsed Time",
                "%.2f sec",
                autoTimer.seconds()
        );

        telemetry.addData(
                "Remaining Time",
                "%.2f sec",
                Math.max(0.0, 30.0 - autoTimer.seconds())
        );

        telemetry.addData(
                "Pose X",
                "%.2f",
                currentPose.x()
        );

        telemetry.addData(
                "Pose Y",
                "%.2f",
                currentPose.y()
        );

        telemetry.addData(
                "Heading",
                "%.1f deg",
                Math.toDegrees(currentPose.heading())
        );

        telemetry.addData("Force Parking", forceParking);
        telemetry.addData("Parking Started", parkingStarted);

        if (detector != null) {
            telemetry.addData(
                    "TIP State",
                    detector.getState()
            );

            telemetry.addData(
                    "Valid Frames",
                    detector.getValidFrames()
            );

            telemetry.addData(
                    "Detected Frames",
                    detector.getDetectedFrames()
            );
        }

        telemetry.update();
    }

    // =========================================================
    // End
    // =========================================================

    @Override
    public void end() {

        if (limelight != null) {
            limelight.stop();
        }

        PoseStorage.save(follower.pose());

        super.end();
    }
}
