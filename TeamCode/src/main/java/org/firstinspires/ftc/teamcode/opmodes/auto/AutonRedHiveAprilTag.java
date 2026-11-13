package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.IndicatorLight;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.HiveAprilTagDetector;

import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@NextAutonomous(
        name = "Auton Red Hive AprilTag",
        group = "Competition"
)
public class AutonRedHiveAprilTag extends NextOpMode {

    // =====================================================
    // Configuration
    // =====================================================

    private static final int PRELOAD_COUNT = 4;
    private static final int PICKUP_SHOT_COUNT = 4;

    private static final long GATE_OPEN_DELAY_MS = 1000;
    private static final long SHOT_MS = 600;
    private static final long PICKUP_MS = 2000;

    private static final long LED_ON_AFTER_PARK_MS = 3000;

    // Force parking after 25 seconds.
    private static final long AUTO_PARK_TIMEOUT_MS = 25000;

    private static final int LIMELIGHT_PIPELINE = 0;

    // RED SCORING CELL: 30-33
    // RED AUDIENCE CELL: 34-37
    private static final int TARGET_TAG_MIN = 30;
    private static final int TARGET_TAG_MAX = 33;

    // =====================================================
    // Robot / Pedro / Vision
    // =====================================================

    private final Robot robot;
    private final Follower follower;

    private Limelight3A limelight;
    private HiveAprilTagDetector hiveDetector;

    private final PoseFactory poseFactory =
            PoseFactory.degrees();

    // =====================================================
    // Existing Poses
    // =====================================================

    private final Pose startPose =
            poseFactory.of(59, 8, 90);

    private final Pose shootPose =
            poseFactory.of(59, 11, 90);

    private final Pose controlPose =
            poseFactory.of(13, 54, 0);

    private final Pose defaultEndPose =
            poseFactory.of(15, 120, 270);

    private final Pose alternateEndPose =
            poseFactory.of(13, 90, 270);

    private Pose endPose = defaultEndPose;

    // =====================================================
    // New Poses - MUST BE CONFIGURED
    // =====================================================

    // Replace with actual field coordinates.
    // -1 means not configured.

    private static final double FLOWER_X = 15;
    private static final double FLOWER_Y = 47;
    private static final double FLOWER_HEADING = 90;

    private static final double OPPOSITE_X = 32;
    private static final double OPPOSITE_Y = 120;
    private static final double OPPOSITE_HEADING = 310;

    private final Pose flowerPose = poseFactory.of(
            FLOWER_X,
            FLOWER_Y,
            FLOWER_HEADING
    );

    private final Pose oppositeShootPose = poseFactory.of(
            OPPOSITE_X,
            OPPOSITE_Y,
            OPPOSITE_HEADING
    );

    // =====================================================
    // Autonomous State
    // =====================================================

    private String autoStep = "INIT";
    private String selectedShoot = "NONE";

    private boolean limelightReady = false;
    private boolean autoConfigured = false;

    // =====================================================
    // 25-Second Timeout State
    // =====================================================

    private long autoStartTime = 0;

    private boolean forcedParking = false;
    private boolean routineFinished = false;

    private Command mainAutoCommand;

    // =====================================================
    // Constructor
    // =====================================================

    public AutonRedHiveAprilTag(Robot robot) {
        super(robot);

        this.robot = robot;

        follower = Constants.create(hardwareMap)
                .withLogger(log ->
                        telemetry.addData(
                                "Follower",
                                log.toString()
                        )
                );

        follower.setPose(startPose);
        follower.update();
    }

    // =====================================================
    // Path Definitions
    // =====================================================

    private Path pathToInitialShoot() {
        return line(startPose, shootPose)
                .linear(startPose, shootPose);
    }

    private Path pathToFlower() {
        return line(shootPose, flowerPose)
                .linear(shootPose, flowerPose);
    }

    private Path pathToShootA() {
        return line(flowerPose, shootPose)
                .linear(flowerPose, shootPose);
    }

    private Path pathToShootB() {
        return line(shootPose, oppositeShootPose)
                .linear(shootPose, oppositeShootPose);
    }

    private Path pathToParkFromA() {
        return line(shootPose, endPose)
                .linear(shootPose, endPose);
    }

    private Path pathToParkFromB() {
        return line(oppositeShootPose, endPose)
                .linear(oppositeShootPose, endPose);
    }

    // =====================================================
    // Emergency Parking Path
    // =====================================================

    private Path pathToParkFromCurrent() {

        // Capture the robot's current estimated pose.
        Pose currentPose = follower.pose();

        return line(currentPose, endPose)
                .linear(currentPose, endPose);
    }

    // =====================================================
    // Shooting Sequence
    // =====================================================

    private Command shoot(int pollenCount) {
        return sequential(

                robot.indicator.set(IndicatorLight.YELLOW),

                robot.shooterDual.reverse(),
                robot.intakeServo.forward(),
                robot.intake.reverse(),

                waitMs(GATE_OPEN_DELAY_MS),

                robot.gate.run(),

                waitMs(SHOT_MS * pollenCount),

                robot.intakeServo.off(),
                robot.intake.off(),
                robot.gate.stop(),
                robot.shooterDual.off(),

                robot.indicator.set(IndicatorLight.OFF)
        );
    }

    // =====================================================
    // FLOWER Pickup
    // =====================================================

    private Command pickup() {
        return sequential(

                instant(() ->
                        autoStep = "PICKUP"
                ),

                robot.indicator.set(IndicatorLight.YELLOW),

                robot.gate.stop(),

                robot.intakeServo.forward(),
                robot.intake.reverse(),

                waitMs(PICKUP_MS),

                robot.intake.off(),
                robot.intakeServo.off(),

                robot.indicator.set(IndicatorLight.OFF)
        );
    }

    // =====================================================
    // AprilTag Detection
    // =====================================================

    private Command detectHive() {
        return sequential(

                instant(() -> {
                    autoStep = "DETECT_HIVE";
                    hiveDetector.startCheck();
                }),

                waitUntil(() ->
                        hiveDetector.getState()
                                != HiveAprilTagDetector.State.CHECKING
                )
        );
    }

    // =====================================================
    // Shoot From Pose A
    // =====================================================

    private Command shootFromA() {
        return sequential(

                instant(() -> {
                    autoStep = "SHOOT_A";
                    selectedShoot = "A";
                }),

                shoot(PICKUP_SHOT_COUNT),

                instant(() ->
                        autoStep = "PARK_FROM_A"
                ),

                follow(follower, pathToParkFromA())
        );
    }

    // =====================================================
    // Shoot From Pose B
    // =====================================================

    private Command shootFromB() {
        return sequential(

                instant(() -> {
                    autoStep = "MOVE_TO_SHOOT_B";
                    selectedShoot = "B";
                }),

                follow(follower, pathToShootB()),

                instant(() ->
                        autoStep = "SHOOT_B"
                ),

                shoot(PICKUP_SHOT_COUNT),

                instant(() ->
                        autoStep = "PARK_FROM_B"
                ),

                follow(follower, pathToParkFromB())
        );
    }

    // =====================================================
    // Vision UNKNOWN Fallback
    // =====================================================

    private Command unknownFallback() {
        return sequential(

                instant(() -> {
                    autoStep = "VISION_UNKNOWN_PARK";
                    selectedShoot = "NONE";
                }),

                follow(follower, pathToParkFromA())
        );
    }

    // =====================================================
    // Conditional Shooting Decision
    // =====================================================

    private Command selectShootingPath() {

        return conditional(

                () -> hiveDetector.getState()
                        == HiveAprilTagDetector.State.VISIBLE,

                // Target Tag detected:
                // Shoot from Pose A.
                shootFromA(),

                conditional(

                        () -> hiveDetector.getState()
                                == HiveAprilTagDetector.State.NOT_VISIBLE,

                        // Target Tag not detected:
                        // Move to opposite shooting position.
                        shootFromB(),

                        // Vision result unknown:
                        // Skip second shooting and park.
                        unknownFallback()
                )
        );
    }

    // =====================================================
    // Main Autonomous Routine
    // =====================================================

    private Command autoRoutine() {
        return sequential(

                // -----------------------------------------
                // Step 1: Initial shooting
                // -----------------------------------------

                instant(() ->
                        autoStep = "INITIAL_MOVE"
                ),

                follow(follower, pathToInitialShoot()),

                instant(() ->
                        autoStep = "PRELOAD_SHOOT"
                ),

                shoot(PRELOAD_COUNT),

                // -----------------------------------------
                // Step 2: FLOWER pickup
                // -----------------------------------------

                instant(() ->
                        autoStep = "MOVE_TO_FLOWER"
                ),

                follow(follower, pathToFlower()),

                pickup(),

                // -----------------------------------------
                // Step 3: Return to Shoot Pose A
                // -----------------------------------------

                instant(() ->
                        autoStep = "MOVE_TO_SHOOT_A"
                ),

                follow(follower, pathToShootA()),

                // -----------------------------------------
                // Step 4: AprilTag detection
                // -----------------------------------------

                detectHive(),

                // -----------------------------------------
                // Step 5: Conditional shooting + parking
                // -----------------------------------------

                selectShootingPath(),

                // -----------------------------------------
                // Step 6: Finish
                // -----------------------------------------

                instant(() ->
                        autoStep = "PARKED"
                ),

                robot.indicator.set(IndicatorLight.GREEN),

                waitMs(LED_ON_AFTER_PARK_MS),

                robot.indicator.set(IndicatorLight.OFF),

                instant(() -> {
                    autoStep = "DONE";
                    routineFinished = true;
                })
        );
    }

    // =====================================================
    // Forced Parking After 25 Seconds
    // =====================================================

    private void forceParking() {

        // Prevent duplicate timeout handling.
        if (forcedParking || routineFinished) {
            return;
        }

        forcedParking = true;

        autoStep = "FORCED_PARKING";
        selectedShoot = "TIMEOUT";

        // -----------------------------------------
        // 1. Cancel main autonomous sequence
        // -----------------------------------------

        if (mainAutoCommand != null) {
            mainAutoCommand.cancel();
        }

        // -----------------------------------------
        // 2. Stop mechanisms
        // -----------------------------------------

        // Mechanism methods return Ivy Commands.
        // Schedule the stop commands together.
        schedule(
                sequential(
                        robot.intake.off(),
                        robot.intakeServo.off(),
                        robot.gate.stop(),
                        robot.shooterDual.off()
                )
        );

        // -----------------------------------------
        // 3. Build path from CURRENT pose
        // -----------------------------------------

        Path emergencyParkPath =
                pathToParkFromCurrent();

        // -----------------------------------------
        // 4. Execute emergency parking
        // -----------------------------------------

        schedule(
                sequential(

                        follow(follower, emergencyParkPath),

                        instant(() -> {
                            autoStep = "FORCED_PARK_COMPLETE";
                            routineFinished = true;
                        }),

                        robot.indicator.set(IndicatorLight.GREEN)
                )
        );
    }

    // =====================================================
    // Disabled Periodic
    // =====================================================

    @Override
    public void disabledPeriodic() {

        if (gamepad1.right_bumper) {

            endPose = alternateEndPose;

        } else if (gamepad1.left_bumper) {

            endPose = defaultEndPose;
        }

        autoConfigured =
                FLOWER_X >= 0 &&
                        FLOWER_Y >= 0 &&
                        OPPOSITE_X >= 0 &&
                        OPPOSITE_Y >= 0;

        telemetry.addData(
                "End Pose",
                endPose == alternateEndPose
                        ? "Alternate (13, 90, 270)"
                        : "Default (15, 120, 270)"
        );

        telemetry.addData(
                "Select",
                "LB: default, RB: alternate"
        );

        telemetry.addData(
                "AprilTag Target",
                "%d - %d",
                TARGET_TAG_MIN,
                TARGET_TAG_MAX
        );

        telemetry.addData(
                "Path Configured",
                autoConfigured
        );

        telemetry.addData(
                "Force Park Timeout",
                "%.1f sec",
                AUTO_PARK_TIMEOUT_MS / 1000.0
        );

        if (!autoConfigured) {
            telemetry.addLine(
                    "WARNING: Configure FLOWER and OPPOSITE poses"
            );
        }

        telemetry.update();
    }

    // =====================================================
    // Start
    // =====================================================

    @Override
    public void start() {

        // -----------------------------------------
        // Initialize mechanisms
        // -----------------------------------------

        robot.gate.setServo(
                hardwareMap.get(
                        CRServo.class,
                        "gateServo"
                )
        );

        robot.intakeServo.setServo(
                hardwareMap.get(
                        CRServo.class,
                        "intakeServo"
                )
        );

        robot.indicator.setServo(
                hardwareMap.get(
                        Servo.class,
                        "rgbLight"
                )
        );

        // -----------------------------------------
        // Check path configuration
        // -----------------------------------------

        autoConfigured =
                FLOWER_X >= 0 &&
                        FLOWER_Y >= 0 &&
                        OPPOSITE_X >= 0 &&
                        OPPOSITE_Y >= 0;

        if (!autoConfigured) {

            autoStep = "INVALID_POSE_CONFIG";

            schedule(
                    robot.indicator.set(
                            IndicatorLight.YELLOW
                    )
            );

            return;
        }

        // -----------------------------------------
        // Initialize Limelight
        // -----------------------------------------

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.setPollRateHz(50);

        limelight.pipelineSwitch(
                LIMELIGHT_PIPELINE
        );

        limelight.start();

        limelightReady = limelight.isConnected();

        hiveDetector = new HiveAprilTagDetector(
                limelight,
                TARGET_TAG_MIN,
                TARGET_TAG_MAX
        );

        if (!limelightReady) {

            autoStep = "LIMELIGHT_NOT_CONNECTED";

            schedule(
                    robot.indicator.set(
                            IndicatorLight.YELLOW
                    )
            );

            return;
        }

        // -----------------------------------------
        // Reset autonomous state
        // -----------------------------------------

        forcedParking = false;
        routineFinished = false;

        autoStep = "STARTED";
        selectedShoot = "NONE";

        // -----------------------------------------
        // Start 25-second timer
        // -----------------------------------------

        autoStartTime =
                System.currentTimeMillis();

        // -----------------------------------------
        // Schedule autonomous routine
        // -----------------------------------------

        mainAutoCommand = autoRoutine();

        schedule(mainAutoCommand);
    }

    // =====================================================
    // Periodic
    // =====================================================

    @Override
    public void periodic() {

        // -----------------------------------------
        // 1. Check autonomous timeout
        // -----------------------------------------

        long elapsedMs = 0;

        if (autoStartTime > 0) {

            elapsedMs =
                    System.currentTimeMillis()
                            - autoStartTime;

            if (!forcedParking &&
                    !routineFinished &&
                    elapsedMs >= AUTO_PARK_TIMEOUT_MS) {

                forceParking();
            }
        }

        // -----------------------------------------
        // 2. Update Pedro follower
        // -----------------------------------------

        follower.update();

        // -----------------------------------------
        // 3. Update AprilTag detector
        // -----------------------------------------

        if (hiveDetector != null &&
                !forcedParking) {

            hiveDetector.update();
        }

        // -----------------------------------------
        // 4. Telemetry
        // -----------------------------------------

        telemetry.addData(
                "Auto Time",
                "%.2f sec",
                elapsedMs / 1000.0
        );

        telemetry.addData(
                "Timeout",
                "%.1f sec",
                AUTO_PARK_TIMEOUT_MS / 1000.0
        );

        telemetry.addData(
                "Forced Parking",
                forcedParking
        );

        telemetry.addData(
                "Routine Finished",
                routineFinished
        );

        telemetry.addData(
                "Auto Step",
                autoStep
        );

        telemetry.addData(
                "Shoot Position",
                selectedShoot
        );

        telemetry.addData(
                "X",
                "%.2f",
                follower.pose().x()
        );

        telemetry.addData(
                "Y",
                "%.2f",
                follower.pose().y()
        );

        telemetry.addData(
                "Heading",
                "%.1f",
                Math.toDegrees(
                        follower.pose().heading()
                )
        );

        telemetry.addData(
                "Follower Mode",
                follower.mode()
        );

        telemetry.addData(
                "Limelight Ready",
                limelightReady
        );

        if (hiveDetector != null) {

            telemetry.addData(
                    "Hive State",
                    hiveDetector.getState()
            );

            telemetry.addData(
                    "Valid Frames",
                    hiveDetector.getValidFrames()
            );

            telemetry.addData(
                    "Detected Frames",
                    hiveDetector.getDetectedFrames()
            );
        }
    }

    // =====================================================
    // End
    // =====================================================

    @Override
    public void end() {

        // Stop mechanisms.
        // These are Ivy Commands; whether they can be
        // scheduled during OpMode shutdown depends on
        // the scheduler lifecycle.
        schedule(
                sequential(
                        robot.intake.off(),
                        robot.intakeServo.off(),
                        robot.gate.stop(),
                        robot.shooterDual.off()
                )
        );

        if (limelight != null) {
            limelight.stop();
        }

        PoseStorage.save(
                follower.pose()
        );
    }
}
