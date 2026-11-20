package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.HiveAprilTagDetector;

import dev.nextftc.robot.drive.DriveCommands;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name = "TeleOp April", group = "Tests")
public class TeleOp_April extends NextOpMode {
    private static final int LIMELIGHT_PIPELINE = 0;
    private static final int FIRST_TAG_ID = 34;
    private static final int LAST_TAG_ID = 37;

    private final Robot robot;
    private Limelight3A limelight;
    private HiveAprilTagDetector detector;

    private HiveAprilTagDetector.State lastResult = HiveAprilTagDetector.State.UNKNOWN;
    private int lastDetectedFrames = 0;
    private int lastValidFrames = 0;

    public TeleOp_April(Robot robot) {
        super(robot);
        this.robot = robot;

        Scheduler.reset();
    }

    @Override
    public void start() {
        CommandGamepad gp1 = new CommandGamepad(gamepad1);

        robot.drivetrain.startDrive(gamepad1);

        gp1.leftBumper().whileTrue(
                infinite(() -> DriveCommands.setScalar(0.4))
                        .setEnd(end -> DriveCommands.setScalar(1.0))
        );

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();

        detector = new HiveAprilTagDetector(limelight, FIRST_TAG_ID, LAST_TAG_ID);
        detector.startCheck();
    }

    @Override
    public void periodic() {
        HiveAprilTagDetector.State state = detector.update();
        if (state != HiveAprilTagDetector.State.CHECKING) {
            lastResult = state;
            lastDetectedFrames = detector.getDetectedFrames();
            lastValidFrames = detector.getValidFrames();
            detector.startCheck();
        }

        telemetry.addData("AprilTag Result", lastResult);
        telemetry.addData("Detected / Valid Frames", lastDetectedFrames + " / " + lastValidFrames);

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addData("Limelight", "No valid result");
        } else {
            StringBuilder allIds = new StringBuilder();
            int target = 0;
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                int id = tag.getFiducialId();
                if (allIds.length() > 0) allIds.append(", ");
                allIds.append(id);

                if (id >= FIRST_TAG_ID && id <= LAST_TAG_ID) {
                    target++;
                    telemetry.addData("Tag " + id,
                            "tx %.2f deg, ty %.2f deg", tag.getTargetXDegrees(), tag.getTargetYDegrees());
                }
            }
            telemetry.addData("All Tag IDs", allIds.length() > 0 ? allIds.toString() : "none");
            telemetry.addData("Target Tags (34-37)", target);
        }

        telemetry.update();
    }

    @Override
    public void end() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
