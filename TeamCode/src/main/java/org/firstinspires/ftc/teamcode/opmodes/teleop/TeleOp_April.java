package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.IndicatorLight;
import org.firstinspires.ftc.teamcode.mechanisms.ShooterDual;
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

    private boolean flowerReady = false;
    private boolean yellowHeld = false;
    private double ledColor = IndicatorLight.OFF;

    public TeleOp_April(Robot robot) {
        super(robot);
        this.robot = robot;

        Scheduler.reset();
    }

    @Override
    public void disabledPeriodic() {
        if (!flowerReady) {
            robot.flower.setServo(hardwareMap.get(Servo.class, "flowerServo"));
            flowerReady = true;
        }
    }

    @Override
    public void start() {
        CommandGamepad gp1 = new CommandGamepad(gamepad1);
        CommandGamepad gp2 = new CommandGamepad(gamepad2);

        robot.gate.setServo(hardwareMap.get(Servo.class, "gateServo"));
        // robot.intakeServo.setServo(hardwareMap.get(CRServo.class, "intakeServo"));
        robot.indicator.setServo(hardwareMap.get(Servo.class, "rgbLight"));
        ledColor = IndicatorLight.OFF;

        // Controller 1: drive
        robot.drivetrain.startDrive(gamepad1);

        gp1.leftBumper().whileTrue(
                infinite(() -> DriveCommands.setScalar(0.4))
                        .setEnd(end -> DriveCommands.setScalar(1.0))
        );

        gp1.x().onTrue(robot.flower.down());
        gp1.y().onTrue(robot.flower.up());

        // Shooter target RPM: gamepad1 dpad = motor 1, gamepad2 dpad = motor 2 (up +100, down -100)
        gp1.dpadUp().onTrue(robot.shooterDual.increaseTargetRpm1());
        gp1.dpadDown().onTrue(robot.shooterDual.decreaseTargetRpm1());
        gp2.dpadUp().onTrue(robot.shooterDual.increaseTargetRpm2());
        gp2.dpadDown().onTrue(robot.shooterDual.decreaseTargetRpm2());

        // Controller 2: intake + shooter
        gp2.b().onTrue(off());
        gp2.a().onTrue(shoot());
        gp2.x().onTrue(intakeon());
        gp2.y().onTrue(robot.gate.run());
        gp2.y().onFalse(robot.gate.stop());
        gp2.y().onTrue(instant(() -> { yellowHeld = true; }));
        gp2.y().onFalse(instant(() -> { yellowHeld = false; }));
        gp2.leftBumper().onTrue(reverse());

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();

        detector = new HiveAprilTagDetector(limelight, FIRST_TAG_ID, LAST_TAG_ID);
        detector.startCheck();
    }

    private Command off() {
        return parallel(
                robot.shooterDual.off(),
                robot.intake.off(),
                // robot.intakeServo.off(),
                robot.gate.stop()
        );
    }

    private Command shoot() {
        return sequential(
                robot.shooterDual.reverse(),
                waitMs(500),
                robot.intake.reverse()
                // robot.intakeServo.forward()
        );
    }

    private Command intakeon() {
        return sequential(
                robot.shooterDual.reverse(),
                robot.intake.reverse()
                // robot.intakeServo.forward()
        );
    }

    private Command reverse() {
        return parallel(
                robot.shooterDual.off(),
                robot.intake.forward()
                // robot.intakeServo.reverse()
        );
    }

    @Override
    public void periodic() {
        boolean rpmDrop = robot.shooterDual.getState() != ShooterDual.ShooterState.OFF
                && Math.abs(robot.shooterDual.getRPM1()) < robot.shooterDual.getTargetRpm() - 70;
        double color = yellowHeld ? IndicatorLight.YELLOW : (rpmDrop ? IndicatorLight.RED : IndicatorLight.OFF);
        if (color != ledColor) {
            robot.indicator.setColor(color);
            ledColor = color;
        }

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

        telemetry.addData("Shooter State", robot.shooterDual.getState());
        telemetry.addData("Shooter RPM 1", String.format("%.0f", robot.shooterDual.getRPM1()));
        telemetry.addData("Shooter RPM 2", String.format("%.0f", robot.shooterDual.getRPM2()));
        telemetry.addData("Shooter Target RPM 1", String.format("%.0f", robot.shooterDual.getTargetRpm1()));
        telemetry.addData("Shooter Target RPM 2", String.format("%.0f", robot.shooterDual.getTargetRpm2()));
        telemetry.addData("Shooter Power 1", robot.shooterDual.getPower1());
        telemetry.addData("Shooter Power 2", robot.shooterDual.getPower2());
        telemetry.addData("Intake State", robot.intake.getState());
        telemetry.addData("Intake Power", robot.intake.getPower());
        // telemetry.addData("Intake Servo Power", robot.intakeServo.getPower());
        telemetry.addData("Gate Position", String.format("%.2f", robot.gate.getPosition()));
        telemetry.update();
    }

    @Override
    public void end() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
