package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.drive.DriveCommands;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name="Example TeleOp Pedro", group = "Examples")
public class ExampleTeleOpPedro extends NextOpMode {
    private final Robot robot;
    private Follower follower;

    public ExampleTeleOpPedro(Robot robot) {
        super(robot);
        this.robot = robot;

        Scheduler.reset();
    }

    // Initialize Pedro Pathing follower during INIT phase
    @Override
    public void disabledPeriodic() {
        if (follower == null) {
            follower = Constants.create(hardwareMap);
            if (follower != null) {
                Pose savedPose = PoseStorage.load();
                if (savedPose != null) {
                    follower.setPose(savedPose);
                }
                follower.update();
                telemetry.addData("Follower Status", "Initialized");
            } else {
                telemetry.addData("Follower Status", "Failed to initialize");
            }
        }
    }

    // Called exactly once after the PLAY button is pressed
    @Override
    public void start() {
        CommandGamepad gp1 = new CommandGamepad(gamepad1);
        CommandGamepad gp2 = new CommandGamepad(gamepad2);

        // Start drive with gamepad1 (field-centric if using Pedro Pathing)
        robot.drivetrain.startDriveFieldCentric(
                gamepad1,
                () -> follower != null ? follower.pose().heading() : 0
        );

        // Power reduction on left bumper
        gp1.leftBumper().whileTrue(
                infinite(() -> DriveCommands.setScalar(0.4))
                        .setEnd(end -> DriveCommands.setScalar(1.0))
        );

        robot.gate.setServo(hardwareMap.get(Servo.class, "gateServo"));

        // Intake and shooter controls on gamepad2 (same buttons drive both)
        gp2.a().onTrue(robot.intake.forward());
        gp2.a().onTrue(robot.shooterDual.forward());
        gp2.b().onTrue(robot.intake.off());
        gp2.b().onTrue(robot.shooterDual.off());
        gp2.x().onTrue(robot.intake.reverse());
        gp2.x().onTrue(robot.shooterDual.reverse());

        // Gate toggle on Y
        gp2.y().onTrue(robot.gate.toggle());
    }

    // Called repeatedly while the OpMode is actively running
    @Override
    public void periodic() {
        if (follower != null) {
            follower.update();

            // Display current pose and sensor data
            telemetry.addData("X Pos", follower.pose().x());
            telemetry.addData("Y Pos", follower.pose().y());
            telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        }

        telemetry.update();
    }

    // Called exactly once when the OpMode finishes execution
}
