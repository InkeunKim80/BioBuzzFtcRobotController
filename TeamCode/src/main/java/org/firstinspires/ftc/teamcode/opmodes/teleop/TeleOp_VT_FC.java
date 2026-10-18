package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.drive.DriveCommands;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@Disabled
@NextTeleop(name = "TeleOp VT FC", group = "Competition")
public class TeleOp_VT_FC extends NextOpMode {
    private final Robot robot;
    private Follower follower;

    public TeleOp_VT_FC(Robot robot) {
        super(robot);
        this.robot = robot;

        Scheduler.reset();
    }

    @Override
    public void disabledPeriodic() {
        if (follower == null) {
            follower = Constants.create(hardwareMap);
            if (follower != null) {
                follower.update();
            }
        }

        telemetry.addData("Follower", follower != null ? "Initialized" : "Not initialized");
        telemetry.update();
    }

    @Override
    public void start() {

        CommandGamepad gp1 = new CommandGamepad(gamepad1);
        CommandGamepad gp2 = new CommandGamepad(gamepad2);

        // Controller 1: field-centric drive + shoot
        robot.drivetrain.startDriveFieldCentric(
                gamepad1,
                () -> follower != null ? follower.pose().heading() : 0.0
        );

        gp1.leftBumper().whileTrue(
                infinite(() -> DriveCommands.setScalar(0.4))
                        .setEnd(end -> DriveCommands.setScalar(1.0))
        );

        gp1.a().onTrue(robot.shooterDual.forward());
        gp1.b().onTrue(robot.shooterDual.off());
        gp1.x().onTrue(robot.shooterDual.reverse());

        gp1.dpadUp().onTrue(robot.shooterDual.increasePower());
        gp1.dpadDown().onTrue(robot.shooterDual.decreasePower());

        // Controller 2: intake + slider
        gp2.a().onTrue(robot.intake.forward());
        gp2.b().onTrue(robot.intake.off());
        gp2.x().onTrue(robot.intake.reverse());

        // Slider placeholder: hardware not yet installed
        // gp2.y().onTrue(robot.slider.extend());
        // gp2.rightBumper().onTrue(robot.slider.returnToStart());
    }

    @Override
    public void periodic() {
        if (follower != null) {
            follower.update();
            telemetry.addData("X", "%.2f", follower.pose().x());
            telemetry.addData("Y", "%.2f", follower.pose().y());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(follower.pose().heading()));
        }

        telemetry.addData("Shooter State", robot.shooterDual.getState());
        telemetry.addData("Shooter RPM 1", robot.shooterDual.getRPM1());
        telemetry.addData("Shooter RPM 2", robot.shooterDual.getRPM2());
        telemetry.addData("Power Multiplier", "%.2f", robot.shooterDual.getPowerMultiplier());
        telemetry.addData("Intake State", robot.intake.getState());
        telemetry.update();
    }
}
