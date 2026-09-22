package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import dev.nextftc.control.drive.MecanumKinematics;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.drive.DriveCommands;
import java.util.function.Supplier;

public class Drivetrain implements Mechanism {

    // Set names for drive motors here
    // Only change the name in "" to match the configuration on the driver hub
    public final NextMotor frontLeft = new NextMotor("frontLeft");
    public final NextMotor frontRight = new NextMotor("frontRight");
    public final NextMotor backLeft = new NextMotor("backLeft");
    public final NextMotor backRight = new NextMotor("backRight");

    // Mecanum kinematics adjustment, default 1.1, if strafe undershoots, increase to 1.2
    private static final double STRAFE_COMPENSATION = 1.1;

    public Drivetrain() {
        frontRight.setDirection(NextMotor.Direction.REVERSE);
        backRight.setDirection(NextMotor.Direction.REVERSE);
    }

    public void startDrive(Gamepad gamepad) {
        DriveCommands.mecanumDrive(frontLeft, frontRight, backLeft, backRight, gamepad, new MecanumKinematics(STRAFE_COMPENSATION)).schedule();
    }

    public void startDriveFieldCentric(Gamepad gamepad, Supplier<Double> heading) {
        DriveCommands.mecanumDriveFieldCentric(frontLeft, frontRight, backLeft, backRight, gamepad, heading, new MecanumKinematics(STRAFE_COMPENSATION)).schedule();
    }
}