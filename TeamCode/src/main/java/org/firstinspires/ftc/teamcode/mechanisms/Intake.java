package org.firstinspires.ftc.teamcode.mechanisms;

import static dev.nextftc.units.Units.Degrees;
import static dev.nextftc.units.Units.RotationsPerMinute;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.hardware.MotorConstants;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {
    private final NextMotor intakeMotor = new NextMotor(
            "intakeMotor",
            Degrees.of(360.0 / MotorConstants.GoBILDA.RPM_435_COUNTS_PER_REV)
    );
    private IntakeState intakeState = IntakeState.OFF;
    public enum IntakeState {
        FORWARD,
        REVERSE,
        OFF
    }

    private static final double FORWARD_POWER = 1.0;
    private static final double REVERSE_POWER = -1.0;
    private static final double OFF_POWER = 0.0;

    public Intake() {
        intakeMotor.setDirection(NextMotor.Direction.REVERSE);
    }

    public Command setState(IntakeState intakeState) {
        return instant(() -> this.intakeState = intakeState);
    }

    public Command reverse() {
        return setState(IntakeState.REVERSE);
    }

    public Command forward() {
        return setState(IntakeState.FORWARD);
    }

    public Command off() {
        return setState(IntakeState.OFF);
    }

    public Command cycle(){
        return instant(() -> {
            switch (intakeState) {
                case FORWARD:
                case REVERSE:
                    intakeState = IntakeState.OFF;
                    break;
                case OFF:
                    intakeState = IntakeState.FORWARD;
                    break;
            }
        });
    }

    public IntakeState getState() {
        return intakeState;
    }

    public double getRPM(){
        return intakeMotor.getEncoderVelocity().into(RotationsPerMinute);
    }

    public double getPower(){
        return intakeMotor.getThrottle();
    }

    @Override
    public void periodic() {
        switch (intakeState) {
            case FORWARD:
                intakeMotor.setThrottle(FORWARD_POWER);
                break;
            case REVERSE:
                intakeMotor.setThrottle(REVERSE_POWER);
                break;
            case OFF:
                intakeMotor.setThrottle(OFF_POWER);
                break;
        }
    }
}