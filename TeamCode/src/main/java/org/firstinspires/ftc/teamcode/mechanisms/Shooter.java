package org.firstinspires.ftc.teamcode.mechanisms;

import static dev.nextftc.units.Units.Degrees;
import static dev.nextftc.units.Units.RotationsPerMinute;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Shooter implements Mechanism {
    private final NextMotor shootMotor = new NextMotor(
            "shootMotor",
            Degrees.of(360.0 / 28.0)
    );
    private Servo gateServo;
    private ShooterState shooterState = ShooterState.OFF;

    public enum ShooterState {
        FORWARD,
        REVERSE,
        OFF
    }

    private static final double FORWARD_POWER = 1.0;
    private static final double REVERSE_POWER = -1.0;
    private static final double OFF_POWER = 0.0;

    private static final double GATE_OPEN = 1.0;
    private static final double GATE_CLOSED = 0.0;

    public Shooter() {
        shootMotor.setDirection(NextMotor.Direction.REVERSE);
    }

    public void setGateServo(Servo servo) {
        this.gateServo = servo;
        gateServo.setPosition(GATE_CLOSED);
    }

    public Command setState(ShooterState shooterState) {
        return instant(() -> {
            this.shooterState = shooterState;
            updateGatePosition();
        });
    }

    public Command forward() {
        return setState(ShooterState.FORWARD);
    }

    public Command reverse() {
        return setState(ShooterState.REVERSE);
    }

    public Command off() {
        return setState(ShooterState.OFF);
    }

    public Command cycle(){
        return instant(() -> {
            switch (shooterState) {
                case FORWARD:
                case REVERSE:
                    shooterState = ShooterState.OFF;
                    break;
                case OFF:
                    shooterState = ShooterState.FORWARD;
                    break;
            }
            updateGatePosition();
        });
    }

    public ShooterState getState() {
        return shooterState;
    }

    public double getRPM(){
        return shootMotor.getEncoderVelocity().into(RotationsPerMinute);
    }

    public double getPower(){
        return shootMotor.getThrottle();
    }

    public double getGatePosition() {
        return gateServo != null ? gateServo.getPosition() : GATE_CLOSED;
    }

    private void updateGatePosition() {
        if (gateServo != null) {
            if (shooterState == ShooterState.FORWARD) {
                gateServo.setPosition(GATE_OPEN);
            } else {
                gateServo.setPosition(GATE_CLOSED);
            }
        }
    }

    @Override
    public void periodic() {
        switch (shooterState) {
            case FORWARD:
                shootMotor.setThrottle(FORWARD_POWER);
                break;
            case REVERSE:
                shootMotor.setThrottle(REVERSE_POWER);
                break;
            case OFF:
                shootMotor.setThrottle(OFF_POWER);
                break;
        }
    }
}
