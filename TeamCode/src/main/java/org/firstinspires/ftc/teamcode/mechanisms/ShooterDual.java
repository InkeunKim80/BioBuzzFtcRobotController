package org.firstinspires.ftc.teamcode.mechanisms;

import static dev.nextftc.units.Units.Degrees;
import static dev.nextftc.units.Units.RotationsPerMinute;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class ShooterDual implements Mechanism {
    private final NextMotor shootMotor1 = new NextMotor(
            "shootMotor1",
            Degrees.of(360.0 / 28.0)
    );
    private final NextMotor shootMotor2 = new NextMotor(
            "shootMotor2",
            Degrees.of(360.0 / 28.0)
    );
    private Servo gateServo;
    private ShooterState shooterState = ShooterState.OFF;
    private double powerMultiplier = 1.0;
    private double targetRPM = 1500;
    private double targetRPM1 = 1500;
    private double targetRPM2 = 1500;
    private boolean useRPMControl = false;
    private double prevError = 0;
    private double integralError = 0;
    private static final double KP = 0.001;
    private static final double KI = 0.0002;
    private static final double KD = 0.0;
    private static final double MAX_INTEGRAL = 2.0;

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

    public ShooterDual() {
        shootMotor1.setDirection(NextMotor.Direction.REVERSE);
        shootMotor2.setDirection(NextMotor.Direction.FORWARD);
    }

    public void setGateServo(Servo servo) {
        this.gateServo = servo;
        gateServo.setPosition(GATE_CLOSED);
    }

    public Command setState(ShooterState shooterState) {
        return instant(() -> {
            this.shooterState = shooterState;
            if (shooterState == ShooterState.REVERSE) {
                this.targetRPM = -1500;
                this.targetRPM1 = -1500;
                this.targetRPM2 = 1500;
            } else if (shooterState == ShooterState.FORWARD) {
                this.targetRPM = 1500;
                this.targetRPM1 = 1500;
                this.targetRPM2 = -1500;
            }
            this.integralError = 0;
            this.prevError = 0;
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

    public ShooterState getState() {
        return shooterState;
    }

    public double getRPM1(){
        return Math.abs(shootMotor1.getEncoderVelocity().into(RotationsPerMinute));
    }

    public double getRPM2(){
        return Math.abs(shootMotor2.getEncoderVelocity().into(RotationsPerMinute));
    }

    public double getPower1(){
        return shootMotor1.getThrottle();
    }

    public double getPower2(){
        return shootMotor2.getThrottle();
    }

    public double getGatePosition() {
        return gateServo != null ? gateServo.getPosition() : GATE_CLOSED;
    }

    public void setPowerMultiplier(double value) {
        this.powerMultiplier = Math.max(0.1, Math.min(1.0, value));
    }

    public double getPowerMultiplier() {
        return powerMultiplier;
    }

    public Command increasePower() {
        return instant(() -> setPowerMultiplier(getPowerMultiplier() + 0.05));
    }

    public Command decreasePower() {
        return instant(() -> setPowerMultiplier(getPowerMultiplier() - 0.05));
    }

    public Command increaseRPM() {
        return instant(() -> setTargetRPM(getTargetRPM() + 200));
    }

    public Command decreaseRPM() {
        return instant(() -> setTargetRPM(getTargetRPM() - 200));
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
        if (shooterState == ShooterState.FORWARD) {
            this.targetRPM1 = Math.abs(rpm);
            this.targetRPM2 = -Math.abs(rpm);
        } else if (shooterState == ShooterState.REVERSE) {
            this.targetRPM1 = -Math.abs(rpm);
            this.targetRPM2 = Math.abs(rpm);
        }
        this.useRPMControl = true;
        this.integralError = 0;
        this.prevError = 0;
    }

    public void disableRPMControl() {
        this.useRPMControl = false;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean isRPMControlEnabled() {
        return useRPMControl;
    }

    public Command enableRPMControl() {
        return instant(() -> {
            this.useRPMControl = true;
            if (shooterState == ShooterState.FORWARD) {
                this.targetRPM = 1500;
                this.targetRPM1 = 1500;
                this.targetRPM2 = -1500;
            } else if (shooterState == ShooterState.REVERSE) {
                this.targetRPM = -1500;
                this.targetRPM1 = -1500;
                this.targetRPM2 = 1500;
            }
            this.integralError = 0;
            this.prevError = 0;
        });
    }

    public Command enablePowerControl() {
        return instant(() -> this.useRPMControl = false);
    }

    public Command adjustLeftBumper() {
        return instant(() -> {
            if (useRPMControl) {
                setTargetRPM(getTargetRPM() - 200);
            } else {
                setPowerMultiplier(getPowerMultiplier() - 0.05);
            }
        });
    }

    public Command adjustRightBumper() {
        return instant(() -> {
            if (useRPMControl) {
                setTargetRPM(getTargetRPM() + 200);
            } else {
                setPowerMultiplier(getPowerMultiplier() + 0.05);
            }
        });
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
        if (useRPMControl && shooterState != ShooterState.OFF) {
            double error1 = Math.abs(targetRPM1) - getRPM1();
            double error2 = Math.abs(targetRPM2) - getRPM2();
            double error = (error1 + error2) / 2.0;

            integralError += error;
            integralError = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integralError));

            double derivative = error - prevError;
            prevError = error;

            double pidOutput = KP * error + KI * integralError + KD * derivative;
            double throttle = Math.max(-1.0, Math.min(1.0, pidOutput));

            shootMotor1.setThrottle(throttle);
            shootMotor2.setThrottle(throttle);
        } else {
            switch (shooterState) {
                case FORWARD:
                    shootMotor1.setThrottle(FORWARD_POWER * powerMultiplier);
                    shootMotor2.setThrottle(FORWARD_POWER * powerMultiplier);
                    break;
                case REVERSE:
                    shootMotor1.setThrottle(REVERSE_POWER * powerMultiplier);
                    shootMotor2.setThrottle(REVERSE_POWER * powerMultiplier);
                    break;
                case OFF:
                    shootMotor1.setThrottle(OFF_POWER);
                    shootMotor2.setThrottle(OFF_POWER);
                    break;
            }
        }
    }
}
