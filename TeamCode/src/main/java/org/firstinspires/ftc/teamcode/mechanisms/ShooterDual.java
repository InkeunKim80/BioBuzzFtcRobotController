package org.firstinspires.ftc.teamcode.mechanisms;

import static dev.nextftc.units.Units.Degrees;
import static dev.nextftc.units.Units.RotationsPerMinute;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.MotorConstants;

public class ShooterDual implements Mechanism {
    private final NextMotor shootMotor1 = new NextMotor(
            "shootMotor1",
            Degrees.of(360.0 / MotorConstants.GoBILDA.RPM_6000_COUNTS_PER_REV)
    );
    private ShooterState shooterState = ShooterState.OFF;
    private double powerMultiplier = 0.55;

    private boolean rpmControl = true;
    private double targetRpm = 2400;
    private double integralError = 0;

    public enum ShooterState {
        FORWARD,
        REVERSE,
        OFF
    }

    private static final double FORWARD_POWER = 1.0;
    private static final double REVERSE_POWER = -1.0;
    private static final double OFF_POWER = 0.0;

    // Feedforward: about 0.55 power at 2400 RPM
    private static final double KFF = 0.55 / 2400.0;
    private double kp = 0.0005;
    private double ki = 0.0001;
    private static final double MAX_INTEGRAL = 3000.0;

    public ShooterDual() {
        shootMotor1.setDirection(NextMotor.Direction.REVERSE);
    }

    public Command setState(ShooterState shooterState) {
        return instant(() -> {
            this.shooterState = shooterState;
            this.integralError = 0;
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
        return shootMotor1.getEncoderVelocity().into(RotationsPerMinute);
    }

    public double getPower1(){
        return shootMotor1.getThrottle();
    }

    public void setPowerMultiplier(double value) {
        this.powerMultiplier = Math.max(0.1, Math.min(1.0, value));
    }

    public double getPowerMultiplier() {
        return powerMultiplier;
    }

    public void enableRpmControl() {
        this.rpmControl = true;
        this.integralError = 0;
    }

    public void disableRpmControl() {
        this.rpmControl = false;
    }

    public boolean isRpmControlEnabled() {
        return rpmControl;
    }

    public void setTargetRpm(double rpm) {
        this.targetRpm = Math.abs(rpm);
        this.integralError = 0;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public void setKp(double value) {
        this.kp = value;
        this.integralError = 0;
    }

    public double getKp() {
        return kp;
    }

    public void setKi(double value) {
        this.ki = value;
        this.integralError = 0;
    }

    public double getKi() {
        return ki;
    }

    public Command increasePower() {
        return instant(() -> setPowerMultiplier(getPowerMultiplier() + 0.01));
    }

    public Command decreasePower() {
        return instant(() -> setPowerMultiplier(getPowerMultiplier() - 0.01));
    }

    public Command adjustLeftBumper() {
        return instant(() -> setPowerMultiplier(getPowerMultiplier() - 0.01));
    }

    public Command adjustRightBumper() {
        return instant(() -> setPowerMultiplier(getPowerMultiplier() + 0.01));
    }

    private double rpmThrottle() {
        double error = targetRpm - Math.abs(getRPM1());
        integralError = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integralError + error));
        double throttle = KFF * targetRpm + kp * error + ki * integralError;
        return Math.max(0.0, Math.min(1.0, throttle));
    }

    @Override
    public void periodic() {
        switch (shooterState) {
            case FORWARD:
                shootMotor1.setThrottle(rpmControl ? rpmThrottle() : FORWARD_POWER * powerMultiplier);
                break;
            case REVERSE:
                shootMotor1.setThrottle(rpmControl ? -rpmThrottle() : REVERSE_POWER * powerMultiplier);
                break;
            case OFF:
                shootMotor1.setThrottle(OFF_POWER);
                break;
        }
    }
}
