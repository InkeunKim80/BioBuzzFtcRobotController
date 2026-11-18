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
    private final NextMotor shootMotor2 = new NextMotor(
            "shootMotor2",
            Degrees.of(360.0 / MotorConstants.GoBILDA.RPM_6000_COUNTS_PER_REV)
    );
    private ShooterState shooterState = ShooterState.OFF;
    private double powerMultiplier = 0.40;

    private boolean rpmControl = true;
    private double targetRpm = 1700;
    private final double[] integralError = new double[2];

    public enum ShooterState {
        FORWARD,
        REVERSE,
        OFF
    }

    private static final double FORWARD_POWER = 1.0;
    private static final double REVERSE_POWER = -1.0;
    private static final double OFF_POWER = 0.0;

    // Feedforward: about 0.55 power at 2400 RPM
    private static final double KFF = 0.5 / 1800.0;
    private double kp = 0.0009;
    private double ki = 0.0001;
    private static final double MAX_INTEGRAL = 3000.0;

    public ShooterDual() {
        shootMotor1.setDirection(NextMotor.Direction.REVERSE);
        shootMotor2.setDirection(NextMotor.Direction.FORWARD);
    }

    private void resetIntegral() {
        integralError[0] = 0;
        integralError[1] = 0;
    }

    public Command setState(ShooterState shooterState) {
        return instant(() -> {
            this.shooterState = shooterState;
            resetIntegral();
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

    public double getRPM2(){
        return shootMotor2.getEncoderVelocity().into(RotationsPerMinute);
    }

    public double getPower1(){
        return shootMotor1.getThrottle();
    }

    public double getPower2(){
        return shootMotor2.getThrottle();
    }

    public void setPowerMultiplier(double value) {
        this.powerMultiplier = Math.max(0.1, Math.min(1.0, value));
    }

    public double getPowerMultiplier() {
        return powerMultiplier;
    }

    public void enableRpmControl() {
        this.rpmControl = true;
        resetIntegral();
    }

    public void disableRpmControl() {
        this.rpmControl = false;
    }

    public boolean isRpmControlEnabled() {
        return rpmControl;
    }

    public void setTargetRpm(double rpm) {
        this.targetRpm = Math.abs(rpm);
        resetIntegral();
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public void setKp(double value) {
        this.kp = value;
        resetIntegral();
    }

    public double getKp() {
        return kp;
    }

    public void setKi(double value) {
        this.ki = value;
        resetIntegral();
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

    private double rpmThrottle(int motorIndex, double currentRpm) {
        double error = targetRpm - Math.abs(currentRpm);
        integralError[motorIndex] = Math.max(-MAX_INTEGRAL,
                Math.min(MAX_INTEGRAL, integralError[motorIndex] + error));
        double throttle = KFF * targetRpm + kp * error + ki * integralError[motorIndex];
        return Math.max(0.0, Math.min(1.0, throttle));
    }

    @Override
    public void periodic() {
        switch (shooterState) {
            case FORWARD:
                if (rpmControl) {
                    shootMotor1.setThrottle(rpmThrottle(0, getRPM1()));
                    shootMotor2.setThrottle(rpmThrottle(1, getRPM2()));
                } else {
                    shootMotor1.setThrottle(FORWARD_POWER * powerMultiplier);
                    shootMotor2.setThrottle(FORWARD_POWER * powerMultiplier);
                }
                break;
            case REVERSE:
                if (rpmControl) {
                    shootMotor1.setThrottle(-rpmThrottle(0, getRPM1()));
                    shootMotor2.setThrottle(-rpmThrottle(1, getRPM2()));
                } else {
                    shootMotor1.setThrottle(REVERSE_POWER * powerMultiplier);
                    shootMotor2.setThrottle(REVERSE_POWER * powerMultiplier);
                }
                break;
            case OFF:
                shootMotor1.setThrottle(OFF_POWER);
                shootMotor2.setThrottle(OFF_POWER);
                break;
        }
    }
}
