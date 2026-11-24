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

    // Target RPM per motor: index 0 = shootMotor1, index 1 = shootMotor2
    private final double[] targetRpm = {1650, 1650};
    private final double[] integralError = new double[2];

    public enum ShooterState {
        FORWARD,
        REVERSE,
        OFF
    }

    private static final double OFF_POWER = 0.0;

    // Feedforward: about 0.55 power at 2400 RPM
    private static final double FF_BASE_POWER = 0.5;
    private static final double FF_RPM_OFFSET = 100.0;
    private static final double RPM_STEP = 100.0;
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

    // Set both motors to the same target RPM.
    public void setTargetRpm(double rpm) {
        setTargetRpm1(rpm);
        setTargetRpm2(rpm);
    }

    public void setTargetRpm1(double rpm) {
        targetRpm[0] = Math.abs(rpm);
        integralError[0] = 0;
    }

    public void setTargetRpm2(double rpm) {
        targetRpm[1] = Math.abs(rpm);
        integralError[1] = 0;
    }

    // Target RPM of motor 1 (kept for existing callers such as LED / telemetry).
    public double getTargetRpm() {
        return targetRpm[0];
    }

    public double getTargetRpm1() {
        return targetRpm[0];
    }

    public double getTargetRpm2() {
        return targetRpm[1];
    }

    // Both motors
    public Command increaseTargetRpm() {
        return instant(() -> {
            setTargetRpm1(getTargetRpm1() + RPM_STEP);
            setTargetRpm2(getTargetRpm2() + RPM_STEP);
        });
    }

    public Command decreaseTargetRpm() {
        return instant(() -> {
            setTargetRpm1(Math.max(0.0, getTargetRpm1() - RPM_STEP));
            setTargetRpm2(Math.max(0.0, getTargetRpm2() - RPM_STEP));
        });
    }

    // Motor 1 only
    public Command increaseTargetRpm1() {
        return instant(() -> setTargetRpm1(getTargetRpm1() + RPM_STEP));
    }

    public Command decreaseTargetRpm1() {
        return instant(() -> setTargetRpm1(Math.max(0.0, getTargetRpm1() - RPM_STEP)));
    }

    // Motor 2 only
    public Command increaseTargetRpm2() {
        return instant(() -> setTargetRpm2(getTargetRpm2() + RPM_STEP));
    }

    public Command decreaseTargetRpm2() {
        return instant(() -> setTargetRpm2(Math.max(0.0, getTargetRpm2() - RPM_STEP)));
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

    private double rpmThrottle(int motorIndex, double currentRpm) {
        double target = targetRpm[motorIndex];
        double error = target - Math.abs(currentRpm);
        integralError[motorIndex] = Math.max(-MAX_INTEGRAL,
                Math.min(MAX_INTEGRAL, integralError[motorIndex] + error));
        double kff = FF_BASE_POWER / (target + FF_RPM_OFFSET);
        double throttle = kff * target + kp * error + ki * integralError[motorIndex];
        return Math.max(0.0, Math.min(1.0, throttle));
    }

    @Override
    public void periodic() {
        switch (shooterState) {
            case FORWARD:
                shootMotor1.setThrottle(rpmThrottle(0, getRPM1()));
                shootMotor2.setThrottle(rpmThrottle(1, getRPM2()));
                break;
            case REVERSE:
                shootMotor1.setThrottle(-rpmThrottle(0, getRPM1()));
                shootMotor2.setThrottle(-rpmThrottle(1, getRPM2()));
                break;
            case OFF:
                shootMotor1.setThrottle(OFF_POWER);
                shootMotor2.setThrottle(OFF_POWER);
                break;
        }
    }
}
