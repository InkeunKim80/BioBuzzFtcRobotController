/*
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
    private final double[] targetRpm = {1650, 1750};
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
    private double kp = 0.0006;
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
*/

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
    private final double[] targetRpm = {1650, 1750};

    // Filtered (low-pass) RPM per motor, updated every periodic() regardless of state
    private final double[] filteredRpm = new double[2];

    // Integral term stored directly in power units (already multiplied by ki and dt)
    private final double[] integralPower = new double[2];

    private long lastNanos = System.nanoTime();

    public enum ShooterState {
        FORWARD,
        REVERSE,
        OFF
    }

    private static final double OFF_POWER = 0.0;
    private static final double RPM_STEP = 100.0;

    // Feedforward per motor: power = kS[i] + kV[i] * targetRpm[i]
    // Measured with MotorTest (open-loop sweep, linear fit).
    // index 0 = shootMotor1, index 1 = shootMotor2
    private final double[] kS = {0.0909, 0.0803};
    private final double[] kV = {0.000152, 0.000156};   // power per RPM

    // Feedback (PI) on RPM error (shared by both motors)
    private double kp = 0.0003;   // power per RPM
    private double ki = 0.0005;   // power per (RPM * second)

    // Integrate only when |error| is within this band (prevents windup during spin-up)
    private static final double I_ZONE_RPM = 300.0;
    // Maximum contribution of the integral term, in power units
    private static final double MAX_I_POWER = 0.15;
    // Low-pass filter coefficient for RPM (0..1, higher = less smoothing)
    private static final double RPM_ALPHA = 0.4;
    // Ignore absurd dt values (loop hiccups)
    private static final double MAX_DT = 0.1;

    public ShooterDual() {
        shootMotor1.setDirection(NextMotor.Direction.REVERSE);
        shootMotor2.setDirection(NextMotor.Direction.FORWARD);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void resetIntegral() {
        integralPower[0] = 0;
        integralPower[1] = 0;
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

    // Raw RPM from the encoder (signed)
    public double getRPM1() {
        return shootMotor1.getEncoderVelocity().into(RotationsPerMinute);
    }

    public double getRPM2() {
        return shootMotor2.getEncoderVelocity().into(RotationsPerMinute);
    }

    // Filtered absolute RPM used by the controller
    public double getFilteredRpm1() {
        return filteredRpm[0];
    }

    public double getFilteredRpm2() {
        return filteredRpm[1];
    }

    public double getPower1() {
        return shootMotor1.getThrottle();
    }

    public double getPower2() {
        return shootMotor2.getThrottle();
    }

    // Set both motors to the same target RPM.
    public void setTargetRpm(double rpm) {
        setTargetRpm1(rpm);
        setTargetRpm2(rpm);
    }

    public void setTargetRpm1(double rpm) {
        targetRpm[0] = Math.abs(rpm);
        integralPower[0] = 0;
    }

    public void setTargetRpm2(double rpm) {
        targetRpm[1] = Math.abs(rpm);
        integralPower[1] = 0;
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

    // Gains (for live tuning)
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

    // Per-motor feedforward constants
    public void setKs1(double value) {
        kS[0] = value;
    }

    public void setKs2(double value) {
        kS[1] = value;
    }

    public void setKv1(double value) {
        kV[0] = value;
    }

    public void setKv2(double value) {
        kV[1] = value;
    }

    public double getKs1() {
        return kS[0];
    }

    public double getKs2() {
        return kS[1];
    }

    public double getKv1() {
        return kV[0];
    }

    public double getKv2() {
        return kV[1];
    }

    // Set the same kS / kV on both motors
    public void setKs(double value) {
        kS[0] = value;
        kS[1] = value;
    }

    public void setKv(double value) {
        kV[0] = value;
        kV[1] = value;
    }

    private void updateFilteredRpm() {
        filteredRpm[0] += RPM_ALPHA * (Math.abs(getRPM1()) - filteredRpm[0]);
        filteredRpm[1] += RPM_ALPHA * (Math.abs(getRPM2()) - filteredRpm[1]);
    }

    // Returns throttle in [0, 1]; the sign (direction) is applied by the caller.
    private double rpmThrottle(int motorIndex, double dt) {
        double target = targetRpm[motorIndex];
        if (target <= 0.0) {
            integralPower[motorIndex] = 0;
            return 0.0;
        }

        double error = target - filteredRpm[motorIndex];

        if (Math.abs(error) < I_ZONE_RPM) {
            integralPower[motorIndex] = clamp(
                    integralPower[motorIndex] + ki * error * dt,
                    -MAX_I_POWER, MAX_I_POWER);
        }

        double feedforward = kS[motorIndex] + kV[motorIndex] * target;
        double throttle = feedforward + kp * error + integralPower[motorIndex];
        return clamp(throttle, 0.0, 1.0);
    }

    @Override
    public void periodic() {
        long now = System.nanoTime();
        double dt = Math.min((now - lastNanos) * 1e-9, MAX_DT);
        lastNanos = now;

        updateFilteredRpm();

        switch (shooterState) {
            case FORWARD:
                shootMotor1.setThrottle(rpmThrottle(0, dt));
                shootMotor2.setThrottle(rpmThrottle(1, dt));
                break;
            case REVERSE:
                shootMotor1.setThrottle(-rpmThrottle(0, dt));
                shootMotor2.setThrottle(-rpmThrottle(1, dt));
                break;
            case OFF:
                shootMotor1.setThrottle(OFF_POWER);
                shootMotor2.setThrottle(OFF_POWER);
                break;
        }
    }
}