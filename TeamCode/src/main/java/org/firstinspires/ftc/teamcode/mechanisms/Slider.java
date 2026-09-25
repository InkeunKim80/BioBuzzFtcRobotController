package org.firstinspires.ftc.teamcode.mechanisms;

import static dev.nextftc.units.Units.Degrees;
import static dev.nextftc.units.Units.RotationsPerMinute;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Slider implements Mechanism {

    private double targetPosition = 0;
    private boolean usePositionControl = false;

    private static final double KP = 0.01;
    private static final double KI = 0.001;
    private static final double KD = 0.005;

    private double integralError = 0;
    private double lastError = 0;

    private static final double POSITION_TOLERANCE = 10.0;
    private static final double MAX_POWER = 1.0;
    private static final double MIN_POWER = -1.0;

    private static final double DEGREES_PER_TICK = 360.0 / 28.0;

    private final NextMotor sliderMotor = new NextMotor(
            "sliderMotor",
            Degrees.of(360.0 / 28.0)
    );

    public Slider() {
        sliderMotor.setDirection(NextMotor.Direction.FORWARD);
    }

    public Command moveToPosition(double position) {
        return instant(() -> {
            targetPosition = position;
            usePositionControl = true;
            integralError = 0;
            lastError = 0;
        });
    }

    public Command extend() {
        return moveToPosition(2000);
    }

    public Command returnToStart() {
        return moveToPosition(0);
    }

    public Command off() {
        return instant(() -> usePositionControl = false);
    }

    public double getCurrentPosition() {
        return sliderMotor.getEncoderPosition().into(Degrees) / DEGREES_PER_TICK;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getRPM(){
        return sliderMotor.getEncoderVelocity().into(RotationsPerMinute);
    }

    public double getPower(){
        return sliderMotor.getThrottle();
    }

    public boolean isAtTarget() {
        return Math.abs(getCurrentPosition() - targetPosition) <= POSITION_TOLERANCE;
    }

    @Override
    public void periodic() {
        if (usePositionControl) {
            double currentPosition = getCurrentPosition();
            double error = targetPosition - currentPosition;

            integralError += error;
            integralError = Math.max(-100, Math.min(100, integralError));

            double derivative = error - lastError;
            lastError = error;

            double power = (KP * error) + (KI * integralError) + (KD * derivative);
            power = Math.max(MIN_POWER, Math.min(MAX_POWER, power));

            sliderMotor.setThrottle(power);
        }
    }
}
