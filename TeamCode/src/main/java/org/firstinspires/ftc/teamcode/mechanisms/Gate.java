package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;

import dev.nextftc.robot.Mechanism;

public class Gate implements Mechanism {
    // Continuous servo power (flip sign if it spins the wrong way)
    private static final double OPEN_POWER = 1.0;
    private static final double REVERSE_POWER = -1.0;
    private static final double STOP_POWER = 0.0;

    private CRServo servo;

    public void setServo(CRServo servo) {
        this.servo = servo;
        servo.setPower(STOP_POWER);
    }

    public Command open() {
        return instant(() -> servo.setPower(OPEN_POWER));
    }

    public Command close() {
        return instant(() -> servo.setPower(STOP_POWER));
    }

    public Command reverse() {
        return instant(() -> servo.setPower(REVERSE_POWER));
    }

    public Command toggle() {
        return instant(() -> servo.setPower(servo.getPower() != STOP_POWER ? STOP_POWER : OPEN_POWER));
    }

    public Command setPower(double power) {
        return instant(() -> servo.setPower(clamp(power)));
    }

    private static double clamp(double power) {
        return Math.max(-1.0, Math.min(1.0, power));
    }

    public double getPower() {
        return servo != null ? servo.getPower() : STOP_POWER;
    }
}
