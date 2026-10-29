package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;

import dev.nextftc.robot.Mechanism;

public class IntakeServo implements Mechanism {
    // Continuous servo power (flip sign if it spins the wrong way)
    private static final double FORWARD_POWER = 1.0;
    private static final double REVERSE_POWER = -1.0;
    private static final double STOP_POWER = 0.0;

    private CRServo servo;

    public void setServo(CRServo servo) {
        this.servo = servo;
        servo.setPower(STOP_POWER);
    }

    public Command forward() {
        return instant(() -> servo.setPower(FORWARD_POWER));
    }

    public Command reverse() {
        return instant(() -> servo.setPower(REVERSE_POWER));
    }

    public Command off() {
        return instant(() -> servo.setPower(STOP_POWER));
    }

    public double getPower() {
        return servo != null ? servo.getPower() : STOP_POWER;
    }
}
