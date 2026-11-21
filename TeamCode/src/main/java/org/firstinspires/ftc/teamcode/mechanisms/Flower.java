package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.robot.Mechanism;

public class Flower implements Mechanism {
    private static final double POSITION_ONE = 1.0;
    private static final double POSITION_ZERO = 0.0;
    private static final double INIT_POSITION = 0.0;

    private Servo servo;

    public void setServo(Servo servo) {
        this.servo = servo;
        servo.setPosition(INIT_POSITION);
    }

    public void setServoAtOne(Servo servo) {
        this.servo = servo;
        servo.setPosition(POSITION_ONE);
    }

    public Command positionOne() {
        return instant(() -> servo.setPosition(POSITION_ONE));
    }

    public Command positionZero() {
        return instant(() -> servo.setPosition(POSITION_ZERO));
    }
}
