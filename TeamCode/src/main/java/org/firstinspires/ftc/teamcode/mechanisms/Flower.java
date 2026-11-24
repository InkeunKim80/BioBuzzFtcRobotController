package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.robot.Mechanism;

public class Flower implements Mechanism {
    private static final double POSITION_DOWN = 1.0;
    private static final double POSITION_UP = 0.5;
    private static final double INIT_POSITION = 0.5;

    private Servo servo;

    public void setServo(Servo servo) {
        this.servo = servo;
        servo.setPosition(INIT_POSITION);
    }

    public void setServoAtOne(Servo servo) {
        this.servo = servo;
        servo.setPosition(POSITION_DOWN);
    }

    public Command positionOne() {
        return instant(() -> servo.setPosition(POSITION_DOWN));
    }

    public Command positionZero() {
        return instant(() -> servo.setPosition(POSITION_UP));
    }
}
