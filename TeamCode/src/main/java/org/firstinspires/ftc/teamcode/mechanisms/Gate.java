package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.robot.Mechanism;

public class Gate implements Mechanism {
    private static final double OPEN_POSITION = 1.0;
    private static final double CLOSED_POSITION = 0.0;

    private Servo servo;

    public void setServo(Servo servo) {
        this.servo = servo;
        servo.setPosition(CLOSED_POSITION);
    }

    public Command open() {
        return instant(() -> servo.setPosition(OPEN_POSITION));
    }

    public Command close() {
        return instant(() -> servo.setPosition(CLOSED_POSITION));
    }

    public Command toggle() {
        return instant(() -> servo.setPosition(servo.getPosition() > 0.5 ? CLOSED_POSITION : OPEN_POSITION));
    }

    public double getPosition() {
        return servo != null ? servo.getPosition() : CLOSED_POSITION;
    }
}
