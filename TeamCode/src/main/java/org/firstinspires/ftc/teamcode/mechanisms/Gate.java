package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.robot.Mechanism;

public class Gate implements Mechanism {
    // Position servo: 0.0 .. 1.0
    private static final double OPEN_POSITION = 1.0;
    private static final double CLOSED_POSITION = 0.6;

    private Servo servo;

    public void setServo(Servo servo) {
        this.servo = servo;
        servo.setPosition(CLOSED_POSITION);
    }

    // run() = open the gate (shoot), stop() = close the gate.
    public Command run() {
        return instant(() -> servo.setPosition(OPEN_POSITION));
    }

    public Command stop() {
        return instant(() -> servo.setPosition(CLOSED_POSITION));
    }

    public double getPosition() {
        return servo != null ? servo.getPosition() : CLOSED_POSITION;
    }
}
