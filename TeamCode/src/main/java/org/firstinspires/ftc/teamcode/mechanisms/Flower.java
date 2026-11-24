package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.robot.Mechanism;

public class Flower implements Mechanism {
    // UP: scoop raised, DOWN: scoop lowered
    private static final double POSITION_DOWN = 1.0;
    private static final double POSITION_UP = 0.5;

    private Servo servo;

    // Called at INIT: the flower starts in the UP position.
    public void setServo(Servo servo) {
        this.servo = servo;
        servo.setPosition(POSITION_UP);
    }

    public Command down() {
        return instant(() -> servo.setPosition(POSITION_DOWN));
    }

    public Command up() {
        return instant(() -> servo.setPosition(POSITION_UP));
    }
}
