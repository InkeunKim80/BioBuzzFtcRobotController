package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.robot.Mechanism;

public class IndicatorLight implements Mechanism {
    public static final double OFF = 0.0;
    public static final double RED = 0.277;
    public static final double YELLOW = 0.388;
    public static final double GREEN = 0.500;
    public static final double BLUE = 0.611;
    public static final double VIOLET = 0.722;
    public static final double WHITE = 1.0;

    private Servo servo;

    public void setServo(Servo servo) {
        this.servo = servo;
        servo.setPosition(OFF);
    }

    public Command set(double color) {
        return instant(() -> servo.setPosition(color));
    }

    public void setColor(double color) {
        servo.setPosition(color);
    }
}
