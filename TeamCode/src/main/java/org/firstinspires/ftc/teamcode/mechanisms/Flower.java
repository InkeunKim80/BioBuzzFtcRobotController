package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;

import dev.nextftc.robot.Mechanism;

public class Flower implements Mechanism {
    private static final double RUN_POWER = 1.0;
    private static final double STOP_POWER = 0.0;

    private CRServo servo;

    public void setServo(CRServo servo) {
        this.servo = servo;
        servo.setPower(STOP_POWER);
    }

    public Command run() {
        return instant(() -> servo.setPower(RUN_POWER));
    }

    public Command stop() {
        return instant(() -> servo.setPower(STOP_POWER));
    }

    public Command toggle() {
        return instant(() -> servo.setPower(servo.getPower() != STOP_POWER ? STOP_POWER : RUN_POWER));
    }
}
