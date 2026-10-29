package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.robot.Mechanism;

public class Flower implements Mechanism {
    private static final double RUN_POSITION = 1.0;
    private static final double STOP_POSITION = 0.5;
    private static final long HALF_PERIOD_MS = 500;

    private Servo servo;
    private boolean sweeping = false;
    private boolean atRunPosition = false;
    private long lastSwitchMs = 0;

    public void setServo(Servo servo) {
        this.servo = servo;
        servo.setPosition(STOP_POSITION);
    }

    public Command run() {
        return instant(() -> servo.setPosition(RUN_POSITION));
    }

    public Command stop() {
        return instant(() -> servo.setPosition(STOP_POSITION));
    }

    public Command toggle() {
        return instant(() -> servo.setPosition(servo.getPosition() > 0.5 ? STOP_POSITION : RUN_POSITION));
    }

    public Command startSweep() {
        return instant(() -> {
            sweeping = true;
            atRunPosition = true;
            lastSwitchMs = System.currentTimeMillis();
            servo.setPosition(RUN_POSITION);
        });
    }

    public Command stopSweep() {
        return instant(() -> {
            sweeping = false;
            servo.setPosition(STOP_POSITION);
        });
    }

    @Override
    public void periodic() {
        if (!sweeping || servo == null) return;

        long now = System.currentTimeMillis();
        if (now - lastSwitchMs >= HALF_PERIOD_MS) {
            atRunPosition = !atRunPosition;
            lastSwitchMs = now;
        }
        servo.setPosition(atRunPosition ? RUN_POSITION : STOP_POSITION);
    }
}
