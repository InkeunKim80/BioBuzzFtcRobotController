package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.robot.Mechanism;

public class Scoop implements Mechanism {
    private Servo scoopServo;
    private ScoopState scoopState = ScoopState.CLOSED;

    public enum ScoopState {
        OPEN,
        CLOSED
    }

    private static final double SCOOP_OPEN = 1.0;
    private static final double SCOOP_CLOSED = 0.0;

    public Scoop() {}

    public void setScoopServo(Servo servo) {
        this.scoopServo = servo;
        scoopServo.setPosition(SCOOP_CLOSED);
    }

    public Command setState(ScoopState scoopState) {
        return instant(() -> {
            this.scoopState = scoopState;
            updateScoopPosition();
        });
    }

    public Command open() {
        return setState(ScoopState.OPEN);
    }

    public Command close() {
        return setState(ScoopState.CLOSED);
    }

    public Command toggle() {
        return instant(() -> {
            scoopState = (scoopState == ScoopState.OPEN) ? ScoopState.CLOSED : ScoopState.OPEN;
            updateScoopPosition();
        });
    }

    public ScoopState getState() {
        return scoopState;
    }

    public double getPosition() {
        return scoopServo != null ? scoopServo.getPosition() : SCOOP_CLOSED;
    }

    private void updateScoopPosition() {
        if (scoopServo != null) {
            if (scoopState == ScoopState.OPEN) {
                scoopServo.setPosition(SCOOP_OPEN);
            } else {
                scoopServo.setPosition(SCOOP_CLOSED);
            }
        }
    }

    @Override
    public void periodic() {
    }
}
