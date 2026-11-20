
package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

public class HiveAprilTagDetector {

    public enum State {
        CHECKING,
        VISIBLE,
        NOT_VISIBLE,
        UNKNOWN
    }

    private static final long CHECK_TIME_MS = 500;
    private static final int MIN_FRAMES = 5;
    private static final int MIN_DETECTIONS = 2;

    private final Limelight3A limelight;
    private final int firstTagId;
    private final int lastTagId;

    private long startTime;
    private int validFrames;
    private int detectedFrames;
    private double lastTimestamp;

    private State state = State.UNKNOWN;

    public HiveAprilTagDetector(
            Limelight3A limelight,
            int firstTagId,
            int lastTagId) {

        this.limelight = limelight;
        this.firstTagId = firstTagId;
        this.lastTagId = lastTagId;
    }

    public void startCheck() {
        startTime = System.currentTimeMillis();
        validFrames = 0;
        detectedFrames = 0;
        lastTimestamp = Double.NaN;
        state = State.CHECKING;
    }

    public State update() {

        if (state != State.CHECKING)
            return state;

        LLResult result = limelight.getLatestResult();

        if (result != null) {

            double timestamp =
                result.getControlHubTimeStamp();

            if (Double.isFinite(timestamp) &&
                (Double.isNaN(lastTimestamp) ||
                 timestamp > lastTimestamp)) {

                lastTimestamp = timestamp;

                // A valid Limelight pipeline result can
                // contain zero target detections.
                validFrames++;

                if (result.isValid()) {

                    for (LLResultTypes.FiducialResult tag
                            : result.getFiducialResults()) {

                        int id = tag.getFiducialId();

                        if (id >= firstTagId &&
                            id <= lastTagId) {

                            detectedFrames++;
                            break;
                        }
                    }
                }
            }
        }

        if (System.currentTimeMillis() - startTime
                >= CHECK_TIME_MS) {

            if (validFrames < MIN_FRAMES) {
                state = State.UNKNOWN;
            } else if (detectedFrames >= MIN_DETECTIONS) {
                state = State.VISIBLE;
            } else if (detectedFrames == 0) {
                state = State.NOT_VISIBLE;
            } else {
                state = State.UNKNOWN;
            }
        }

        return state;
    }

    public State getState() {
        return state;
    }

    public int getValidFrames() {
        return validFrames;
    }

    public int getDetectedFrames() {
        return detectedFrames;
    }
}
