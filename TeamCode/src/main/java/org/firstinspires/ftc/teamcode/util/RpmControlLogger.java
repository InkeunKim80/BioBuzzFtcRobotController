package org.firstinspires.ftc.teamcode.util;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Logs RPM control debug data to a timestamped CSV file under /sdcard/FIRST/rpm_logs
 * on the Control Hub for RPM control tuning and validation.
 *
 * Captures: time, targetRPM, actualRPM1, actualRPM2, error, power, powerMultiplier,
 *           integralError, pidOutput, state
 *
 * Writes are buffered and periodically flushed to minimize per-loop I/O overhead.
 * Pull the file off the robot with `adb pull` or through Control Hub's web file manager.
 */
public class RpmControlLogger {

    private static final File LOG_DIR = new File("/sdcard/FIRST/rpm_logs");

    private final BufferedWriter writer;
    private final long startTimeNs;
    private final String filePath;
    private long lastFlushNs;
    private static final long FLUSH_INTERVAL_NS = 1_000_000_000L;

    public RpmControlLogger(String opModeName) {
        BufferedWriter w = null;
        String path = null;
        try {
            if (!LOG_DIR.exists()) {
                LOG_DIR.mkdirs();
            }
            String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            File file = new File(LOG_DIR, opModeName + "_" + timestamp + ".csv");
            w = new BufferedWriter(new FileWriter(file));
            w.write("time_s,targetRPM,actualRPM1,actualRPM2,error,power,powerMultiplier,integralError,pidOutput,state");
            w.newLine();
            path = file.getAbsolutePath();
        } catch (IOException e) {
            w = null;
        }
        this.writer = w;
        this.filePath = path;
        this.startTimeNs = System.nanoTime();
        this.lastFlushNs = this.startTimeNs;
    }

    // Returns file path if successful, null if file creation failed
    public String filePath() {
        return filePath;
    }

    /**
     * Log RPM control data
     * @param targetRPM Target RPM value
     * @param actualRPM1 Actual RPM from Motor 1
     * @param actualRPM2 Actual RPM from Motor 2
     * @param error Error value (target - actual)
     * @param power Throttle power output (-1.0 to 1.0)
     * @param powerMultiplier Power multiplier (0.1 to 1.0)
     * @param integralError Accumulated integral error from PID
     * @param pidOutput Raw PID output before clamping
     * @param state Shooter state (FORWARD/REVERSE/OFF)
     */
    public void log(double targetRPM, double actualRPM1, double actualRPM2,
                    double error, double power, double powerMultiplier,
                    double integralError, double pidOutput, String state) {
        if (writer == null) return;

        long now = System.nanoTime();
        double t = (now - startTimeNs) / 1e9;

        try {
            writer.write(String.format(Locale.US, "%.3f,%.1f,%.1f,%.1f,%.1f,%.4f,%.2f,%.4f,%.4f,%s",
                    t, targetRPM, actualRPM1, actualRPM2, error, power,
                    powerMultiplier, integralError, pidOutput, state));
            writer.newLine();

            if (now - lastFlushNs >= FLUSH_INTERVAL_NS) {
                writer.flush();
                lastFlushNs = now;
            }

        } catch (IOException e) {
            // Keep the OpMode running
        }
    }

    public void close() {
        if (writer == null) return;
        try {
            writer.close();
        } catch (IOException e) {
            // Nothing more we can do
        }
    }
}
