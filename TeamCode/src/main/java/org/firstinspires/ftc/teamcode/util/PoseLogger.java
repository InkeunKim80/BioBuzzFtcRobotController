package org.firstinspires.ftc.teamcode.util;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/*
 * Appends robot pose samples to a timestamped CSV file under /sdcard/FIRST/localization_logs
 * on the Control Hub, for offline localization validation (e.g. comparing against a simulated
 * path in a Jupyter notebook).
 *
 * Pull the file off the robot either with `adb pull` or through the Control Hub's built-in
 * web file manager (Program & Manage -> Manage -> Files) while connected to its Wi-Fi.
 *
 * Writes are buffered in memory and only flushed to disk on close(), so per-loop overhead
 * stays negligible. Since OpMode loops stop cooperatively (checking opModeIsActive()), calling
 * close() from a finally block after the loop is enough to guarantee the data is saved.
 */

/*
 * Appends robot pose samples to a timestamped CSV file under
 * /sdcard/FIRST/localization_logs on the Control Hub.
 *
 * Writes are buffered and periodically flushed to disk to minimize
 * per-loop I/O overhead while reducing data loss if the OpMode or
 * Robot Controller terminates unexpectedly.
 */
public class PoseLogger {

    private static final File LOG_DIR = new File("/sdcard/FIRST/localization_logs");

    private final BufferedWriter writer;
    private final long startTimeNs;
    private final String filePath;
    private long lastFlushNs;
    private static final long FLUSH_INTERVAL_NS = 1_000_000_000L;

    public PoseLogger(String opModeName) {
        BufferedWriter w = null;
        String path = null;
        try {
            if (!LOG_DIR.exists()) {
                LOG_DIR.mkdirs();
            }
            String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            File file = new File(LOG_DIR, opModeName + "_" + timestamp + ".csv");
            w = new BufferedWriter(new FileWriter(file));
            w.write("time_s,x_in,y_in,heading_deg");
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

    // Null once the file couldn't be created (e.g. storage unavailable); callers can use this
    // to show a telemetry warning instead of silently losing data.
    public String filePath() {
        return filePath;
    }

    public void log(double xInches, double yInches, double headingRadians) {
        if (writer == null) return;

        long now = System.nanoTime();
        double t = (now - startTimeNs) / 1e9;

        try {
            writer.write(String.format(Locale.US, "%.3f,%.3f,%.3f,%.3f",
                    t, xInches, yInches, Math.toDegrees(headingRadians)));
            writer.newLine();

            if (now - lastFlushNs >= FLUSH_INTERVAL_NS) {
                writer.flush();
                lastFlushNs = now;
            }

        } catch (IOException e) {
            // Keep the OpMode running.
        }
    }

    public void close() {
        if (writer == null) return;
        try {
            writer.close();
        } catch (IOException e) {
            // Nothing more we can do.
        }
    }
}
