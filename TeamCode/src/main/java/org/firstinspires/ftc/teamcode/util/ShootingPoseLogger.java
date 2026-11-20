package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/*
 * Appends labeled shooting poses to a timestamped CSV file under
 * /sdcard/FIRST/shooting_pose_logs on the Control Hub.
 *
 * Each call to log() writes and flushes one line right away, so entries survive an
 * unexpected stop. Pull the file with `adb pull` or the Control Hub web file manager
 * (Program & Manage -> Manage -> Files).
 */
public class ShootingPoseLogger {

    private static final File LOG_DIR = new File("/sdcard/FIRST/shooting_pose_logs");

    private final BufferedWriter writer;
    private final long startTimeNs;
    private final String filePath;

    public ShootingPoseLogger(String opModeName) {
        BufferedWriter w = null;
        String path = null;
        try {
            if (!LOG_DIR.exists()) {
                LOG_DIR.mkdirs();
            }
            String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            File file = new File(LOG_DIR, opModeName + "_" + timestamp + ".csv");
            w = new BufferedWriter(new FileWriter(file));
            w.write("time_s,label,x_in,y_in,heading_deg");
            w.newLine();
            w.flush();
            path = file.getAbsolutePath();
        } catch (IOException e) {
            w = null;
        }
        this.writer = w;
        this.filePath = path;
        this.startTimeNs = System.nanoTime();
    }

    // Null if the file could not be created (e.g. storage unavailable).
    public String filePath() {
        return filePath;
    }

    public void log(String label, Pose pose) {
        if (writer == null) return;

        double t = (System.nanoTime() - startTimeNs) / 1e9;

        try {
            writer.write(String.format(Locale.US, "%.3f,%s,%.3f,%.3f,%.3f",
                    t, label, pose.x(), pose.y(), Math.toDegrees(pose.heading())));
            writer.newLine();
            writer.flush();
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
