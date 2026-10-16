package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;

public final class PoseStorage {
    private static Pose lastPose;

    private PoseStorage() {}

    public static void save(Pose pose) {
        lastPose = pose;
    }

    public static Pose load() {
        return lastPose;
    }
}
