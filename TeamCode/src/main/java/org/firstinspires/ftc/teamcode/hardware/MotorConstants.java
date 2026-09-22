package org.firstinspires.ftc.teamcode.hardware;

public final class MotorConstants {
    private MotorConstants() {}
    public static final class GoBILDA {
        private GoBILDA() {}
        public static final double RPM_6000_COUNTS_PER_REV = 28.0;
        public static final double RPM_1620_COUNTS_PER_REV = ((1+(46.0/17.0)) * 28.0);
        public static final double RPM_1150_COUNTS_PER_REV = ((1+(46.0/11.0)) * 28.0);
        public static final double RPM_435_COUNTS_PER_REV = ((((1+(46.0/17.0))) * (1+(46.0/17.0))) * 28.0);
        public static final double RPM_312_COUNTS_PER_REV = ((((1+(46.0/17.0))) * (1+(46.0/11.0))) * 28.0);
        public static final double RPM_223_COUNTS_PER_REV = ((((1+(46.0/11.0))) * (1+(46.0/11.0))) * 28.0);
        public static final double RPM_117_COUNT_PER_REV = ((((1+(46.0/17.0))) * (1+(46.0/17.0))) * (1+(46.0/17.0)) * 28.0);
        public static final double RPM_84_COUNTS_PER_REV = ((((1+(46.0/17.0))) * (1+(46.0/17.0))) * (1+(46.0/11.0)) * 28.0);
        public static final double RPM_60_COUNTS_PER_REV = ((((1+(46.0/17.0))) * (1+(46.0/11.0))) * (1+(46.0/11.0)) * 28.0);
        public static final double RPM_43_COUNTS_PER_REV = ((((1+(46.0/11.0))) * (1+(46.0/11.0))) * (1+(46.0/11.0)) * 28.0);
        public static final double RPM_30_COUNTS_PER_REV = ((((((1+(46.0/17.0))) * (1+(46.0/17.0))) * (1+(46.0/17.0))) * (1+(46.0/17.0))) * 28.0);
    }
}