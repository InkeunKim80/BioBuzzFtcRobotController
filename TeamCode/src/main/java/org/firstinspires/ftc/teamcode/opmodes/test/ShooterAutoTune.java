package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.MotorConstants;

/**
 * Automatic PI tuning for the shooter motors (feedforward + PI structure of ShooterDual).
 *
 * Method (step-response identification + SIMC tuning rule)
 *   1. Settle at a low power (FF for 60% of the target RPM).
 *   2. Step to the FF power for the target RPM and record the RPM response.
 *   3. Fit a first-order-plus-dead-time model  G(s) = K e^(-theta s) / (tau s + 1)
 *        K     = delta rpm / delta power
 *        tau   = 1.5 * (t63.2 - t28.3)      (Smith's two-point method)
 *        theta = t63.2 - tau
 *   4. SIMC PI rule (tauC = TAU_C_FACTOR * tau):
 *        Kp = tau / (K * (tauC + theta))     [power per RPM]
 *        Ti = min(tau, 4 * (tauC + theta))
 *        Ki = Kp / Ti                        [power per (RPM * s)]
 *   5. (optional) Verify: run FF + PI with the new gains from a standstill and report
 *      overshoot and settling time.
 *
 * Controls
 *   X : run auto-tune (about 15 seconds)
 *   A : verify with the tuned gains (about 6 seconds)
 *   B : stop everything / abort
 *
 * Keep the shooter clear of balls and hands before running.
 * Use the same battery state you will race with.
 */
@TeleOp(name = "Shooter PI AutoTune", group = "Test")
public class ShooterAutoTune extends LinearOpMode {

    private static final String[] MOTOR_NAMES = {"shootMotor1", "shootMotor2"};

    // Target RPM per motor (same as ShooterDual defaults)
    private static final double[] TARGET_RPM = {1650, 1750};

    // Feedforward per motor (same as ShooterDual; keep in sync with the MotorTest result)
    private static final double[] KS = {0.0909, 0.0803};
    private static final double[] KV = {0.000152, 0.000156};

    // Step test settings
    private static final double LOW_FRACTION = 0.6;   // start the step from 60% of target RPM
    private static final double SETTLE_SEC = 3.0;     // settle time at the low power
    private static final double AVG_SEC = 0.5;        // averaging window for steady-state RPM
    private static final double STEP_SEC = 5.0;       // recording time after the step
    private static final double STEP_FILTER_ALPHA = 0.5;
    private static final double MIN_DELTA_RPM = 200.0;

    // Tuning aggressiveness: closed-loop time constant = TAU_C_FACTOR * tau.
    // Smaller = faster / more aggressive, larger = slower / more robust.
    private static final double TAU_C_FACTOR = 0.5;

    // Verification settings (match ShooterDual)
    private static final double VERIFY_SEC = 5.0;
    private static final double SETTLE_BAND = 0.03;   // +-3% of target counts as settled
    private static final double I_ZONE_RPM = 300.0;
    private static final double MAX_I_POWER = 0.15;
    private static final double RPM_ALPHA = 0.4;

    private static final int BUF = 1000;

    private final DcMotorEx[] motors = new DcMotorEx[2];
    private final double countsPerRev = MotorConstants.GoBILDA.RPM_6000_COUNTS_PER_REV;

    // Identification / tuning results
    private boolean tuned = false;
    private final double[] modelK = new double[2];
    private final double[] modelTau = new double[2];
    private final double[] modelTheta = new double[2];
    private final double[] kp = new double[2];
    private final double[] ki = new double[2];
    private final boolean[] settledWarning = new boolean[2];
    private String message = "";

    // Verification results
    private boolean verified = false;
    private final double[] overshootPct = new double[2];
    private final double[] settleTime = new double[2];

    @Override
    public void runOpMode() {
        for (int i = 0; i < 2; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        // Same directions as ShooterDual
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Shooter PI AutoTune ready");
        telemetry.addLine("X: auto-tune, A: verify, B: stop");
        telemetry.update();

        waitForStart();

        boolean prevX = false;
        boolean prevA = false;

        while (opModeIsActive()) {
            boolean x = gamepad1.x;
            boolean a = gamepad1.a;

            if (x && !prevX) {
                runAutoTune();
            } else if (a && !prevA) {
                if (tuned) {
                    runVerify();
                } else {
                    message = "Run auto-tune (X) first.";
                }
            }
            prevX = x;
            prevA = a;

            stopMotors();
            showResults();
            sleep(50);
        }

        stopMotors();
    }

    // ---------------------------------------------------------------------------------
    // Identification
    // ---------------------------------------------------------------------------------
    private void runAutoTune() {
        tuned = false;
        verified = false;
        message = "";

        // 1. Settle at the low power and average the steady-state RPM
        for (int i = 0; i < 2; i++) {
            motors[i].setPower(clamp(KS[i] + KV[i] * TARGET_RPM[i] * LOW_FRACTION, 0, 1));
        }
        double[] low = holdAndAverage("settling at low power", SETTLE_SEC, AVG_SEC);
        if (low == null) {
            message = "Aborted.";
            stopMotors();
            return;
        }

        // 2. Step up to the target-RPM power and record the response
        double[] pHigh = new double[2];
        double[] pLow = new double[2];
        for (int i = 0; i < 2; i++) {
            pHigh[i] = clamp(KS[i] + KV[i] * TARGET_RPM[i], 0, 1);
            pLow[i] = clamp(KS[i] + KV[i] * TARGET_RPM[i] * LOW_FRACTION, 0, 1);
        }

        double[] tBuf = new double[BUF];
        double[][] rBuf = new double[2][BUF];
        double[] filtered = {low[0], low[1]};
        int n = 0;

        for (int i = 0; i < 2; i++) motors[i].setPower(pHigh[i]);
        long start = System.nanoTime();
        double t;
        while (opModeIsActive() && (t = elapsed(start)) < STEP_SEC && n < BUF) {
            if (gamepad1.b) {
                message = "Aborted.";
                stopMotors();
                return;
            }
            for (int i = 0; i < 2; i++) {
                filtered[i] += STEP_FILTER_ALPHA * (getRpm(i) - filtered[i]);
                rBuf[i][n] = filtered[i];
            }
            tBuf[n] = t;
            n++;

            telemetry.addLine("== AUTO-TUNE: recording step response ==");
            telemetry.addData("t", "%.1f / %.1f s", t, STEP_SEC);
            telemetry.addData("rpm1", "%.0f", filtered[0]);
            telemetry.addData("rpm2", "%.0f", filtered[1]);
            telemetry.update();
            sleep(20);
        }
        stopMotors();
        if (!opModeIsActive() || n < 20) {
            message = "Not enough data.";
            return;
        }

        // 3. Fit the model and compute the gains for each motor
        for (int i = 0; i < 2; i++) {
            double highAvg = windowAverage(tBuf, rBuf[i], n, STEP_SEC - AVG_SEC, STEP_SEC);
            double prevAvg = windowAverage(tBuf, rBuf[i], n, STEP_SEC - 2 * AVG_SEC, STEP_SEC - AVG_SEC);
            double deltaRpm = highAvg - low[i];
            double deltaPower = pHigh[i] - pLow[i];

            if (deltaRpm < MIN_DELTA_RPM || deltaPower <= 0) {
                message = "Motor " + (i + 1) + ": RPM change too small (" + Math.round(deltaRpm) + ").";
                return;
            }

            settledWarning[i] = Math.abs(highAvg - prevAvg) > 0.03 * deltaRpm;

            double t28 = crossingTime(tBuf, rBuf[i], n, low[i] + 0.283 * deltaRpm);
            double t63 = crossingTime(tBuf, rBuf[i], n, low[i] + 0.632 * deltaRpm);
            if (Double.isNaN(t28) || Double.isNaN(t63)) {
                message = "Motor " + (i + 1) + ": response did not reach 63%.";
                return;
            }

            double tau = Math.max(1.5 * (t63 - t28), 0.05);
            double theta = Math.max(t63 - tau, 0.02);
            double k = deltaRpm / deltaPower;           // RPM per unit power

            double tauC = TAU_C_FACTOR * tau;
            double kpVal = tau / (k * (tauC + theta));  // power per RPM
            double ti = Math.min(tau, 4.0 * (tauC + theta));
            double kiVal = kpVal / ti;                  // power per (RPM * s)

            modelK[i] = k;
            modelTau[i] = tau;
            modelTheta[i] = theta;
            kp[i] = kpVal;
            ki[i] = kiVal;
        }

        tuned = true;
        message = "Auto-tune done.";
    }

    // ---------------------------------------------------------------------------------
    // Verification: FF + PI from a standstill with the tuned gains
    // ---------------------------------------------------------------------------------
    private void runVerify() {
        verified = false;
        message = "";

        // Wait for the motors to spin down first
        if (!spinDown()) {
            message = "Aborted.";
            stopMotors();
            return;
        }

        double[] filtered = new double[2];
        double[] integral = new double[2];
        double[] maxRpm = new double[2];
        double[] lastOutside = new double[2];

        long start = System.nanoTime();
        long last = start;
        double t;
        while (opModeIsActive() && (t = elapsed(start)) < VERIFY_SEC) {
            if (gamepad1.b) {
                message = "Aborted.";
                stopMotors();
                return;
            }
            long now = System.nanoTime();
            double dt = Math.min((now - last) * 1e-9, 0.1);
            last = now;

            for (int i = 0; i < 2; i++) {
                filtered[i] += RPM_ALPHA * (getRpm(i) - filtered[i]);
                double error = TARGET_RPM[i] - filtered[i];
                if (Math.abs(error) < I_ZONE_RPM) {
                    integral[i] = clamp(integral[i] + ki[i] * error * dt, -MAX_I_POWER, MAX_I_POWER);
                }
                double power = KS[i] + KV[i] * TARGET_RPM[i] + kp[i] * error + integral[i];
                motors[i].setPower(clamp(power, 0, 1));

                maxRpm[i] = Math.max(maxRpm[i], filtered[i]);
                if (Math.abs(error) > SETTLE_BAND * TARGET_RPM[i]) lastOutside[i] = t;
            }

            telemetry.addLine("== VERIFY ==");
            telemetry.addData("t", "%.1f / %.1f s", t, VERIFY_SEC);
            telemetry.addData("rpm1 / target", "%.0f / %.0f", filtered[0], TARGET_RPM[0]);
            telemetry.addData("rpm2 / target", "%.0f / %.0f", filtered[1], TARGET_RPM[1]);
            telemetry.update();
            sleep(20);
        }
        stopMotors();
        if (!opModeIsActive()) return;

        for (int i = 0; i < 2; i++) {
            overshootPct[i] = Math.max(0, (maxRpm[i] - TARGET_RPM[i]) / TARGET_RPM[i] * 100.0);
            settleTime[i] = lastOutside[i];
        }
        verified = true;
        message = "Verify done.";
    }

    // ---------------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------------
    private double[] holdAndAverage(String label, double settleSec, double avgSec) {
        long start = System.nanoTime();
        double[] sum = new double[2];
        int n = 0;
        double t;
        while (opModeIsActive() && (t = elapsed(start)) < settleSec + avgSec) {
            if (gamepad1.b) return null;
            double r0 = getRpm(0);
            double r1 = getRpm(1);
            if (t >= settleSec) {
                sum[0] += r0;
                sum[1] += r1;
                n++;
            }
            telemetry.addLine("== AUTO-TUNE: " + label + " ==");
            telemetry.addData("t", "%.1f / %.1f s", t, settleSec + avgSec);
            telemetry.addData("rpm1", "%.0f", r0);
            telemetry.addData("rpm2", "%.0f", r1);
            telemetry.update();
            sleep(20);
        }
        if (!opModeIsActive() || n == 0) return null;
        return new double[]{sum[0] / n, sum[1] / n};
    }

    private boolean spinDown() {
        stopMotors();
        long start = System.nanoTime();
        while (opModeIsActive() && elapsed(start) < 6.0) {
            if (gamepad1.b) return false;
            double r0 = getRpm(0);
            double r1 = getRpm(1);
            telemetry.addLine("== waiting for spin-down ==");
            telemetry.addData("rpm1", "%.0f", r0);
            telemetry.addData("rpm2", "%.0f", r1);
            telemetry.update();
            if (r0 < 100 && r1 < 100) return true;
            sleep(50);
        }
        return opModeIsActive();
    }

    private static double windowAverage(double[] t, double[] r, int n, double from, double to) {
        double sum = 0;
        int count = 0;
        for (int j = 0; j < n; j++) {
            if (t[j] >= from && t[j] <= to) {
                sum += r[j];
                count++;
            }
        }
        return count > 0 ? sum / count : r[n - 1];
    }

    /** First time the response crosses the level (linear interpolation), or NaN if never. */
    private static double crossingTime(double[] t, double[] r, int n, double level) {
        if (r[0] >= level) return t[0];
        for (int j = 1; j < n; j++) {
            if (r[j - 1] < level && r[j] >= level) {
                double frac = (level - r[j - 1]) / (r[j] - r[j - 1]);
                return t[j - 1] + frac * (t[j] - t[j - 1]);
            }
        }
        return Double.NaN;
    }

    private void showResults() {
        telemetry.addLine("X: auto-tune | A: verify | B: stop");
        if (!message.isEmpty()) telemetry.addLine(message);

        if (tuned) {
            telemetry.addLine("== TUNED GAINS ==");
            for (int i = 0; i < 2; i++) {
                telemetry.addData("motor" + (i + 1) + " model",
                        "K %.0f rpm/pwr, tau %.2fs, theta %.2fs", modelK[i], modelTau[i], modelTheta[i]);
                telemetry.addData("motor" + (i + 1) + " kp / ki",
                        "%.6f / %.6f", kp[i], ki[i]);
                if (settledWarning[i]) {
                    telemetry.addLine("motor" + (i + 1) + ": response not fully settled, increase STEP_SEC");
                }
            }
            telemetry.addData("average kp / ki",
                    "%.6f / %.6f", (kp[0] + kp[1]) / 2.0, (ki[0] + ki[1]) / 2.0);
        }

        if (verified) {
            telemetry.addLine("== VERIFY RESULT ==");
            for (int i = 0; i < 2; i++) {
                telemetry.addData("motor" + (i + 1),
                        "overshoot %.1f%%, settled (+-3%%) at %.2fs", overshootPct[i], settleTime[i]);
            }
        }
        telemetry.update();
    }

    private void stopMotors() {
        motors[0].setPower(0);
        motors[1].setPower(0);
    }

    private double getRpm(int i) {
        return Math.abs(motors[i].getVelocity()) * 60.0 / countsPerRev;
    }

    private static double elapsed(long startNanos) {
        return (System.nanoTime() - startNanos) * 1e-9;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}