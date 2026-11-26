
package org.firstinspires.ftc.teamcode.test;
 
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
 
import org.firstinspires.ftc.teamcode.hardware.MotorConstants;
 
/**
 * Open-loop test for the two shooter motors.
 *
 * Purpose: measure steady-state RPM at fixed power levels and fit
 *     power = kS + kV * rpm
 * i.e. the feedforward constants used by ShooterDual (rpm = (power - kS) / kV).
 *
 * Controls
 *   MANUAL mode (default)
 *     dpad up / dpad down : power +0.05 / -0.05 (both motors)
 *     A                   : set power to 0
 *     X                   : start automatic sweep
 *   AUTO sweep
 *     B                   : abort sweep and stop motors
 *   Any time
 *     B                   : stop motors
 *
 * Keep the shooter clear of balls and hands before running.
 */
@TeleOp(name = "Motor FF Test", group = "Test")
public class MotorTest extends LinearOpMode {
 
    // Same names and directions as ShooterDual
    private static final String MOTOR1_NAME = "shootMotor1";
    private static final String MOTOR2_NAME = "shootMotor2";
 
    // Sweep settings
    private static final double[] SWEEP_POWERS = {0.2, 0.3, 0.4, 0.5, 0.6};
    private static final double SPINUP_SEC = 3.0;   // wait for speed to settle
    private static final double SAMPLE_SEC = 1.5;   // average RPM over this window
    private static final double POWER_STEP = 0.05;
 
    // RPMs you plan to shoot at (used for the prediction printout)
    private static final double TARGET_RPM_1 = 1650;
    private static final double TARGET_RPM_2 = 1750;
 
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private final double countsPerRev = MotorConstants.GoBILDA.RPM_6000_COUNTS_PER_REV;
 
    // Sweep results
    private final double[] avgRpm1 = new double[SWEEP_POWERS.length];
    private final double[] avgRpm2 = new double[SWEEP_POWERS.length];
    private int sweepDone = 0;
    private double[] fit1 = null;   // {kS, kV}
    private double[] fit2 = null;
 
    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotorEx.class, MOTOR1_NAME);
        motor2 = hardwareMap.get(DcMotorEx.class, MOTOR2_NAME);
 
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
 
        for (DcMotorEx m : new DcMotorEx[]{motor1, motor2}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
 
        telemetry.addLine("Motor FF Test ready");
        telemetry.addLine("dpad up/down: power, A: zero, X: auto sweep, B: stop");
        telemetry.update();
 
        waitForStart();
 
        double power = 0.0;
        boolean prevUp = false;
        boolean prevDown = false;
 
        while (opModeIsActive()) {
            // Edge detection for dpad
            if (gamepad1.dpad_up && !prevUp) power += POWER_STEP;
            if (gamepad1.dpad_down && !prevDown) power -= POWER_STEP;
            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;
            if (gamepad1.a) power = 0.0;
            if (gamepad1.b) power = 0.0;
            power = clamp(power, 0.0, 1.0);
 
            if (gamepad1.x) {
                runSweep();
                power = 0.0;
            }
 
            motor1.setPower(power);
            motor2.setPower(power);
 
            telemetry.addLine("== MANUAL ==");
            telemetry.addData("power", "%.2f", power);
            telemetry.addData("rpm1", "%.0f", getRpm(motor1));
            telemetry.addData("rpm2", "%.0f", getRpm(motor2));
            addResults();
            telemetry.update();
 
            sleep(20);
        }
 
        motor1.setPower(0);
        motor2.setPower(0);
    }
 
    private void runSweep() {
        sweepDone = 0;
        fit1 = null;
        fit2 = null;
 
        for (int i = 0; i < SWEEP_POWERS.length && opModeIsActive(); i++) {
            double p = SWEEP_POWERS[i];
            motor1.setPower(p);
            motor2.setPower(p);
 
            // Spin-up: wait for speed to settle
            long start = System.nanoTime();
            while (opModeIsActive() && elapsed(start) < SPINUP_SEC) {
                if (gamepad1.b) {
                    abortSweep();
                    return;
                }
                telemetry.addLine("== AUTO SWEEP: spin-up ==");
                telemetry.addData("step", "%d / %d", i + 1, SWEEP_POWERS.length);
                telemetry.addData("power", "%.2f", p);
                telemetry.addData("rpm1", "%.0f", getRpm(motor1));
                telemetry.addData("rpm2", "%.0f", getRpm(motor2));
                telemetry.update();
                sleep(20);
            }
 
            // Sampling: average RPM over the window
            double sum1 = 0;
            double sum2 = 0;
            int n = 0;
            start = System.nanoTime();
            while (opModeIsActive() && elapsed(start) < SAMPLE_SEC) {
                if (gamepad1.b) {
                    abortSweep();
                    return;
                }
                double r1 = getRpm(motor1);
                double r2 = getRpm(motor2);
                sum1 += r1;
                sum2 += r2;
                n++;
                telemetry.addLine("== AUTO SWEEP: sampling ==");
                telemetry.addData("step", "%d / %d", i + 1, SWEEP_POWERS.length);
                telemetry.addData("power", "%.2f", p);
                telemetry.addData("rpm1", "%.0f", r1);
                telemetry.addData("rpm2", "%.0f", r2);
                telemetry.update();
                sleep(20);
            }
 
            if (n > 0) {
                avgRpm1[i] = sum1 / n;
                avgRpm2[i] = sum2 / n;
                sweepDone = i + 1;
            }
        }
 
        motor1.setPower(0);
        motor2.setPower(0);
 
        if (sweepDone >= 2) {
            fit1 = linearFit(SWEEP_POWERS, avgRpm1, sweepDone);
            fit2 = linearFit(SWEEP_POWERS, avgRpm2, sweepDone);
        }
    }
 
    private void abortSweep() {
        motor1.setPower(0);
        motor2.setPower(0);
        if (sweepDone >= 2) {
            fit1 = linearFit(SWEEP_POWERS, avgRpm1, sweepDone);
            fit2 = linearFit(SWEEP_POWERS, avgRpm2, sweepDone);
        }
    }
 
    private void addResults() {
        if (sweepDone == 0) return;
 
        telemetry.addLine("== SWEEP RESULT (avg rpm) ==");
        for (int i = 0; i < sweepDone; i++) {
            telemetry.addData(String.format("power %.2f", SWEEP_POWERS[i]),
                    "m1 %.0f | m2 %.0f", avgRpm1[i], avgRpm2[i]);
        }
 
        if (fit1 != null) {
            telemetry.addData("motor1 kS / kV", "%.4f / %.6f", fit1[0], fit1[1]);
            telemetry.addData("motor1 power @ " + (int) TARGET_RPM_1,
                    "%.3f", fit1[0] + fit1[1] * TARGET_RPM_1);
        }
        if (fit2 != null) {
            telemetry.addData("motor2 kS / kV", "%.4f / %.6f", fit2[0], fit2[1]);
            telemetry.addData("motor2 power @ " + (int) TARGET_RPM_2,
                    "%.3f", fit2[0] + fit2[1] * TARGET_RPM_2);
        }
    }
 
    /**
     * Least-squares fit of power = kS + kV * rpm.
     * x = measured rpm, y = applied power. Returns {kS, kV}.
     */
    private static double[] linearFit(double[] powers, double[] rpms, int count) {
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (int i = 0; i < count; i++) {
            double x = rpms[i];
            double y = powers[i];
            sx += x;
            sy += y;
            sxx += x * x;
            sxy += x * y;
        }
        double denom = count * sxx - sx * sx;
        if (Math.abs(denom) < 1e-9) return new double[]{0.0, 0.0};
        double kV = (count * sxy - sx * sy) / denom;
        double kS = (sy - kV * sx) / count;
        return new double[]{kS, kV};
    }
 
    private double getRpm(DcMotorEx motor) {
        return Math.abs(motor.getVelocity()) * 60.0 / countsPerRev;
    }
 
    private static double elapsed(long startNanos) {
        return (System.nanoTime() - startNanos) * 1e-9;
    }
 
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
 