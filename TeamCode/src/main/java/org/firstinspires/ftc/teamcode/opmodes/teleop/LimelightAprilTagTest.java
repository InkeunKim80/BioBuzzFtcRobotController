
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Limelight AprilTag Test")
public class LimelightAprilTagTest extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(
            Limelight3A.class, "limelight"
        );

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Limelight Ready");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {

                LLResult result = limelight.getLatestResult();

                if (result == null || !result.isValid()) {
                    telemetry.addLine("No valid target");
                } else {

                    List<LLResultTypes.FiducialResult> tags =
                        result.getFiducialResults();

                    int count = 0;

                    for (LLResultTypes.FiducialResult tag : tags) {

                        int id = tag.getFiducialId();

                        if (id < 30 || id > 45) {
                            continue;
                        }

                        count++;

                        double tx = tag.getTargetXDegrees();
                        double ty = tag.getTargetYDegrees();

                        telemetry.addData("Tag ID", id);
                        telemetry.addData("Horizontal", "%.2f deg", tx);
                        telemetry.addData("Vertical", "%.2f deg", ty);
                    }

                    telemetry.addData("BioBuzz Tags", count);
                }

                telemetry.update();
                sleep(50);
            }
        } finally {
            limelight.stop();
        }
    }
}
