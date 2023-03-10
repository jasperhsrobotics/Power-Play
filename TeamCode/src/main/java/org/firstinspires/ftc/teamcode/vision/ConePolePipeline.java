package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

@Disabled
public class ConePolePipeline extends OpenCvPipeline {
    private final ConeDetectionPipeline coneDetectionPipeline = ConeDetectionPipeline.redConeDetector();
    private final PoleDetectionPipeline poleDetectionPipeline = new PoleDetectionPipeline();

    public boolean coneDetected = false;
    public boolean poleDetected = false;

    public double coneError = 0.0;
    public double poleError = 0.0;

    private final Telemetry telemetry;

    public ConePolePipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public ConePolePipeline() {
        this.telemetry = null;
    }

    @Override
    public Mat processFrame(Mat mat) {
        coneDetectionPipeline.processFrame(mat);
        poleDetectionPipeline.processFrame(mat);

        coneDetected = coneDetectionPipeline.getDetected();
        poleDetected = poleDetectionPipeline.getDetected();

        coneError = coneDetectionPipeline.error;
        poleError = poleDetectionPipeline.error;

        if (telemetry != null) {
            telemetry.addData("coneError", coneError);
            telemetry.addData("poleError", poleError);
        }

        return mat;
    }
}
