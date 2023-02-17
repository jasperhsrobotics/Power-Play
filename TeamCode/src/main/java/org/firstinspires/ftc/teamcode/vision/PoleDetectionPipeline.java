package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;


public final class PoleDetectionPipeline extends OpenCvPipeline {
    private static final Scalar[] BOUNDS = new Scalar[]{new Scalar(10.0, 100.0, 100.0), new Scalar(40.0, 255.0, 255.0)};
    private static final Size BLURSIZE = new Size(1, 1);
    public static final int MAX_OFFSET = 1;

    private final Telemetry telemetry;

    public PoleDetectionPipeline(@NotNull Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public PoleDetectionPipeline() {
        telemetry = null;
    }

    /**
     * Offset the center of the largest yellow blob is from the middle of the screen, in pixels.
     */
    public double error = Double.NaN;

    /**
     * See whether there is an actual detection (although there are a lot of false positives
     * and uncertainties for this one), or if there is no suitable candidate.
     */
    public boolean getDetected() {
        return !Double.isNaN(error);
    }

    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat kernel = new Mat();
    private final Mat hierarchy = new Mat();

    private final List<MatOfPoint> contours = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, BOUNDS[0], BOUNDS[1], mask);
        Imgproc.GaussianBlur(mask, mask, BLURSIZE, 0);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        contours.clear();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<MatOfPoint2f> approxs = DetectionUtils.getApproximates(contours);
        List<RotatedRect> rotatedRects = DetectionUtils.getRotatedRects(approxs);

        if (rotatedRects.size() != 0) {
            RotatedRect widest = Collections.max(rotatedRects, Comparator.comparing((rect -> rect.size.width)));
            Imgproc.rectangle(input, widest.boundingRect(), DetectionUtils.GREEN);

            double middle = widest.center.x;
            Imgproc.line(input, new Point(middle, 0), new Point(middle, input.height()), DetectionUtils.RED);

            error = input.width() / 2.0 - middle;
        } else {
            error = Double.NaN;
        }

        if (telemetry != null) {
            telemetry.addData("pole error", error);
//            telemetry.update();
        }
        return input;
    }
}
