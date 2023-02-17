package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.Nullable;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.TestOnly;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;


public final class ConeDetectionPipeline extends OpenCvPipeline {
    private static final Scalar[] RED1 =
            new Scalar[]{new Scalar(0, 50, 50), new Scalar(10, 255, 255)};
    private static final Scalar[] RED2 =
            new Scalar[]{new Scalar(170, 50, 50), new Scalar(180, 255, 255)};
    private static final Scalar[] BLUE =
            new Scalar[]{new Scalar(100, 50, 50), new Scalar(130, 255, 255)};
    public static final int MAX_OFFSET = 3;

    @Nullable private final Telemetry telemetry;

    public enum ConeColor {RED, BLUE}

    @NotNull public ConeColor conecolor;

    @TestOnly
    public ConeDetectionPipeline(@Nullable Telemetry telemetry) {
        this.telemetry = telemetry;
        conecolor = ConeColor.RED;
    }

    public ConeDetectionPipeline(@NotNull ConeColor conecolor) {
        this.telemetry = null;
        this.conecolor = conecolor;
    }

    public ConeDetectionPipeline(@NotNull ConeColor conecolor, @NotNull Telemetry telemetry) {
        this.telemetry = telemetry;
        this.conecolor = conecolor;
    }

    @NotNull
    public static ConeDetectionPipeline redConeDetector() {
        return new ConeDetectionPipeline(ConeColor.RED);
    }

    @NotNull
    public static ConeDetectionPipeline blueConeDetector() {
        return new ConeDetectionPipeline(ConeColor.BLUE);
    }

    /**
     * Offset the center of the largest red/blue blob is from the
     * middle of the screen, in pixels.
     */
    public double error = Double.NaN;

    /**
     * See whether there is an actual detection, or if there are no suitable candidates.
     */
    public boolean getDetected() {
        return !Double.isNaN(error);
    }

    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat mask2 = new Mat();
    private final Mat kernel = new Mat();
    private final Mat hierarchy = new Mat();

    private final List<MatOfPoint> contours = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        Rect roi = new Rect(0, input.height() / 4, input.width(), input.height() / 2);
        Imgproc.rectangle(input, roi, DetectionUtils.GREEN);
        Mat inputroi = new Mat(input, roi);

        double lo_offset = input.width() / 2.0 - MAX_OFFSET;
        double hi_offset = input.width() / 2.0 + MAX_OFFSET;
        Imgproc.line(input,
                new Point(lo_offset, 0),
                new Point(input.width() / 2.0 - MAX_OFFSET, input.height()),
                DetectionUtils.GREEN
        );
        Imgproc.line(input,
                new Point(hi_offset, 0),
                new Point(input.width() / 2.0 + MAX_OFFSET, input.height()),
                DetectionUtils.GREEN
        );

        Imgproc.cvtColor(inputroi, hsv, Imgproc.COLOR_RGB2HSV);

        if (conecolor == ConeColor.RED) {
            Core.inRange(hsv, RED1[0], RED1[1], mask);
            Core.inRange(hsv, RED2[0], RED2[1], mask2);
            Core.bitwise_or(mask, mask2, mask);
        } else {
            Core.inRange(hsv, BLUE[0], BLUE[1], mask);
        }
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        contours.clear();
        Imgproc.findContours(mask,
                contours,
                hierarchy,
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE
        );

        MatOfPoint approxCone = approximateMaxHull();
        if (approxCone != null) {
            Imgproc.drawContours(inputroi,
                    Collections.singletonList(approxCone),
                    -1,
                    DetectionUtils.BLUE
            );

            Point center = DetectionUtils.getContourCenter(approxCone);
            if (center != null) {
                double middle = center.x;
                Imgproc.line(input,
                        new Point(middle, 0),
                        new Point(middle, input.height()),
                        DetectionUtils.RED
                );

                error =
                        (lo_offset < middle && middle < hi_offset) ? 0 : input.width() / 2.0 - middle;
            }
        } else {
            error = Double.NaN;
        }

        if (telemetry != null) {
            telemetry.addData("cone error", error);
//            telemetry.update();
        }
        inputroi.release();
        return input;
    }

    @Nullable
    private MatOfPoint approximateMaxHull() {
        List<MatOfPoint> mint = DetectionUtils.getConvexHulls(contours);
        if (mint.size() == 0) return null;

        MatOfPoint maxAreaContour =
                Collections.max(mint, Comparator.comparing(Imgproc::contourArea));
        if (Imgproc.contourArea(maxAreaContour) < 1500) return null;

        MatOfPoint2f matAs2f = DetectionUtils.matToMat2f(maxAreaContour);
        MatOfPoint2f approx = new MatOfPoint2f();
        Imgproc.approxPolyDP(matAs2f, approx, 0.05 * Imgproc.arcLength(matAs2f, true), true);
        return DetectionUtils.mat2fToMat(approx);
    }
}
