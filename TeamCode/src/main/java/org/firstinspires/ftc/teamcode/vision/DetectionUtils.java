package org.firstinspires.ftc.teamcode.vision;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;


public class DetectionUtils {
    public static final Scalar LO_YELLOW = new Scalar(10.0, 125.0, 150.0);
    public static final Scalar HI_YELLOW = new Scalar(35.0, 255.0, 255.0);
    public static final Scalar GREEN = new Scalar(0.0, 255.0, 0.0, 255.0);
    public static final Scalar RED = new Scalar(255.0, 0.0, 0.0);
    public static final Scalar BLUE = new Scalar(0.0, 0.0, 255.0);


    @NotNull
    public static MatOfPoint getConvexHull(@NotNull MatOfPoint contour) {
        MatOfInt hullmat = new MatOfInt();
        Imgproc.convexHull(contour, hullmat);

        MatOfPoint hull = new MatOfPoint();
        hull.create(hullmat.rows(), 1, CvType.CV_32SC2);

        for (int i = 0; i < hullmat.rows(); i++)
            hull.put(i, 0, contour.get((int) hullmat.get(i, 0)[0], 0));
        return hull;
    }

    @NotNull
    public static List<MatOfPoint> getConvexHulls(@NotNull List<MatOfPoint> contours) {
        List<MatOfPoint> hulls = new ArrayList<>(contours.size());
        contours.forEach(it -> hulls.add(getConvexHull(it)));
        return hulls;
    }


    @NotNull
    public static MatOfPoint2f getApproximate(@NotNull MatOfPoint contour, double epilisonMultiplier) {
        MatOfPoint2f mat2f = matToMat2f(contour);
        double epilison = epilisonMultiplier * Imgproc.arcLength(mat2f, true);
        MatOfPoint2f approx = new MatOfPoint2f();
        Imgproc.approxPolyDP(mat2f, approx, epilison, true);
        return approx;
    }

    @NotNull
    public static MatOfPoint2f getApproximate(@NotNull MatOfPoint contour) {
        return getApproximate(contour, 0.01);
    }

    @NotNull
    public static ArrayList<MatOfPoint2f> getApproximates(@NotNull List<MatOfPoint> contours, double epilisonMultiplier) {
        ArrayList<MatOfPoint2f> approxs = new ArrayList<>(contours.size());
        contours.forEach(it -> approxs.add(getApproximate(it, epilisonMultiplier)));
        return approxs;
    }

    @NotNull
    public static ArrayList<MatOfPoint2f> getApproximates(@NotNull List<MatOfPoint> contours) {
        return getApproximates(contours, 0.01);
    }

    public static void putText(@NotNull Mat input, Point point, String text) {
        Imgproc.putText(input, text, point, Imgproc.FONT_HERSHEY_PLAIN, 1, new Scalar(255, 0, 0));
    }


    @Nullable
    public static Point getContourCenter(@NotNull MatOfPoint contour) {
        Moments moment = Imgproc.moments(contour);
        if (moment.m00 != 0.0)
            return new Point(moment.m10 / moment.m00, moment.m01 / moment.m00);
        return null;
    }

    @NotNull
    public static List<Point> getContourCenters(@NotNull List<MatOfPoint> contours) {
        List<Point> centers = new ArrayList<>();
        contours.forEach(it -> {
            Point center = getContourCenter(it);
            if (center != null) centers.add(center);
        });
        return centers;
    }

    public static void drawContourCenters(@NotNull Mat input, @NotNull List<MatOfPoint> contours) {
        getContourCenters(contours).forEach(it -> Imgproc.circle(input, it, 1, RED, 3));
    }


    @NotNull
    public static MatOfPoint2f matToMat2f(@NotNull MatOfPoint matOfPoint) {
        MatOfPoint2f matOfPoint2f = new MatOfPoint2f();
        matOfPoint2f.fromArray(matOfPoint.toArray());
        return matOfPoint2f;
    }

    @NotNull
    public static MatOfPoint mat2fToMat(@NotNull MatOfPoint2f matOfPoint2f) {
        MatOfPoint matOfPoint = new MatOfPoint();
        matOfPoint.fromArray(matOfPoint2f.toArray());
        return matOfPoint;
    }

    @NotNull
    public static List<MatOfPoint> mat2fToMatList(@NotNull List<MatOfPoint2f> matOfPoint2fs) {
        return matOfPoint2fs.stream().map(DetectionUtils::mat2fToMat)
                            .collect(Collectors.toList());
    }


    public static void filterByArea(@NotNull List<MatOfPoint> contours, double minArea) {
        filterByArea(contours, minArea, Double.MAX_VALUE);
    }

    public static void filterByArea(@NotNull List<MatOfPoint> contours, double minArea, double maxArea) {
        contours.removeIf(c -> {
            double area = Imgproc.contourArea(c);
            return !(minArea <= area && area <= maxArea);
        });
    }


    public static double getRotatedRectHorizontalDistance(@NotNull RotatedRect rect) {
        Point[] pts = new Point[4];
        rect.points(pts);
        return ((pts[1].x - pts[0].x) + (pts[2].x - pts[3].x)) / 2.0;
    }

    public static double areaOfParallelogram(@NotNull RotatedRect rect) {
        Point[] points = new Point[4];
        rect.points(points);
        Point lowLeft = points[0], topleft = points[1], topRight = points[2], lowRight = points[3];

        double width = lowRight.x - lowLeft.x;
        double height = lowLeft.y - topleft.y;
        return rect.boundingRect().area() - width * height;
    }

    @NotNull
    public static List<RotatedRect> getRotatedRects(@NotNull List<MatOfPoint2f> contours) {
        ArrayList<RotatedRect> rects = new ArrayList<>(contours.size());
        for (MatOfPoint2f contour : contours)
            rects.add(Imgproc.minAreaRect(contour));
        return rects;
    }

    public static void drawBoundingRects(@NotNull Mat input, @NotNull List<RotatedRect> rotatedRects) {
        rotatedRects.stream().map(RotatedRect::boundingRect)
                    .collect(Collectors.toList())
                    .forEach(it -> Imgproc.rectangle(input, it, GREEN));
    }

    public static void drawRectCenters(@NotNull Mat img, @NotNull List<Rect> rects) {
        rects.forEach(it -> {
            Point center = new Point(it.x + it.width / 2.0, it.y + it.height / 2.0);
            Imgproc.ellipse(img, center, new Size(it.width / 2.0, it.height / 2.0), 0, 0, 360, RED);
        });
    }
}
