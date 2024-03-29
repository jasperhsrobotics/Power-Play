package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AutoRight extends LinearOpMode {
    boolean finished = false;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS, WE MAY NEED TO ALTER TAG SIZE
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Claw claw;
        Lift lift;

        claw = new Claw(hardwareMap);
        lift = new Lift(hardwareMap);
        lift.setManual(false);
        lift.reset();
        lift.setGoingTo(0);
        claw.setGoingTo(0);


       camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        FtcDashboard.getInstance().startCameraStream(camera, 30.0);

        telemetry.setMsTransmissionInterval(50);



        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        drive.setPoseEstimate(new Pose2d(-36, 63, Math.toRadians(270)));

        finished = false;
        TrajectorySequence ree = drive.trajectorySequenceBuilder(new Pose2d(-36, 63, Math.toRadians(270)))
                .addTemporalMarker(() -> {
                    lift.setGoingTo(1);
                })
                .waitSeconds(0.5)

                // Drop preload
                .setTangent(Math.toRadians(0))
                .setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 30;
                    }
                })
                .splineToConstantHeading(new Vector2d(-9, 43), Math.toRadians(270))
                .addTemporalMarker(() -> {
                    lift.setGoingTo(3);
                })
                .resetVelConstraint()
                .splineToConstantHeading(new Vector2d(-9, 20), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-26, 13), Math.toRadians(270))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    lift.setGoingToSpecific(3000);
                    claw.setGoingTo(1);
                })

                // Get first cone
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-52, 14, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    lift.setGoingToSpecific(540);
                })
                .waitSeconds(1)
                .setTangent(Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(-65, 13), Math.toRadians(-180))
                .setTangent(Math.toRadians(90))
                .addTemporalMarker(() -> {
                    claw.setGoingTo(0);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    lift.setGoingTo(2);
                })
                .waitSeconds(0.5)

                // Drop first cone
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-53, 12, Math.toRadians(180)), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    lift.setGoingTo(3);
                })
                .splineToSplineHeading(new Pose2d(-26, 10, Math.toRadians(270)), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    lift.setGoingToSpecific(3000);
                    claw.setGoingTo(1);
                })

                // Get second cone
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-52, 11, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    lift.setGoingToSpecific(420);
                })
                .waitSeconds(1)
                .setTangent(Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(-64, 11), Math.toRadians(-180))
                .setTangent(Math.toRadians(90))
                .addTemporalMarker(() -> {
                    claw.setGoingTo(0);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    lift.setGoingTo(2);
                })
                .waitSeconds(0.5)

                // Drop second cone
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-53, 12, Math.toRadians(180)), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    lift.setGoingTo(3);
                })
                .splineToSplineHeading(new Pose2d(-23.4, 8.5, Math.toRadians(270)), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    lift.setGoingToSpecific(3000);
                    claw.setGoingTo(1);
                })
                .waitSeconds(0.5)


/*
//                // Get second cone
//                .waitSeconds(0.5)
//                .setTangent(Math.toRadians(80)) //90
//                .splineToSplineHeading(new Pose2d(-52, 13, Math.toRadians(180)), Math.toRadians(180))
//                .addTemporalMarker(() -> {
//                    lift.setGoingToSpecific(420);
//                })
//                .waitSeconds(1)
//                /*.addTemporalMarker(() -> {
//                    ElapsedTime time = new ElapsedTime();
//                    time.reset();
//                    time.startTime();
//                    while (claw.distanceCentimeters() > 3 && time.seconds() < 1.2) {
//                        telemetry.addData("Distance: ", claw.distanceCentimeters());
//                        telemetry.update();
//                        drive.setWeightedDrivePower(new Pose2d(
//                                0.3, 0, 0
//                        ));
//                    }
//                    time.reset();
//                    drive.setWeightedDrivePower(new Pose2d(
//                            0, 0, 0
//                    ));
//                    drive.setPoseEstimate(new Pose2d(
//                            -66, 13, Math.toRadians(-180)
//                    ));
//                    claw.setGoingTo(0);
//                })*/
//
//                .waitSeconds(1)
//                .addTemporalMarker(() -> {
//                    lift.setGoingTo(2);
//                })
//                .waitSeconds(0.5)
//
//                // Drop second cone
//                .setTangent(Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(-53, 13, Math.toRadians(180)), Math.toRadians(0))
//                .addTemporalMarker(() -> {
//                    lift.setGoingTo(3);
//                })
//                .splineToSplineHeading(new Pose2d(-26, 9, Math.toRadians(270)), Math.toRadians(270))
//                /*.addTemporalMarker(() -> {
//                    drive.setPoseEstimate(new Pose2d(-28, 11, Math.toRadians(270)));
//                })*/
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    lift.setGoingToSpecific(3000);
//                    claw.setGoingTo(1);
//                })

                .addTemporalMarker(() -> {
                    finished = true;
                })

                .build();

        TrajectorySequence strafeRight = drive.trajectorySequenceBuilder(ree.end())
                .back(3)
                .lineToSplineHeading(new Pose2d(-60, 14, Math.toRadians(270)))
                .build();

        TrajectorySequence strafeLeft = drive.trajectorySequenceBuilder(ree.end())
                .back(3)
                .lineToSplineHeading(new Pose2d(-14, 14, Math.toRadians(270)))
                .build();

        TrajectorySequence dontStrafe = drive.trajectorySequenceBuilder(ree.end())
                .back(3)
                .strafeRight(13)
                .build();

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            lift.update(0);
            claw.update();
            telemetry.addLine("Claw closed");
            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        drive.followTrajectorySequenceAsync(ree);


        while (opModeIsActive()) {
            drive.update();
            lift.update(0);
            claw.update();
            PoseStorage.currentPose = drive.getPoseEstimate();
            if (!drive.isBusy()) break;
        }


        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            drive.followTrajectorySequence(strafeLeft);
        } else if (tagOfInterest.id == MIDDLE) {
            drive.followTrajectorySequence(dontStrafe);
        } else {
            drive.followTrajectorySequence(strafeRight);
        }

//        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
//        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}