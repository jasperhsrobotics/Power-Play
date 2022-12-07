package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class CycleAutonomousPark extends LinearOpMode {
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


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
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

        drive.setPoseEstimate(new Pose2d(-34, 62, Math.toRadians(270)));

        finished = false;
        TrajectorySequence ree = drive.trajectorySequenceBuilder(new Pose2d(-34, 62, Math.toRadians(270)))
                .waitSeconds(0.5)
                // Preload
                .lineTo(new Vector2d(-34, 60))
                .addDisplacementMarker(() -> {
                    lift.setGoingTo(1);
                })
                .lineToLinearHeading(new Pose2d(-34, 22, Math.toRadians(-180)))
                //.lineTo(new Vector2d(-34, 52))
                //.turn(Math.toRadians(-90))
                //.strafeTo(new Vector2d(-34, 22))
                .strafeLeft(15)
                .strafeRight(15)
                .forward(7)
                .waitSeconds(0.1)
                // Drops preset cone
                .addDisplacementMarker(() -> {
                    claw.setGoingTo(1);
                })
                .waitSeconds(0.1)
                .addDisplacementMarker(() -> {
                    lift.setGoingToSpecific(480);
                })
                .strafeTo(new Vector2d(-34, 12))

                // Get Cone 1
                .addDisplacementMarker(() -> {
                    // open claw
                    claw.setGoingTo(1);
                })
                .lineTo(new Vector2d(-63.8, 12))
                .waitSeconds(0.2)
                .addDisplacementMarker(() -> {
                    claw.setGoingTo(0);
                })
                .waitSeconds(0.4)
                .addDisplacementMarker(() -> {
                    lift.setGoingTo(3);
                })
                .waitSeconds(0.5)
                // Drop first cone
                .lineTo(new Vector2d(-13, 12))
//                .turn(Math.toRadians(45))
                .lineToLinearHeading(new Pose2d(-13, 12, Math.toRadians(225)))
                .lineTo(new Vector2d(-18, 5))
                .addDisplacementMarker(() -> {
                    claw.setGoingTo(1);
                })
                .forward(-5)
                .turn(Math.toRadians(-45))

                // Get Cone 2
                .addDisplacementMarker(() -> {
                    claw.setGoingTo(1);
                    lift.setGoingToSpecific(450);
                })
                .lineTo(new Vector2d(-63.8, 12))
                .waitSeconds(0.2)
                .addDisplacementMarker(() -> {
                    claw.setGoingTo(0);
                })
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    lift.setGoingTo(3);
                })
                .waitSeconds(0.5)

                // Drop second cone
                .lineTo(new Vector2d(-13, 12))
                .turn(Math.toRadians(45))
                .lineTo(new Vector2d(-18, 5))
                .addDisplacementMarker(() -> {
                    claw.setGoingTo(1);
                })
                .forward(-5)
                .turn(Math.toRadians(-45))

                .strafeRight(28)

                .addDisplacementMarker(() -> {
                    finished = true;
                })

                .build();

        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(ree.end())
                .forward(24)
                .build();

        TrajectorySequence moveForward2 = drive.trajectorySequenceBuilder(moveForward.end())
                .forward(24)
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

            claw.setGoingTo(0);
            claw.update();
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

        claw.setGoingTo(0);

        drive.followTrajectorySequenceAsync(ree);


        while (opModeIsActive()) {
            drive.update();
            lift.update(0);
            claw.update();
            if (!drive.isBusy()) break;
            //if (finished == true) break;
        }

        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {

        } else if (tagOfInterest.id == MIDDLE) {
            drive.followTrajectorySequence(moveForward);
        } else {
            drive.followTrajectorySequence(moveForward);
            drive.followTrajectorySequence(moveForward2);
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