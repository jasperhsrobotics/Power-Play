package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
public class AutoAsync extends LinearOpMode {
    SampleMecanumDrive drive;
    Claw claw;
    Lift lift;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        claw = new Claw(hardwareMap);
        lift = new Lift(hardwareMap);
        lift.setManual(false);
        lift.reset();


        claw.update();

        //lift = new Lift(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-34, 62, Math.toRadians(270)));

        TrajectorySequence ree = drive.trajectorySequenceBuilder(new Pose2d(-34, 62, Math.toRadians(270)))
                .waitSeconds(0.5)
                // Preload
                .lineTo(new Vector2d(-34, 60))
                .addDisplacementMarker(() -> {
                    lift.setGoingTo(1);
                })
                .lineTo(new Vector2d(-34, 52))
                .turn(Math.toRadians(-90))
                .strafeTo(new Vector2d(-34, 22))
                .forward(7)
                .waitSeconds(0.1)
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
                    claw.setGoingTo(1);
                })
                .lineTo(new Vector2d(-63.8, 12))
                .waitSeconds(0.2)
                .addDisplacementMarker(() -> {
                    claw.setGoingTo(0);
                })
                .waitSeconds(0.7)
                .addDisplacementMarker(() -> {
                    lift.setGoingTo(3);
                })
                .waitSeconds(0.5)
                // Drop first cone
                .lineTo(new Vector2d(-13, 12))
                .turn(Math.toRadians(45))
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
                .waitSeconds(0.7)
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


                .build();

        waitForStart();

        if (opModeIsActive()) {
            claw.setGoingTo(0);

            drive.followTrajectorySequenceAsync(ree);

            while (opModeIsActive()) {
                drive.update();
                lift.update(0);
                claw.update();
            }
        }
    }
}
