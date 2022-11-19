package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class ConeTest extends LinearOpMode {
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence test = drive.trajectorySequenceBuilder(new Pose2d(-38, 60, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-34, 38, Math.toRadians(50)))
                .strafeTo(new Vector2d(-34, 12))
                .lineToLinearHeading(new Pose2d(-52, 12, Math.toRadians(180)))
                .build();


        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(test);
        }

    }
}
