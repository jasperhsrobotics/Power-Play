package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive")
public class DriveTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // init =========================
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory strafeL25 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(25)
                .build();
        Trajectory strafeR25 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(-25)
                .build();
        Trajectory forward30A = drive.trajectoryBuilder(new Pose2d())
                .forward(25)
                .build();
        Trajectory forward30B = drive.trajectoryBuilder(strafeR25.end())
                .forward(25)
                .build();

        waitForStart();
        // start =========================

        if (isStopRequested()) return;

        //drive.followTrajectory(strafeR25);
        //drive.followTrajectory(forward30);

        Pose2d poseEstimate = drive.getPoseEstimate();

        while (!isStopRequested() && opModeIsActive());
    }
}
