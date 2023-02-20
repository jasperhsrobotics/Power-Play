package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Chassis {
    SampleMecanumDrive drive;
    public double speed;

    /**
     * Initializes the Chassis class
     * @param hardwareMap The hardwareMap of your opmode
     * @param speed The speed that the robot should move at, [-1, 1]
     */
    public Chassis(HardwareMap hardwareMap, double speed) {
        drive = new SampleMecanumDrive(hardwareMap);
        //drive.setPoseEstimate(PoseStorage.currentPose);
        this.speed = speed;
    }

    /**
     * Calculates the power of each wheel
     * @param y The translational joystick value in the y-direction
     * @param x The translational joystick value in the x-direction
     * @param rx The heading joystick value in the x-direction
     * @return An array of powers that correspond to the motor powers
     */
    double[] calculatePower(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y - x + rx) / denominator;
        double backLeftPower = (y + x + rx) / denominator;
        double frontRightPower = (y + x - rx) / denominator;
        double backRightPower = (y - x - rx) / denominator;

        double[] powers = {frontLeftPower, backLeftPower, backRightPower, frontRightPower};

        for (int i=0; i<4; i++)
            powers[i] = powers[i] * speed;

        return powers;
    }

    public void updateField(double y, double x, double rx) {
        Pose2d poseEstimate = drive.getPoseEstimate();

        Vector2d input = new Vector2d(
                -y * speed,
                x * speed
        ).rotated(-poseEstimate.getHeading());

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        rx
                )
        );

        drive.update();
        drive.updatePoseEstimate();
    }

    public void resetHeading() {
        PoseStorage.currentPose = new Pose2d();
    }

    /**
     * Updates the power of each motor
     * @param y The translational joystick value in the y-direction
     * @param x The translational joystick value in the x-direction
     * @param rx The heading joystick value in the x-direction
     */
    public void update(double y, double x, double rx) {
        double[] powers = calculatePower(y, x, rx);
        drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
        drive.updatePoseEstimate();
    }
}
