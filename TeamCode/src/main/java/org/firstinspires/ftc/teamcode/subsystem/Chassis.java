package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Chassis {
    SampleMecanumDrive drive;
    double speed;

    /**
     * Initializes the Chassis class
     * @param hardwareMap The hardwareMap of your opmode
     * @param speed The speed that the robot should move at, [-1, 1]
     */
    public Chassis(HardwareMap hardwareMap, double speed) {
        drive = new SampleMecanumDrive(hardwareMap);
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

    /**
     * Updates the power of each motor
     * @param y The translational joystick value in the y-direction
     * @param x The translational joystick value in the x-direction
     * @param rx The heading joystick value in the x-direction
     */
    public void update(double y, double x, double rx) {
        double[] powers = calculatePower(y, x, rx);
        drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
    }
}
