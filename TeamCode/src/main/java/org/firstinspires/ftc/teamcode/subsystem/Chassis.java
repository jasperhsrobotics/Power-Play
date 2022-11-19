package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Chassis {
    SampleMecanumDrive drive;
    double speed;

    public Chassis(HardwareMap hardwareMap, double speed) {
        drive = new SampleMecanumDrive(hardwareMap);
        this.speed = speed;
    }

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

    public void update(double y, double x, double rx) {
        double[] powers = calculatePower(y, x, rx);
        drive.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
    }
}
