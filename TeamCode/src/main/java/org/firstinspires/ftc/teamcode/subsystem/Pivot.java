package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Pivot {
    static private Servo pivotServo;

    public static double POS_RIGHT = 0.15;
    public static double POS_MIDDLE = 0.45;
    public static double POS_LEFT = 0.8;

    static double goingTo;

    /**
     * Initializes the Pivot class
     *
     * @param hardwareMap The hardwareMap of your OpMode
     */
    public Pivot(HardwareMap hardwareMap) {
        pivotServo = hardwareMap.servo.get("pivotServo");
    }

    /**
     * Sets the target position of the lift to a preset position
     *
     * @param increment The increment that the lift should go to <br>
     *                  0 - Down <br>
     *                  1 - Low pole <br>
     *                  2 - Medium pole <br>
     *                  3 - High pole <br>
     */
    public void setGoingTo(int increment) {
        switch (increment) {
            case 0:
                goingTo = POS_LEFT;
                break;
            case 1:
                goingTo = POS_MIDDLE;
                break;
            case 2:
                goingTo = POS_RIGHT;
                break;
        }
    }

    public void update() {
        pivotServo.setPosition(goingTo);
    }
}
