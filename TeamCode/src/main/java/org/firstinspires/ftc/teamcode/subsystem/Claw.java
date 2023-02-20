package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Claw {
    static private Servo clawLeft;
    static private Servo clawRight;
    private NormalizedColorSensor distanceSensor;

    // left
    final double POS_GRAB_ONE = 0.2;
    final double POS_OPEN_ONE = 0.35;

    // right
    final double POS_GRAB_TWO = 0.6;
    final double POS_OPEN_TWO = 0.7;

    public static double goingTo;
    public static double goingToTwo;

    /**
     * Initializes the Claw class
     *
     * @param hardwareMap The hardwareMap of your opmode
     */
    public Claw(HardwareMap hardwareMap) {
        clawLeft = hardwareMap.servo.get("leftServo");
        clawRight = hardwareMap.servo.get("rightServo");
        distanceSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        clawLeft.setDirection(Servo.Direction.FORWARD);
        clawRight.setDirection(Servo.Direction.REVERSE);

        goingTo = POS_GRAB_ONE;
        goingToTwo = POS_GRAB_TWO;
    }

    /**
     * Sets the target position
     *
     * @param increment The increment that the claw should go to<br>
     *                  0 - Down <br>
     *                  1 - Low pole <br>
     */
    public void setGoingTo(int increment) {
        switch (increment) {
            case 0:
                goingTo = POS_GRAB_ONE;
                goingToTwo = POS_GRAB_TWO;
                break;
            case 1:
                goingTo = POS_OPEN_ONE;
                goingToTwo = POS_OPEN_TWO;
                break;
        }
    }

    public double distanceCentimeters() {
        return ((DistanceSensor) distanceSensor).getDistance(DistanceUnit.CM);
    }

    /**
     * Updates the claw power and position, call on each loop
     */
    public void update() {
        clawLeft.setPosition(goingTo);
        clawRight.setPosition(goingToTwo);
    }
}
