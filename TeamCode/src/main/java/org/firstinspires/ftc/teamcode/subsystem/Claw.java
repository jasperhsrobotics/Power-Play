package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    static private Servo claw;

    final double POS_OPEN = 0.59;
    final double POS_GRAB = 0.79;

    public static double goingTo;

    /**
     * Initializes the Claw class
     * @param hardwareMap The hardwareMap of your opmode
     */
    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.servo.get("claw");
        claw.setDirection(Servo.Direction.FORWARD);
        goingTo = POS_GRAB;
    }

    /**
     * Sets the target position
     * @param increment The increment that the claw should go to<br>
     *                  0 - Down <br>
     *                  1 - Low pole <br>
     */
    public void setGoingTo(int increment) {
        switch (increment) {
            case 0:
                goingTo = POS_GRAB;
                break;
            case 1:
                goingTo = POS_OPEN;
                break;
        }
    }

    /**
     * Updates the claw power and position, call on each loop
     */
    public void update() {
        claw.setPosition(goingTo);
    }
}
