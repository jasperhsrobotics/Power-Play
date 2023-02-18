package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    static private DcMotor lift;

    final int POS_DOWN = 50;
    final int POS_LOW = 1136;
    final int POS_MID = 2183;
    final int POS_HIGH = 3028;

    static int goingTo;
    static boolean manual;

    /**
     * Initializes the Lift class
     * @param hardwareMap The hardwareMap of your OpMode
     */
    public Lift(HardwareMap hardwareMap) {
        lift = hardwareMap.dcMotor.get("linearSlide");

        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.reset();
    }

    /**
     * Sets the starting position of the lift to the current position; Use sparingly
     */
    public void reset() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goingTo = POS_DOWN;
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets the mode of the lift
     * @param manual Whether the lift should move manually or not <br>
     *               true - move by controller <br>
     *               false - move by setting position
     */
    public void setManual(boolean manual) {
        Lift.manual = manual;
    }

    /**
     * Sets the target position of the lift to a preset position
     * @param increment The increment that the lift should go to <br>
     *                  0 - Down <br>
     *                  1 - Low pole <br>
     *                  2 - Medium pole <br>
     *                  3 - High pole <br>
     */
    public void setGoingTo(int increment) {
        switch (increment) {
            case 0:
                goingTo = POS_DOWN;
                break;
            case 1:
                goingTo = POS_LOW;
                break;
            case 2:
                goingTo = POS_MID;
                break;
            case 3:
                goingTo = POS_HIGH;
                break;
        }
    }

    /**
     * Sets the target position of the lift to a custom position; use sparingly
     * @param goingTo Sets the target to a specific position
     */
    public void setGoingToSpecific(int goingTo) {
        Lift.goingTo = goingTo;
    }

    /**
     * Calculates the power that should be applied to the lift in manual mode
     * @param stickVal value returned by the joystick <br>
     * @return the calculated power
     */
    double calculatePowerManual(double stickVal) {
        if(Math.abs(stickVal) > 0.1 && lift.getCurrentPosition() >= POS_DOWN) {
            return -stickVal;
        } else if (lift.getCurrentPosition() < POS_DOWN) {
            return 0.2;
        } else {
            return 0.01;
        }
    }

    /**
     * Calculates the power that should be applied to the lift in automatic mode
     * @return the calculated power
     */
    double calculatePowerAuto() {
        /*if (lift.getCurrentPosition() < 10 && goingTo == 0) {
            return 0;
        }*/

        // change when the lift is rebuilt
        if (Math.abs(goingTo - lift.getCurrentPosition()) < 15) {
            return 0.01;
        } else if (Math.abs(goingTo - lift.getCurrentPosition()) < 100) {
            if (goingTo < lift.getCurrentPosition()) {
                return -0.3;
            } else {
                return 0.3;
            }
        } else {
            // active lift motion
            // note: might continue without stopping if ping gets too high
            if (goingTo < lift.getCurrentPosition()) {
                return -0.8;
            } else {
                return 0.9;
            }
        }
    }

    public int getPosition() {
        return lift.getCurrentPosition();
    }

    /**
     * Updates the lift power and position, call on each loop
     * @param stickVal the current value of the stick that moves the lift if in manual mode
     */
    public void update(double stickVal) {
        goingTo = Math.max(POS_DOWN, goingTo);
        if (manual) {
            lift.setPower(calculatePowerManual(stickVal));
            goingTo = lift.getCurrentPosition();
        } else {
            lift.setPower(calculatePowerAuto());
        }
    }
}
