package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    static private DcMotor lift;

    final int POS_DOWN = 15;
    final int POS_LOW = 1900;
    final int POS_MID = 3200;
    final int POS_HIGH = 4500;

    final int OFFSET = 50;

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
    }

    /**
     * Sets the starting position of the lift to the current position; Use sparingly
     */
    public void reset() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goingTo = POS_DOWN;
        manual = false;
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
        goingTo += OFFSET;
    }

    /**
     * Sets the target position of the lift to a custom position; use sparingly
     * @param goingTo Sets the target to a specific position
     */
    public void setGoingToSpecific(int goingTo) {
        this.goingTo = goingTo;
    }

    /**
     * Calculates the power that should be applied to the lift in manual mode
     * @param stickVal value returned by the joystick <br>
     * @return the calculated power
     */
    double calculatePowerManual(double stickVal) {
        if(Math.abs(stickVal) > 0.1) {
            return -stickVal * 0.7;
        } else {
            return 0.01;
        }
    }

    /**
     * Calculates the power that should be applied to the lift in automatic mode
     * @return the calculated power
     */
    double calculatePowerAuto() {
        // change when the lift is rebuilt
        if (Math.abs(goingTo - lift.getCurrentPosition()) < 20) {
            return 0.1;
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

    /**
     * Updates the lift power and position, call on each loop
     * @param stickVal the current value of the stick that moves the lift if in manual mode
     */
    public void update(double stickVal) {
        if (manual) {
            lift.setPower(calculatePowerManual(stickVal));
            goingTo = lift.getCurrentPosition();
        } else {
            lift.setPower(calculatePowerAuto());
        }
    }
}
