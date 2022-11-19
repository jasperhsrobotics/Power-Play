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

    static int goingTo;
    static boolean manual;

    public Lift(HardwareMap hardwareMap) {
        lift = hardwareMap.dcMotor.get("linearSlide");

        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void reset() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goingTo = POS_DOWN;
        manual = false;
    }

    public void setManual(boolean manual) {
        this.manual = manual;
    }

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

    double calculatePowerManual(double stickVal) {
        if(Math.abs(stickVal) > 0.1) {
            return -stickVal * 0.7;
        } else {
            return 0.01;
        }
    }

    double calculatePowerAuto() {
        if (Math.abs(goingTo - lift.getCurrentPosition()) < 20) {
            return 0.1;
        } else {
            // active lift motion
            // note: might continue without stopping if ping gets too high
            if (goingTo < lift.getCurrentPosition()) {
                return -0.5;
            } else {
                return 0.5;
            }
        }
    }

    public void update(double stickVal) {
        if (manual) {
            lift.setPower(calculatePowerManual(stickVal));
            goingTo = lift.getCurrentPosition();
        } else {
            lift.setPower(calculatePowerAuto());
        }
    }
}
