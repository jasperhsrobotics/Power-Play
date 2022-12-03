package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    static private Servo claw;

    final double POS_GRAB = 0.55;
    final double POS_OPEN = 0.39;

    public static double goingTo;

    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.servo.get("claw");
        claw.setDirection(Servo.Direction.FORWARD);
        goingTo = POS_GRAB;
    }

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

    public void update() {
        claw.setPosition(goingTo);
    }
}
