package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.Chassis;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.LiftPID;
import org.firstinspires.ftc.teamcode.subsystem.Pivot;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Claw Test")
public class ClawTest extends OpMode {
    Claw claw;

    @Override
    public void init() {
        claw = new Claw(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_left) {
            claw.setGoingTo(0);
        } else if (gamepad1.dpad_right) {
            claw.setGoingTo(1);
        }
        claw.update();

        //telemetry.addData("power:", lift.seePID());
    }
}
