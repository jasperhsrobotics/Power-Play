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


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp One Controller")
public class TeleOpOneController extends OpMode {
    Lift lift;
    Chassis chassis;
    Claw claw;

    @Override
    public void init() {
        claw = new Claw(hardwareMap);
        lift = new Lift(hardwareMap);
        chassis = new Chassis(hardwareMap, 0.6);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            lift.setManual(true);
        } else if (gamepad1.dpad_down) {
            lift.setManual(false);
        }

        if (gamepad1.a) {
            lift.setGoingTo(0);
        } else if (gamepad1.b) {
            lift.setGoingTo(1);
        } else if (gamepad1.x) {
            lift.setGoingTo(2);
        } else if (gamepad1.y) {
            lift.setGoingTo(3);
        }

        // 0 is inner
        if(gamepad1.dpad_right) {
            claw.setGoingTo(0);
        } else if (gamepad1.dpad_left) {
            claw.setGoingTo(1);
        }

        lift.update(gamepad2.left_stick_y);
        chassis.update(-gamepad1.left_stick_y, -gamepad1.left_stick_x * 1.1, gamepad1.right_stick_x);
        claw.update();

        //telemetry.addData("power:", lift.seePID());
    }
}
