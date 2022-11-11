package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Motor Test")
public class MotorTest extends OpMode {

    DcMotor linearSlide;

    double power;

    @Override
    public void init() {
        linearSlide = hardwareMap.dcMotor.get("linearSlide"); // slkdjflkjsdflkjslfkdj
        linearSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        power = 0.5;
    }

    @Override
    public void loop() {
        setMotorPower(power);
    }

    public void setMotorPower(double power) {
        double slidePower = gamepad1.left_stick_y;

        linearSlide.setPower(slidePower);
    }
}
