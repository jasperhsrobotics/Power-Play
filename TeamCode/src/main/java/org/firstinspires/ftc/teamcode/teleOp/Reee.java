package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp One Controller")
public class Reee extends OpMode {

//    public enum LiftState {
//        START,
//        LOW, MID, HIGH
//,    }

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor lift;
    int liftPosition;
    double clawPosition;
//    LiftState liftState;
    int liftGoingTo;

    Servo claw;
    double claw_pos;
    static final double CLAW_GRAB = 0.48;
    static final double CLAW_OPEN = 0.39;

    static final int LIFT_DOWN = 0;
    static final int LIFT_LOW = 1800;
    static final int LIFT_MID = 3100;
    static final int LIFT_HIGH = 4350;

    BNO055IMU imu;

    double power;

    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        lift = hardwareMap.dcMotor.get("linearSlide"); // slkdjflkjsdflkjslfkdj
        claw = hardwareMap.servo.get("claw");

        power = 0.5;

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // TODO: fix this

        claw_pos = CLAW_OPEN;
        claw.setPosition(claw_pos);

        liftPosition = 0; // TODO: fix this
        clawPosition = 0;

//        liftState = LiftState.LOW;
        liftGoingTo = 0;
    }

    @Override
    public void loop() {
        /*if(gamepad1.dpad_up) {
            lift.setPower(1);
        } else if (gamepad1.dpad_down) {
            lift.setPower(-1);
        } else {
            lift.setPower(0.05);
        } */

        if (gamepad1.a) {
            liftGoingTo = LIFT_DOWN;
        } if (gamepad1.b) {
            liftGoingTo = LIFT_LOW;
        } if (gamepad1.x) {
            liftGoingTo = LIFT_MID;
        } if (gamepad1.y) {
            liftGoingTo = LIFT_HIGH;
        }

        if (Math.abs(liftGoingTo - lift.getCurrentPosition()) < 20) {
            // prevent lift from sliding down
            if(lift.getCurrentPosition() > 4000 && liftGoingTo == LIFT_HIGH) {
                // add more power cuz it slips at the top
                lift.setPower(0.03);
            } else {
                lift.setPower(0.01);
            }
        } else {
            // active lift motion
            // note: might continue without stopping if ping gets too high
            if (liftGoingTo < lift.getCurrentPosition()) {
                lift.setPower(-0.5);
            } else {
                lift.setPower(0.5);
            }
        }

        if(gamepad1.dpad_left) {
            claw_pos = CLAW_GRAB;
        } else if (gamepad1.dpad_right) {
            claw_pos = CLAW_OPEN;
        }

        setWheelPower(power);
        claw.setPosition(claw_pos);

        liftPosition = lift.getCurrentPosition();
        clawPosition = claw.getPosition(); // doesn't read the current position, just looks at what position the servo was last set at

        telemetry.addData("Lift Position: ", liftPosition);
        telemetry.addData("Claw Position: ", clawPosition);
    }

    public void setWheelPower(double power) {
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y - x + rx) / denominator;
        double backLeftPower = (y + x + rx) / denominator;
        double frontRightPower = (y + x - rx) / denominator;
        double backRightPower = (y - x - rx) / denominator;

        frontLeft.setPower(frontLeftPower * power);
        backLeft.setPower(backLeftPower * power);
        frontRight.setPower(frontRightPower * power);
        backRight.setPower(backRightPower * power);
    }
}
