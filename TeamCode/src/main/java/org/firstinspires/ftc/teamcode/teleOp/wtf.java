package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp
public class wtf extends LinearOpMode {
    private static final double CLAW_GRAB = 0.48;
    private static final double CLAW_OPEN = 0.39;

    private static final int POS_DOWN = 15;
    private static final int POS_LOW = 1900;
    private static final int POS_MID = 3200;
    private static final int POS_HIGH = 4500;

    private final Gamepad prevGp2 = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor lift = (DcMotor) hardwareMap.get("linearSlide");
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo claw = (Servo) hardwareMap.get("claw");
        claw.setDirection(Servo.Direction.FORWARD);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();

            int target = 0;
            if (gamepad2.a && !prevGp2.a) target = POS_DOWN;
            else if (gamepad2.b && !prevGp2.b) target = POS_LOW;
            else if (gamepad2.x && !prevGp2.x) target = POS_MID;
            else if (gamepad2.y && !prevGp2.y) target = POS_HIGH;
            lift.setTargetPosition(target);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(0.6);

            if (gamepad2.left_bumper) {
                if (claw.getPosition() == CLAW_GRAB) claw.setPosition(CLAW_OPEN);
                else claw.setPosition(CLAW_GRAB);
            }
            try {
                prevGp2.copy(gamepad2);
            } catch (RobotCoreException ignored) {
            }
            telemetry.update();
        }
    }
}
