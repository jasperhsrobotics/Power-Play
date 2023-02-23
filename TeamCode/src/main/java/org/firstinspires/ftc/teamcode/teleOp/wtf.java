//package org.firstinspires.ftc.teamcode.teleOp;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.exception.RobotCoreException;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.subsystem.Claw;
//import org.firstinspires.ftc.teamcode.subsystem.Lift;
//import org.firstinspires.ftc.teamcode.subsystem.PID;
//import org.firstinspires.ftc.teamcode.vision.ConeDetectionPipeline;
//import org.firstinspires.ftc.teamcode.vision.PoleDetectionPipeline;
//
//// dont forgor to change ds font size to small
//@TeleOp(name = "🟦🟦🟦🟦🟦🟦🟦🟦🟦⬛⬛⬛⬛⬛🟦🟦🟦🟦🟦🟦🟦\n" + "🟦🟦🟦🟦🟦🟦🟦🟦⬛⬛🟥🟥🟥⬛⬛⬛🟦🟦🟦🟦🟦\n" + "🟦🟦🟦🟦🟦🟦🟦⬛⬛🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦🟦\n" + "🟦🟦🟦🟦🟦🟦🟦⬛🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦🟦\n" + "🟦🟦🟦🟦🟦🟦🟦⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" + "🟦🟦🟦🟦🟦🟦🟦⬛🟥🟥🟥⬜⬜⬜⬜⬜⬜⬛⬛🟦🟦\n" + "🟦🟦🟦🟦⬛⬛⬛🟥🟥🟥⬜⬜⬜⬜⬜⬜⬜⬜⬛🟦🟦\n" + "🟦🟦🟦⬛🟥⬛⬛🟥🟥🟥⬜⬜⬜⬜⬜⬜⬜⬜⬛🟦🟦\n" + "🟦🟦🟦⬛🟥⬛⬛🟥🟥🟥⬛⬜⬜⬜⬜⬜⬜⬜⬛🟦🟦\n" + "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥⬛⬛⬛⬛⬛⬛⬛🟦🟦🟦\n" + "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" + "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" + "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" + "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" + "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" + "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" + "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" + "🟦🟦🟦⬛⬛⬛⬛🟥🟥🟥🟥⬛⬛🟥🟥🟥🟥⬛🟦🟦🟦\n" + "🟦🟦🟦🟦🟦⬛⬛🟥🟥🟥🟥⬛⬛🟥🟥🟥🟥⬛🟦🟦🟦\n" + "🟦🟦🟦🟦🟦⬛⬛⬛🟥🟥⬛⬛⬛🟥🟥🟥⬛🟦🟦🟦🟦\n" + "🟦🟦🟦🟦🟦🟦🟦⬛⬛⬛⬛🟦⬛⬛⬛⬛⬛🟦🟦🟦🟦")
//public class wtf extends LinearOpMode {
//    private final Gamepad gp1 = new Gamepad();
//    private final Gamepad gp2 = new Gamepad();
//    private final Gamepad prevGp1 = new Gamepad();
//    private final Gamepad prevGp2 = new Gamepad();
//
//    private final ConeDetectionPipeline coneDetector = ConeDetectionPipeline.redConeDetector();
//    private final PoleDetectionPipeline poleDetector = new PoleDetectionPipeline();
//
//    private final PID turnControl = new PID(0.001, 0, 0);
//    public static double kP = 0.001;
//    public static double kI = 0.0;
//    public static double kD = 0.0;
//
//    private boolean coneAiming = false;
//    private boolean poleAiming = false;
//
//    private SampleMecanumDrive drive;
//    private Servo claw;
//    private DcMotor lift;
//
//    public static double FAST = 1.0;
//    public static double NORMAL = 0.8;
//    public static double SLOW = 0.5;
//    private double driveSpeed = NORMAL;
//
//    private double clawPos = 0.0;
//    private int slidePos = 0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        lift = (DcMotor) hardwareMap.get("linearSlide");
//        lift.setDirection(DcMotorSimple.Direction.FORWARD);
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        claw = (Servo) hardwareMap.get("claw");
//        claw.setDirection(Servo.Direction.FORWARD);
//
//        drive = new SampleMecanumDrive(hardwareMap);
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//            try {
//                gp1.copy(gamepad1);
//                gp2.copy(gamepad2);
//            } catch (RobotCoreException e) {
//                e.printStackTrace();
//            }
//
//            if (gp1.left_trigger != 0) driveSpeed = FAST;
//            else if (gp1.right_trigger != 0) driveSpeed = SLOW;
//            else driveSpeed = NORMAL;
//
//            if (gp1.dpad_left && !prevGp1.dpad_left) coneAiming = !coneAiming;
//            if (coneAiming && coneDetector.getDetected()) {
//                gp1.right_stick_x = (float) turnControl.calculate(0.0, coneDetector.error);
//            }
//
//            if (gp1.dpad_right && !prevGp1.dpad_right) poleAiming = !poleAiming;
//            if (poleAiming && poleDetector.getDetected() && !coneAiming) {
//                gp1.right_stick_x = (float) turnControl.calculate(0.0, poleDetector.error);
//            }
//
//            drive.setWeightedDrivePower(new Pose2d(
//                    new Vector2d(
//                            -gp1.left_stick_y * driveSpeed,
//                            -gp1.left_stick_x * driveSpeed
//                    ).rotated(-drive.getRawExternalHeading() + Math.PI),
//                    -gp1.right_stick_x * driveSpeed
//            ));
//            drive.update();
//
//            boolean changed = true;
//            if (gp1.a && !prevGp1.a) slidePos = Lift.POS_DOWN;
//            else if (gp1.b && !prevGp1.b) slidePos = Lift.POS_LOW;
//            else if (gp1.x && !prevGp1.x) slidePos = Lift.POS_MID;
//            else if (gp1.y && !prevGp1.y) slidePos = Lift.POS_HIGH;
//            else changed = false;
//
//            if (changed) {
//                lift.setTargetPosition(slidePos);
//                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                lift.setPower(1.0);
//            }
//
//            if (gp1.left_bumper && !prevGp1.left_bumper) {
//                if (clawPos == Claw.POS_GRAB) claw.setPosition(Claw.POS_OPEN);
//                else claw.setPosition(Claw.POS_GRAB);
//                clawPos = Claw.POS_OPEN;
//            }
//
//            try {
//                prevGp1.copy(gp1);
//                prevGp2.copy(gp2);
//            } catch (RobotCoreException e) {
//                e.printStackTrace();
//            }
//            turnControl.Kp = kP;
//            turnControl.Ki = kI;
//            turnControl.Kd = kD;
//            telemetry.update();
//        }
//    }
//}
