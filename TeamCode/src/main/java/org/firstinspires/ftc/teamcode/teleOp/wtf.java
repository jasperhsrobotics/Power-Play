package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.PID;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.ConeDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.ConePolePipeline;
import org.firstinspires.ftc.teamcode.vision.PoleDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class wtf extends LinearOpMode {
    private final Gamepad gp1 = new Gamepad();
    private final Gamepad gp2 = new Gamepad();
    private final Gamepad prevGp1 = new Gamepad();
    private final Gamepad prevGp2 = new Gamepad();

    private final ConeDetectionPipeline coneDetector = ConeDetectionPipeline.redConeDetector();
    private final PoleDetectionPipeline poleDetector = new PoleDetectionPipeline();
    private ConePolePipeline conePolePipeline = new ConePolePipeline(telemetry);

    private final PID turnControl = new PID(0.001, 0, 0);
    public static double kP = 0.001;
    public static double kI = 0.0;
    public static double kD = 0.0;

    private boolean coneAiming = false;
    private boolean poleAiming = false;

    private SampleMecanumDrive drive;
    private Lift lift;
    private Claw claw;
    private OpenCvCamera camera;

    public static double FAST = 1.0;
    public static double NORMAL = 0.8;
    public static double SLOW = 0.5;
    private double driveSpeed = NORMAL;

    private boolean clawIsOpened = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardwareMap);
        claw = new Claw(hardwareMap);

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        FtcDashboard.getInstance().startCameraStream(camera, 30.0);
        camera.setPipeline(conePolePipeline);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            try {
                gp1.copy(gamepad1);
                gp2.copy(gamepad2);
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }

            if (gp1.left_trigger != 0) driveSpeed = FAST;
            else if (gp1.right_trigger != 0) driveSpeed = SLOW;
            else driveSpeed = NORMAL;

            if (gp1.dpad_left && !prevGp1.dpad_left) coneAiming = !coneAiming;
            if (coneAiming && coneDetector.getDetected()) {
                gp1.right_stick_x = (float) turnControl.calculate(0.0, conePolePipeline.coneError);
            }

            if (gp1.dpad_right && !prevGp1.dpad_right) poleAiming = !poleAiming;
            if (poleAiming && poleDetector.getDetected() && !coneAiming) {
                gp1.right_stick_x = (float) turnControl.calculate(0.0, conePolePipeline.poleError);
            }

            telemetry.addData("rsx", gp1.right_stick_x);

            drive.setWeightedDrivePower(new Pose2d(
                    new Vector2d(
                            -gp1.left_stick_y * driveSpeed,
                            -gp1.left_stick_x * driveSpeed
                    ).rotated(-drive.getRawExternalHeading() + Math.PI),
                    -gp1.right_stick_x * driveSpeed
            ));
            drive.update();

            if (gp1.a && !prevGp1.a) lift.setGoingTo(0);
            else if (gp1.b && !prevGp1.b) lift.setGoingTo(1);
            else if (gp1.x && !prevGp1.x) lift.setGoingTo(2);
            else if (gp1.y && !prevGp1.y) lift.setGoingTo(3);
            lift.update(0);

            if (gp1.left_bumper && !prevGp1.left_bumper) {
                clawIsOpened = !clawIsOpened;
                if (clawIsOpened) claw.setGoingTo(1);
                else claw.setGoingTo(0);
            }
            claw.update();

            try {
                prevGp1.copy(gp1);
                prevGp2.copy(gp2);
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }
            turnControl.Kp = kP;
            turnControl.Ki = kI;
            turnControl.Kd = kD;
            telemetry.update();
        }
    }
}
