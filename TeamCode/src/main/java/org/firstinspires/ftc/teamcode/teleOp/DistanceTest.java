package org.firstinspires.ftc.teamcode.teleOp;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.Chassis;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Lift;

@TeleOp(name = "Sensor: Color", group = "Sensor")
public class DistanceTest extends OpMode {
    Lift lift;
    Chassis chassis;
    Claw claw;
    boolean gotCone = false;

    @Override
    public void init() {
        claw = new Claw(hardwareMap);
        lift = new Lift(hardwareMap);
        chassis = new Chassis(hardwareMap, 0.2);
    }

    @Override
    public void loop() {
        // 0 is inner
        if (claw.distanceCentimeters() < 1.5) {
            claw.setGoingTo(0);
            gotCone = true;
        } else {
            claw.setGoingTo(1);
            gotCone = false;
        }

        if (!gotCone) {
            chassis.updateField(-1, 0, 0);
        } else {
            chassis.updateField(0, 0, 0);
        }


        claw.update();

        telemetry.addData("power:", lift.getPosition());
    }
}