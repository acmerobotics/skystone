package com.acmerobotics.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.TheColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="ColorSensorTest")
@Config
public class ColorSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        TheColorSensor sensor = new TheColorSensor(hardwareMap);


        waitForStart();

        while(!isStopRequested()){

            sensor.HSV();
            telemetry.addData("HSV direct", sensor.hsvValues);

            telemetry.addData("HSV", sensor.HSV());

            telemetry.addData("red", sensor.RED());
            telemetry.addData("green", sensor.GREEN());
            telemetry.addData("blue", sensor.BLUE());

            telemetry.update();
        }
    }
}
