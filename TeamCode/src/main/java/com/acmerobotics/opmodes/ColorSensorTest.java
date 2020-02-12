package com.acmerobotics.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.TheColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="ColorSensorTest")
@Config
public class ColorSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        TheColorSensor sensor = new TheColorSensor(hardwareMap);

        waitForStart();

        while(!isStopRequested()){

            sensor.HSV();
            telemetry.addData("hue", sensor.hsvValues[0]);
            telemetry.addData("sat.", sensor.hsvValues[1]);
            telemetry.addData("val.", sensor.hsvValues[2]);

            telemetry.addLine();

            telemetry.addData("red", sensor.RED());
            telemetry.addData("green", sensor.GREEN());
            telemetry.addData("blue", sensor.BLUE());
            telemetry.addData("hue", sensor.hue());

            telemetry.addLine();

            telemetry.addData("hue function", sensor.isSkystoneHue());
            telemetry.addData("sat function", sensor.isSkystoneSat());

            telemetry.update();
        }
    }
}
