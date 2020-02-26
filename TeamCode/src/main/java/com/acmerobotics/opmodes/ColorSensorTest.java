package com.acmerobotics.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.TheColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="ColorSensorTest")
@Config
public class ColorSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        TheColorSensor colorSensor = new TheColorSensor(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();

        while(!isStopRequested()){

            colorSensor.HSV();

            dashboardTelemetry.addData("hue", colorSensor.hsvValues[0]);
            dashboardTelemetry.addData("skytoneHue", TheColorSensor.skystoneHue);
            dashboardTelemetry.addData("skystone found", colorSensor.isSkystoneHue());

            dashboardTelemetry.update();

            telemetry.addData("skystone found", colorSensor.isSkystoneHue());

            telemetry.update();


        }
    }
}
