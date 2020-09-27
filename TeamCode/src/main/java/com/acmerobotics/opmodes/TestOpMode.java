package com.acmerobotics.opmodes;

import com.acmerobotics.RobomaticTesting.roboConfig;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.robomatic.config.ConfigurationLoader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="A1 Test")
public class TestOpMode extends LinearOpMode {

    @Override
    public void runOpMode(){

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry Telemetry = dashboard.getTelemetry();

        roboConfig config = (roboConfig) new ConfigurationLoader(hardwareMap.appContext).getConfig();

        Telemetry.addData("color", config.color);
        Telemetry.addData("delay", config.delay);
        Telemetry.addData("starting location", config.startLocation);

        Telemetry.addLine("before start");

        Telemetry.update();

        waitForStart();

        Telemetry.clearAll();
        Telemetry.update();

        while(!isStopRequested()){

            Telemetry.addData("color", config.color);
            Telemetry.addData("delay", config.delay);
            Telemetry.addData("starting location", config.startLocation);

            Telemetry.update();
        }
    }

}
