package com.acmerobotics.RobomaticTesting.IndividualTests;

import com.acmerobotics.RobomaticTesting.roboRobot;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestingOpMode extends LinearOpMode {

    @Override
    public void runOpMode(){

        roboRobot robot = new roboRobot(this);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        waitForStart();

        while(!isStopRequested()){

            if (gamepad1.a) {


            }

            if (gamepad1.a) {


            }

        }
    }
}
