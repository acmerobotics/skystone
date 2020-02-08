package com.acmerobotics.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="omniTest")
@Config
public class omniTest extends LinearOpMode {

    public boolean positionReached = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, false);

        drive.resetAngle();

        waitForStart();

        while(!isStopRequested()){

            drive.goToStrafingPos(-4, 0.3, "right");

            if (drive.atStrafingPos()){
                drive.stopMotors();
            }

            telemetry.addData("target", drive.getAngle());
            telemetry.update();
        }
    }
}