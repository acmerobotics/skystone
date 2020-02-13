package com.acmerobotics.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="omniTest")
@Config
public class omniTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);

        waitForStart();

        while (!isStopRequested()){

            if (gamepad1.x){
                intake.rightFullyOpen();
            }
        }
    }
}