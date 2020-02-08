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

        drive.turnPower = 0.35;

        waitForStart();

        while(!isStopRequested()){

            drive.setDegrees(80);

            drive.getDegrees();

            if(drive.getAngle() == 0) {
                drive.clockwise();
            }

            if(drive.getDegrees() > 0) {

                if(drive.getAngle() < drive.getDegrees()){
                    drive.counterClockwise();

                } else {

                    drive.stopMotors();
                }

            } else {

                if(drive.getAngle() > drive.getDegrees()){
                    drive.clockwise();

                } else {

                    drive.stopMotors();
                }

            }

            telemetry.addData("current", drive.getDegrees());
            telemetry.addLine();

            telemetry.addData("target", drive.getAngle());
            telemetry.update();
        }
    }
}