package com.acmerobotics.robot;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.practice.Vector2d;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.util.StickyGamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="PracticeTele")
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //SkyStoneRobot robot = new SkyStoneRobot(this);
        Drive drive = new Drive(hardwareMap);
        TelemetryPacket telemetryPacket = new TelemetryPacket()


        while (!isStopRequested()){

            if(gamepad1.x){
                drive.setPower(new Vector2d(1), 1);
            }

            if(gamepad1.y){
                drive.setPower(new Vector2d(-1), -1);
            }

            drive.setPower(new Vector2d(gamepad1.left_stick_y, gamepad1.left_stick_x), gamepad1.right_stick_x);
            drive.update();




        }


    }
}
