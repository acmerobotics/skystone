package com.acmerobotics.robot;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.practice.Vector2d;
import com.acmerobotics.robomatic.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SkyStoneRobot robot = new SkyStoneRobot(this);


        while (!isStopRequested()){

            robot.drive.setPower(new Vector2d(-gamepad1.left_stick_y, gamepad1.left_stick_x), gamepad1.right_stick_x);

        }

    }
}
