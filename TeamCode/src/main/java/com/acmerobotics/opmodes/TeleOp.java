package com.acmerobotics.opmodes;

import com.acmerobotics.robot.Drive;
import com.acmerobotics.util.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //SkyStoneRobot robot = new SkyStoneRobot(this);
        Drive drive = new Drive(hardwareMap);


        while (!isStopRequested()){

            drive.setPower(new Vector2d(-gamepad1.left_stick_y, gamepad1.left_stick_x), -gamepad1.right_stick_x);

        }

    }

}
