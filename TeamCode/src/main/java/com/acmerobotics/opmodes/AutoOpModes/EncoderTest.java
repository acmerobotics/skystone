package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.robot.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Encoder Test")
public class EncoderTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Drive drive = new Drive(hardwareMap, false);

        waitForStart();

        while(!isStopRequested()){

            drive.encoderTest1();
            drive.encoderTest2();

            telemetry.addData("motor 1", drive.encoderTest1());
            telemetry.addData("motor 2", drive.encoderTest2());
            telemetry.update();



        }

    }
}
