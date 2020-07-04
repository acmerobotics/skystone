package com.acmerobotics.opmodes;

import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.Intake;
import com.acmerobotics.robot.skystoneVuforia;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="vuforiaTest")

public class vuforiaTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{

        skystoneVuforia vision = new skystoneVuforia();
        Drive drive = new Drive(hardwareMap, false);
        Intake intake = new Intake(hardwareMap);

        intake.rightOpen();
        drive.resetEncoders();
        drive.resetAngle();

        vision.init(hardwareMap);


        waitForStart();


        // prevents init stop glitch
        if (isStopRequested()){
            return;
        }

        vision.activate();

        while (!isStopRequested()){
            vision.goToSkystone(drive, intake); // detect and approach skystone

            vision.moveToBridge(drive); // move and score skystone

            vision.telemetry(telemetry);
        }
    }

}
