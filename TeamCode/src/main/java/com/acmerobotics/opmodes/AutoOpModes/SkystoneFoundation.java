package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.TheColorSensor;
import com.acmerobotics.robot.armEncoder;
import com.acmerobotics.robot.liftEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="SkystoneFoundation")

@Config
public class SkystoneFoundation extends LinearOpMode {

    @Override
    public void runOpMode(){

        Drive drive = new Drive(hardwareMap, false);
        TheColorSensor colorSensor = new TheColorSensor(hardwareMap);
        armEncoder arm = new armEncoder(hardwareMap);
        liftEncoder lift = new liftEncoder(hardwareMap);

        drive.resetEncoders();
        drive.resetAngle();
        drive.resetEncoderOmni();
        drive.resetStrafingPos();

        arm.resetEncoder();

        waitForStart();

        while(!isStopRequested()){

            // skystone piece

            // foundation piece

        }
    }
}
