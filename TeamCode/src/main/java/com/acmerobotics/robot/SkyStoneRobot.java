package com.acmerobotics.robot;

import com.acmerobotics.robomatic.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SkyStoneRobot extends Robot {

    public final Drive drive;


    public SkyStoneRobot(LinearOpMode opMode){
        super(opMode);

        registerHub("Expansion Hub 2");

        drive = new Drive(this);
        registerSubsytem(drive);


    }
}
