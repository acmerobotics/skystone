package com.acmerobotics.RobomaticTesting;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config

public class roboRobot extends Robot {

    // init subsystem
    public final roboDrive drive;
    public final roboArm arm;
    public final roboLift lift;
    public final roboIntake intake;

    public roboRobot(LinearOpMode opMode){
        super(opMode); // gets LinearOpMode and hardwareMap so Robotic can interact with them

        registerHub("hub0"); // lets Robomatic interact with the devices connected with the hub
        registerHub("hub1");

        // create obj
        drive = new roboDrive(this, opMode);
        arm = new roboArm(this);
        lift = new roboLift(this);
        intake = new roboIntake(this);

        // will put subsystem into a list of subsystems to run their update method
        registerSubsytem(drive);
        registerSubsytem(arm);
        registerSubsytem(lift);
        registerSubsytem(intake);
    }
}