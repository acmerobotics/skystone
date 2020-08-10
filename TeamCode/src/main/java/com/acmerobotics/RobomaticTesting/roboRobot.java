package com.acmerobotics.RobomaticTesting;

import com.acmerobotics.RobomaticTesting.util.SkystoneConfiguration;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.config.ConfigurationLoader;
import com.acmerobotics.robomatic.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config

public class roboRobot extends Robot {

    // init subsystem
    public final roboDrive drive;
    public final roboArm arm;
    public final roboLift lift;
    public final roboIntake intake;
    // public final runToTest;
    // public final ImuTest

    private HardwareMap map;

    public SkystoneConfiguration config;

    public roboRobot(LinearOpMode opMode, HardwareMap map){
        super(opMode); // gets LinearOpMode and hardwareMap so Robotic can interact with them

        registerHub("hub0"); // lets Robomatic interact with the devices connected with the hub
        registerHub("hub1");

        this.map = map;

        // create obj
        drive = new roboDrive(this, opMode);
        arm = new roboArm(this);
        lift = new roboLift(this);
        intake = new roboIntake(this);
        config = (SkystoneConfiguration) new ConfigurationLoader(map.appContext).getConfig();
        // runToTest = new runToTest(this);
        // ImyTest = new ImuTest(this);

        // will put subsystem into a list of subsystems to run their update method
        registerSubsytem(drive); // subsystem is spelt wrong here (subsytem)
        registerSubsytem(arm);
        registerSubsytem(lift);
        registerSubsytem(intake);
        // registerSubsytem(runToTest);
        // registerSubsytem(ImuTest);
    }
}
