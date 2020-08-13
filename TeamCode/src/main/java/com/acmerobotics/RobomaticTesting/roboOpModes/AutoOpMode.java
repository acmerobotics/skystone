package com.acmerobotics.RobomaticTesting.roboOpModes;

import com.acmerobotics.RobomaticTesting.roboRobot;
import com.acmerobotics.RobomaticTesting.util.SkystoneConfiguration;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.config.ConfigurationLoader;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name="AutoOpMode")
public abstract class AutoOpMode extends LinearOpMode {

    protected roboRobot roboRobot;
    protected SkystoneConfiguration.StartLocation startLocation;
    protected SkystoneConfiguration.ParkingLocation parkingLocation;
    protected SkystoneConfiguration.AllianceColor allianceColor;
    protected SkystoneConfiguration.AutoType autoType;
    private double startTime;

    @Override
    public void runOpMode() throws InterruptedException {
        roboRobot = new roboRobot(this, hardwareMap);
        //placeholder for vision


        SkystoneConfiguration config = (SkystoneConfiguration) new ConfigurationLoader(hardwareMap.appContext).getConfig();

        waitForStart();

        startTime = System.currentTimeMillis();

        roboRobot.drive.strafePower = 0.5;

        startLocation = roboRobot.config.startLocation;
        parkingLocation = roboRobot.config.parkingLocation;
        allianceColor = roboRobot.config.allianceColor;
        autoType = roboRobot.config.autoType;

        //initialization routine
        roboRobot.arm.runTo(110);
        if (roboRobot.config.hasInitializationBeenRun){
            roboRobot.lift.tightenLiftString();
            roboRobot.lift.goToBottom();
        }


        run();



    }

    protected abstract void run ();

    public AutoAction grabFoundation = () -> {
      roboRobot.foundationMover.grab();

    };

    public AutoAction storeFoundation = () -> {
        roboRobot.foundationMover.store();
    };

    public AutoAction grabSkystone = () -> {
        roboRobot.drive.grab();
    };

    public AutoAction releaseSkystone = () -> {
        roboRobot.drive.release();
    };

    public AutoAction liftToBottom = () -> {
        roboRobot.lift.goToBottom();
    };

    public double currentTime () {

        return (System.currentTimeMillis() - startTime) / 1000.0;
    }

    public double timeLeft () {
        return 30 - currentTime();

    }


}
