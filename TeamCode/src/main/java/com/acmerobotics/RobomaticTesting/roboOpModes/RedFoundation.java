package com.acmerobotics.RobomaticTesting.roboOpModes;

import com.acmerobotics.RobomaticTesting.util.SkystoneConfiguration;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Foundation")
public class RedFoundation extends AutoOpMode {

    @Override
    protected void run() {

        autoType = SkystoneConfiguration.AutoType.FOUNDATION;
        startLocation = SkystoneConfiguration.StartLocation.BUILDING_ZONE;
        parkingLocation = SkystoneConfiguration.ParkingLocation.BRIDGE;
        allianceColor = SkystoneConfiguration.AllianceColor.RED;
        roboRobot.config.collectBothSkystones = false;

        roboRobot.drive.resetEncoders();
        roboRobot.drive.resetAngle();
        roboRobot.drive.resetEncoderOmni();

        roboRobot.drive.IgoToStrafingPos(10, "left");

        roboRobot.drive.resetEncoders();
        roboRobot.drive.goToPosition(29, 0.25);

        roboRobot.drive.moveForward(0.18); //this might cause problems, but we'll cross that bridge later
        grabFoundation.execute();
        roboRobot.pause(900);

        roboRobot.drive.resetAngle();
        //here's a turn. wasn't it wonderful? (see blue foundation for context)

        roboRobot.drive.resetEncoders();
        roboRobot.drive.goToPosition(20, 0.75);

        storeFoundation.execute();

        roboRobot.drive.resetEncoders();
        roboRobot.drive.goToPosition(-3, 0.5);

        roboRobot.drive.resetAngle();
        //i'm not doing the code for this

        roboRobot.drive.resetEncoders();
        roboRobot.drive.goToPosition(41, -0.5);


    }
}
