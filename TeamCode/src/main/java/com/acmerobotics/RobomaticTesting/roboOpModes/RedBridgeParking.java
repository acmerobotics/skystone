package com.acmerobotics.RobomaticTesting.roboOpModes;

import com.acmerobotics.RobomaticTesting.util.SkystoneConfiguration;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Bridge Parking")
public class RedBridgeParking extends AutoOpMode {

    @Override
    protected void run() {

        autoType = SkystoneConfiguration.AutoType.PARKING;
        parkingLocation = SkystoneConfiguration.ParkingLocation.BRIDGE;
        allianceColor = SkystoneConfiguration.AllianceColor.RED;
        roboRobot.config.collectBothSkystones = false;

        if (startLocation == SkystoneConfiguration.StartLocation.BUILDING_ZONE) {

            roboRobot.drive.resetEncoders();
            roboRobot.drive.goToPosition(-5, 0.5);

            roboRobot.drive.resetEncoderOmni();
            roboRobot.drive.IgoToStrafingPos(25, "right");

        } else {

            roboRobot.drive.resetEncoders();
            roboRobot.drive.goToPosition(10, 0.5);

            roboRobot.drive.resetEncoderOmni();
            roboRobot.drive.IgoToStrafingPos(100,  "left");


        }

    }
}
