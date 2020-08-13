package com.acmerobotics.RobomaticTesting.roboOpModes;

import com.acmerobotics.RobomaticTesting.util.SkystoneConfiguration;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Bridge Parking")
public class BlueBridgeParking extends AutoOpMode {

    @Override
    protected void run() {

        autoType = SkystoneConfiguration.AutoType.PARKING;
        parkingLocation = SkystoneConfiguration.ParkingLocation.BRIDGE;
        allianceColor = SkystoneConfiguration.AllianceColor.BLUE;
        roboRobot.config.collectBothSkystones = false;

        if (startLocation == SkystoneConfiguration.StartLocation.BUILDING_ZONE) {

            roboRobot.drive.goToPosition(10, 0.5);

            roboRobot.drive.IgoToStrafingPos(100, "left");

        } else {

            roboRobot.drive.goToPosition(5, 0.5);

            roboRobot.drive.IgoToStrafingPos(10, "right");

        }

    }
}

