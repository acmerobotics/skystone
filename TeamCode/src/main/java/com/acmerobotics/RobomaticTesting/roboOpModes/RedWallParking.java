package com.acmerobotics.RobomaticTesting.roboOpModes;

import com.acmerobotics.RobomaticTesting.util.SkystoneConfiguration;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Wall Parking")
public class RedWallParking extends AutoOpMode {

    @Override
    protected void run() {

        autoType = SkystoneConfiguration.AutoType.PARKING;
        parkingLocation = SkystoneConfiguration.ParkingLocation.WALL;
        allianceColor = SkystoneConfiguration.AllianceColor.RED;
        roboRobot.config.collectBothSkystones = false;

        if (startLocation == SkystoneConfiguration.StartLocation.BUILDING_ZONE) {

            roboRobot.drive.resetEncoders();
            roboRobot.drive.goToPosition(-9, 0.5);

        } else {

            roboRobot.drive.resetEncoders();
            roboRobot.drive.goToPosition(9, 0.5);

        }

    }
}
