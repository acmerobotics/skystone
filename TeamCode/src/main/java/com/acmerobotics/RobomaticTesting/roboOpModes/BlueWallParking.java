package com.acmerobotics.RobomaticTesting.roboOpModes;

import com.acmerobotics.RobomaticTesting.util.SkystoneConfiguration;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Wall Parking")
public class BlueWallParking extends AutoOpMode {

    @Override
    protected void run() {

        autoType = SkystoneConfiguration.AutoType.PARKING;
        parkingLocation = SkystoneConfiguration.ParkingLocation.WALL;
        allianceColor = SkystoneConfiguration.AllianceColor.BLUE;
        roboRobot.config.collectBothSkystones = false;

        if (startLocation == SkystoneConfiguration.StartLocation.BUILDING_ZONE) {

            roboRobot.drive.resetEncoders();
            roboRobot.drive.goToPosition(6, 0.5);

        } else {

            roboRobot.drive.resetEncoders();
            roboRobot.drive.goToPosition(9, 0.5);


        }

    }
}
