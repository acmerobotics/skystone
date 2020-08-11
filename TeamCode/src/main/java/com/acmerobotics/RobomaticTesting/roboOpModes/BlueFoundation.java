package com.acmerobotics.RobomaticTesting.roboOpModes;

import com.acmerobotics.RobomaticTesting.util.SkystoneConfiguration;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="blue foundation")
public class BlueFoundation extends AutoOpMode {

    @Override
    protected void run() {
        startLocation = SkystoneConfiguration.StartLocation.FOUNDATION;
        parkingLocation = SkystoneConfiguration.ParkingLocation.BRIDGE;
        allianceColor = SkystoneConfiguration.AllianceColor.BLUE;
        roboRobot.config.collectBothSkystones = false;

        roboRobot.drive.IgoToStrafingPos(10, "right");

        roboRobot.drive.goToPosition(21, 0.5);

        roboRobot.drive.moveForward(0.18);
        grabFoundation.execute();
        roboRobot.pause(900);

        roboRobot.drive.resetAngle();
        //there would a be a turn in here but I'm too lazy to fix it

        roboRobot.drive.resetEncoders();
        roboRobot.drive.goToPosition(19, 0.75);

        storeFoundation.execute();

        roboRobot.drive.resetEncoders();
        roboRobot.drive.goToPosition(-3, 0.5);

        roboRobot.drive.resetAngle();
        //again, turn. again, not bothering

        roboRobot.drive.resetEncoders();
        roboRobot.drive.goToPosition(45, -0.5);




    }
}
