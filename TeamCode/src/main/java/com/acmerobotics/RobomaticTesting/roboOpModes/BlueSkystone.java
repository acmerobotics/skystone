package com.acmerobotics.RobomaticTesting.roboOpModes;

import com.acmerobotics.RobomaticTesting.util.SkystoneConfiguration;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Skystone")
public class BlueSkystone extends AutoOpMode {

    private int path = 0;

    @Override
    protected void run() {

        autoType = SkystoneConfiguration.AutoType.SKYSTONE;
        startLocation = SkystoneConfiguration.StartLocation.LOADING_ZONE;
        parkingLocation = SkystoneConfiguration.ParkingLocation.BRIDGE; //I think the skystones park at bridge but i honestly don't know
        allianceColor = SkystoneConfiguration.AllianceColor.BLUE;
        roboRobot.config.collectBothSkystones = false;

        //lift down
        roboRobot.arm.runTo(80);
        roboRobot.lift.tightenLiftString();
        roboRobot.lift.goToBottom();

        //lift up
        roboRobot.arm.armMotor.setPower(-0.2);
        roboRobot.lift.runTo(2000, 0.5);

        //go to blocks
        roboRobot.drive.resetEncoders();
        roboRobot.drive.resetEncoderOmni();

        //looking for skystone
        //would have color sensor stuff, but that has to be updated to Robomatic and I don't wanna rn

        //Actually this whole class has to be adjusted to fit with robomatic, so unless we want to test this particular auto with the new
        //system, I'm just going to leave it.

    }
}
