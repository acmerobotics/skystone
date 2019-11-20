package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.robot.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="parking blue")



public class ParkingBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap);

        while(!isStopRequested()) {

            //none of the encoder movement will work without the encoder info such as tick count (found in drive)

            drive.moveRobotTo("forward", 10);

            drive.turnRobotTo("right", 90); //won't work without adding values to getRadianLength, wheel to center variable

            drive.moveRobotTo("back", -10);
        }
    }
}