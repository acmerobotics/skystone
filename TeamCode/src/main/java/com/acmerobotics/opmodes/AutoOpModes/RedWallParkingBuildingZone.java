package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.robot.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Wall Parking BZ")
public class RedWallParkingBuildingZone extends LinearOpMode {
    private int state;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, false);

        state = 0;

        waitForStart();

        while(!isStopRequested()){

            switch (state) {

                //TODO add the recalibration stuff at the beginning

                case 0:

                    drive.goToPosition(-8, 0.5);

                    state++;

                    break;

                case 1:

                    if(drive.atLinearPos()){
                        drive.stopMotors();

                        state++;
                    }



            }

        }

    }
}
