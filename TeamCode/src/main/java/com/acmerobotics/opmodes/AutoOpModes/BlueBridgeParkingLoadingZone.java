package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.robot.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Bridge Parking LZ")
public class BlueBridgeParkingLoadingZone extends LinearOpMode {
    public int state;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, false);


        state = 0;

        waitForStart();


        while(!isStopRequested()){

            switch (state){

                case 0:

                    drive.goToPosition(10, 0.5);

                    state++;

                    break;

                case 1:

                    if(drive.atLinearPos()){
                        drive.stopMotors();

                        state++;
                    }

                    break;


                case 2:

                    drive.goToStrafingPos(30, 0.5, "left");

                    state++;

                    break;


                case 3:

                    if(drive.atStrafingPos()){
                        drive.stopMotors();

                        state++;
                    }

                    break;

                case 4:

                    //TODO add the recalibration stuff
            }

        }

    }
}
