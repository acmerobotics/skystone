package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.armEncoder;
import com.acmerobotics.robot.liftEncoder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Bridge Parking LZ")
public class RedBridgeParkingLoadingZone extends LinearOpMode {
    public int state;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, false);
        armEncoder arm = new armEncoder(hardwareMap);
        liftEncoder lift = new liftEncoder(hardwareMap);

        state = 0;
        drive.resetEncoderOmni();
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
                        drive.resetEncoderOmni();

                        state++;
                    }

                    break;


                case 2:

                    drive.goToStrafingPos(30,0.5, "left");

                    state++;

                    break;


                case 3:

                    if(drive.atStrafingPos()){
                        drive.stopMotors();

                        state++;
                    }

                    break;

                case 4:

                    arm.runTo(110);

                    if (lift.bottomSet){
                        state++;
                    }

                    else{
                        lift.tightenLiftString();

                        lift.goToBottom();
                    }

                    break;

            }

        }

    }
}
