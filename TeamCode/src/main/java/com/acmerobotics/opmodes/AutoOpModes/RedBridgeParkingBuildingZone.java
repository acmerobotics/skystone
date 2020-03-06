package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.armEncoder;
import com.acmerobotics.robot.liftEncoder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Bridge Parking BZ")
public class RedBridgeParkingBuildingZone extends LinearOpMode {
    private int state;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, false);
        armEncoder arm = new armEncoder(hardwareMap);
        liftEncoder lift = new liftEncoder(hardwareMap);

        state = 0;
        drive.strafePower = 0.5;
        drive.resetEncoderOmni();
        waitForStart();


        while (!isStopRequested()){

            switch (state){

                case 0:

                    arm.runTo(110);

                    if (lift.bottomSet){
                        state++;
                    }

                    else{
                        lift.tightenLiftString();

                        lift.goToBottom();
                    }

                    break;

                case 1:

                    drive.resetEncoders();

                    drive.goToPosition(-5, 0.5);

                    state++;

                    break;

                case 2:

                    if(drive.atLinearPos()){
                        drive.stopMotors();
                        drive.resetEncoderOmni();

                        state++;
                    }

                    break;


                case 3:

                    drive.IgoToStrafingPos(25, "right");

                    state++;

                    break;


                case 4:

                    if(drive.IatStrafingPos()){
                        drive.stopMotors();

                        state++;
                    }

                    break;


            }

        }
    }
}
