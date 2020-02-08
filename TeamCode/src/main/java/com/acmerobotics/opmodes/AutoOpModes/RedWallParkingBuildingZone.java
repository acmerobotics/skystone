package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.armEncoder;
import com.acmerobotics.robot.liftEncoder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Wall Parking BZ")
public class RedWallParkingBuildingZone extends LinearOpMode {
    private int state;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, false);
        armEncoder arm = new armEncoder(hardwareMap);
        liftEncoder lift = new liftEncoder(hardwareMap);

        state = 0;

        waitForStart();

        while(!isStopRequested()){

            switch (state) {

                case 0:

                    drive.goToPosition(10, 0.5);

                    state++;

                    break;

                case 1:

                    if(drive.atLinearPos()){
                        drive.stopMotors();

                        state++;
                    }


                case 2:

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
