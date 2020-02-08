package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.robot.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Wall Parking LZ")
public class RedWallParkingLoadingZone extends LinearOpMode {
    private int state;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, false);

        state = 0;

        drive.resetEncoders();

        telemetry.addData("state", state);
        telemetry.addData("current pos", drive.getCurrentPos());
        telemetry.update();

        waitForStart();

        telemetry.clearAll();

        while(!isStopRequested()){

            switch (state) {

                //TODO add in all the recalibration things

                case 0:

                    drive.goToPosition(9, 0.5);

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
