package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.robot.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Wall Parking BZ")
public class BlueWallParkingBuildingZone extends LinearOpMode {
    public int state;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, false);

        state = 0;

        drive.resetEncoders();

        telemetry.addData("state", state);
        telemetry.addData("current pos", drive.getCurrentPos());
        telemetry.addData("target pos bool", drive.returnAtTargetPos());
        telemetry.update();

        waitForStart();

        telemetry.clearAll();

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


                    //TODO add in all the recalibration things


            }



        }

    }
}
