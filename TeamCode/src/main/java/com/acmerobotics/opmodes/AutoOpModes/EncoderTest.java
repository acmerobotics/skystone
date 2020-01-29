package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.robot.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Encoder Test")
public class EncoderTest extends LinearOpMode {


    private int state;

    @Override
    public void runOpMode() throws InterruptedException {

        Drive drive = new Drive(hardwareMap, false);

        state = 0;

        drive.resetEncoders();

        waitForStart();

        while(!isStopRequested()){

            switch (state) {

                case 0:

                    drive.goToPosition(24, 0.25);

                    state++;

                    break;

                case 1:

                    if(drive.atLinearPos()){
                        drive.stopMotors();

                        state++;
                    }


            }

            telemetry.addData("motor 0", drive.encoderTest0());
            telemetry.addData("motor 1", drive.encoderTest1());
            telemetry.addData("motor 2", drive.encoderTest2());
            telemetry.addData("motor 3", drive.encoderTest3());
            telemetry.update();



        }

    }
}
