package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.TheColorSensor;
import com.acmerobotics.robot.armEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="SkystoneGrabTest")
@Config

public class SkystoneGrabTest extends LinearOpMode{

    private boolean inStone1Position = false;

    private int ticksTraveled1 = 0;
    private int ticksTraveled2 = 0;

    private boolean stone1Detected = false;
    private boolean stone2Detected = false;

    private int stonesMoved = 0;

    private int timeToPark = 8; // minimum time needed to park
    private int passedBlocks; // amount of ticks equal to the length of block, checks if drive passed the blocks

    private String state = "atParking"; // "grabFirst"  "returnFirst" "grabSecond" "returnSecond" "grabAny"

        @Override
        public void runOpMode() throws InterruptedException {
            TheColorSensor colorSensor = new TheColorSensor(hardwareMap);
            Drive drive = new Drive(hardwareMap, false);
            armEncoder arm = new armEncoder(hardwareMap);
            ElapsedTime time = new ElapsedTime();

            drive.resetEncoders();

            waitForStart();
            time.reset();

            while(!isStopRequested()){
                colorSensor.HSV();

                switch (state) {

                    case "atParking":
                        drive.resetEncoders();


                    case "firstBlock":
                        if (time.seconds() > timeToPark){
                            if (drive.motors[0].getCurrentPosition() < passedBlocks){
                                if (!stone1Detected) {
                                    if (colorSensor.isSkystoneSat()) {

                                        stone1Detected = true;
                                        ticksTraveled1 = drive.motors[0].getCurrentPosition();

                                    } else {
                                        drive.moveBack();
                                        ticksTraveled1 = drive.motors[0].getCurrentPosition();
                                    }
                                } else {

                                    //move towards block   ////////////////////////////////////////////////
                                    drive.grab();
                                    Thread.sleep(1500);
                                    //back up ////////////////////////////////////////////////////

                                    drive.goToPosition(-48, 0.6);

                                    drive.release();
                                    Thread.sleep(1000);

                                    drive.goToPosition(0, 0.5); // get under bridge

                                    stonesMoved = 1;
                                    state = "secondBlock";
                                }

                            } else{
                                // passed blocks so just go park
                                double inchesTobackUp = -(drive.ticksToInches(ticksTraveled1));
                                drive.goToPosition((int)inchesTobackUp, 0.6);
                            }

                        } else{
                            // only enough time to park
                            double inchesTobackUp = -(drive.ticksToInches(ticksTraveled1));
                            drive.goToPosition((int)inchesTobackUp, 0.6);
                        }

                        break;


                    case "secondBlock":
                        if (time.seconds() > timeToPark){
                            if (drive.motors[0].getCurrentPosition() < passedBlocks){
                                if (!inStone1Position){
                                    double stone1Position = drive.ticksToInches(ticksTraveled2);
                                    drive.goToPosition((int)stone1Position + 5, 0.6);
                                    inStone1Position = true;
                                }

                                else {
                                    if (!stone2Detected) {
                                        if (colorSensor.isSkystoneHue()) {

                                            ticksTraveled2 = drive.motors[0].getCurrentPosition();
                                            stone2Detected = true;

                                        } else {
                                            drive.moveBack();
                                            ticksTraveled2 = drive.motors[0].getCurrentPosition();
                                        }

                                    }else{

                                        //move towards block   /////////////////////////////////////////////////
                                        drive.grab();
                                        Thread.sleep(1500);
                                        //back up ////////////////////////////////////////////////////

                                        double inchesTobackUp = -(drive.ticksToInches(ticksTraveled2));
                                        drive.goToPosition((int)inchesTobackUp + 6, 0.6);

                                        drive.release();
                                        Thread.sleep(1000);

                                        drive.goToPosition(-6, 0.5); // get under bridge

                                        stonesMoved = 2;
                                        state = "thirdBlock";
                                    }
                                }

                            } else{
                                // passed blocks so just go park
                                double inchesTobackUp = -(drive.ticksToInches(ticksTraveled2));
                                drive.goToPosition((int)inchesTobackUp, 0.6);
                            }

                        } else{
                            // only enough time to park
                            double inchesTobackUp = -(drive.ticksToInches(ticksTraveled2));
                            drive.goToPosition((int)inchesTobackUp, 0.6);
                        }

                        break;

                    case "thirdBlock":
                        //if there is enough time grab any block (closest to bridge)
                }

                telemetry.addData( "current position", drive.motors[0].getCurrentPosition());
                telemetry.addData("time" , time.seconds());
                telemetry.addData( "skystone", colorSensor.isSkystoneSat());
                telemetry.addData("state" , state);
                //telemetry.addData( , );
                telemetry.update();

            }

        }
}
