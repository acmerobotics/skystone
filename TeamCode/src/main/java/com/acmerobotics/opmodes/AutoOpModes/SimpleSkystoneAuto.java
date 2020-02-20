package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.AngleCorrector;
import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.TheColorSensor;
import com.acmerobotics.robot.armEncoder;
import com.acmerobotics.robot.liftEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="SimpleSkystoneAuto")
@Config
public class SimpleSkystoneAuto extends LinearOpMode { /////////////////////////   red side

    private int traveled = 0;
    private boolean stoneDetected = false;
    private boolean liftDown = false;

    private double parkTime = 10;
    private boolean timeReset = false;

    private double passedBlocks;

    private double parkingLocation;
    private double blockLocation = 26; // inches
    private double underBridge = 23; // will be negative
    private double awayBridge = 15; // will be negative

    private double forward = 0.5; // time
    private double back = -1.2; // time

    private int grabbed = 0;

    private String state = "goToBlocks";

    @Override
    public void runOpMode() throws InterruptedException {
        TheColorSensor colorSensor = new TheColorSensor(hardwareMap);
        Drive drive = new Drive(hardwareMap, false);
        ElapsedTime time = new ElapsedTime();

        drive.resetEncoders();
        drive.resetAngle();
        drive.resetEncoderOmni();
        drive.resetStrafingPos();

        //drive.moveForwardPower = 0.3;

        waitForStart();
        //time.reset();

        while (!isStopRequested()) {

            switch(state){

                /////////////////////// time and encoder checks /////////////////////////
//                case "checkEncoders":
//                    if (drive.motors[0].getCurrentPosition() > passedBlocks){ //// need to convert to ticks
//                        drive.goToPosition(parkingLocation, 0.3);
//                    }
//
//                    else{
//                        // change state
//                    }
//
//                    break;
//
//
//                case "checkTime":
//                    if (time.seconds() < parkTime){
//                        drive.goToPosition(parkingLocation, 0.3);
//                    }
//
//                    else{
//                        // change state
//                    }
//
//                    break;

                ////////////////////////////////////////////////////////////////////////////

                case "goToBlocks":

                    drive.IgoToStrafingPos(27, "right");

                    if (drive.IatStrafingPos()){
                        drive.stopMotors();
                        drive.resetStrafingPos();
                        time.reset();

                        state = "atBlocks";
                    }
                    break;

//                case "angle adjust 1":
//                    if (time.seconds() < 1) {
//                        drive.correctingPower(0, 0, "");
//                        drive.correctingPower(0, 1, "");
//                        drive.correctingPower(0, 2, "");
//                        drive.correctingPower(0, 3, "");
//                    }
//                    else{
//                        state = "atBlocks";
//                    }
//                    break;


                case "atBlocks":
                    drive.resetEncoders();
                    drive.resetEncoderOmni();
                    traveled = 0;

                    state = "lookingForSkystone";

                    break;


                case "lookingForSkystone":
                    colorSensor.HSV();
                    if (colorSensor.isSkystoneHue()) {

                        drive.stopMotors();
                        traveled = drive.motors[0].getCurrentPosition();

                        state = "approach1";

                    } else {
                        drive.moveForward(-0.28);
                        traveled = drive.motors[0].getCurrentPosition();
                    }

                    break;


                case "approach1":

                    // 2 in
                    drive.IgoToStrafingPos(2, "right");

                    if (drive.IatStrafingPos()){
                        drive.stopMotors();
                        drive.resetStrafingPos();
                        time.reset();
                        drive.resetEncoders();

                        state = "move3";
                    }
                    break;


//                case "angle adjust 3":
//                    if (time.seconds() < 1) {
//                        drive.correctingPower(0, 0);
//                        drive.correctingPower(0, 1);
//                        drive.correctingPower(0, 2);
//                        drive.correctingPower(0, 3);
//                    }
//                    else{
//                        state = "grabBlock";
//                    }
//                    break;

                case "move3":
                    drive.goToPosition(-4, 0.25);

                    if (drive.atLinearPos()){
                        drive.stopMotors();

                        state = "grabBlock";
                    }


                case "grabBlock":

                    drive.grab();
                    Thread.sleep(1500);
                    state = "retreat";

                    break;

                case "retreat":

                    drive.Pcoefficient = 0.03;
                    drive.IgoToStrafingPos(6, "left");

                    if (drive.IatStrafingPos()){
                        drive.resetStrafingPos();
                        drive.stopMotors();
                        time.reset();
                        drive.resetEncoders();
                        state = "getToZero";
                    }
                    break;


//                case "angle adjust 4":
//                    if (time.seconds() < 1) {
//                        drive.correctingPower(0, 0);
//                        drive.correctingPower(0, 1);
//                        drive.correctingPower(0, 2);
//                        drive.correctingPower(0, 3);
//                    }
//                    else{
//                        state = "getToZero";
//                    }
//                    break;


                case "getToZero":
                    drive.goToPosition(0, 0.3);

                    if (drive.atLinearPos()){
                        drive.stopMotors();
                        drive.resetEncoders();

                        state = "score";
                    }



                case "score":
                    drive.goToPosition((int)(underBridge + awayBridge), 0.3);

                    if (drive.atLinearPos()){
                        drive.stopMotors();
                        grabbed++;

                        state = "next";
                    }


                case "next":

                    drive.release();
                    Thread.sleep(1500);

//                    if (grabbed == 2){
//                        drive.goToPosition((int)underBridge, 0.3);
//                    }

//                    else {
//                        state = "return";
//                    }

                    break;


                case "return":
                    double Return = drive.ticksToInches(traveled);
                    drive.goToPosition(-((int)Return + 3), 0.3);

                    if (drive.atLinearPos()){
                        drive.stopMotors();
                        drive.resetStrafingPos();


                        state = "resetStrafe";
                    }

                    break;

                case "resetStrafe":

                    // 16
                    drive.IgoToStrafingPos(6, "right");

                    if (drive.IatStrafingPos()){
                        drive.stopMotors();
                        drive.resetStrafingPos();
                        time.reset();

                        state = "angle adjust 2";
                    }
                    break;

            }



            /////////////////////////////////////////////////////////////////////////////////////////

            telemetry.addData( "skystone", colorSensor.isSkystoneSat());
            telemetry.addData( "current position 0", drive.motors[0].getCurrentPosition());
            telemetry.addData("pCoefficient", drive.Pcoefficient);
            telemetry.addLine();

            telemetry.addLine();

            telemetry.addData("angle error", drive.error);

            telemetry.addData("state", state);
            telemetry.update();
        }
    }
}
