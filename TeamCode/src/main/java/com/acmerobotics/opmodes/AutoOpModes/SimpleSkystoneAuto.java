package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.dashboard.config.Config;
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
    private double blockLocation = 24; // inches
    private double underBridge = -23; // will be negative
    private double awayBridge = -15; // will be negative

    private double forward = 0.5; // time
    private double back = -1.2; // time

    private int grabbed = 0;

    private String state = "init";

    @Override
    public void runOpMode() throws InterruptedException {
        TheColorSensor colorSensor = new TheColorSensor(hardwareMap);
        Drive drive = new Drive(hardwareMap, false);
        armEncoder arm = new armEncoder(hardwareMap);
        liftEncoder lift = new liftEncoder(hardwareMap);
        //ElapsedTime time = new ElapsedTime();

        drive.resetEncoders();
        drive.resetAngle();
        drive.resetEncoderOmni();
        drive.resetStrafingPos();

        drive.moveForwardPower = 0.3;
        drive.turnPower = 0.35;
        drive.strafePower = 0.3;

        waitForStart();
        //time.reset();

        while (!isStopRequested()) {
            colorSensor.HSV();

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

                case "init":
                    arm.runTo(110);

                    if (lift.bottomSet){
                        state = "goToBlocks";
                    }

                    else{
                        lift.tightenLiftString();

                        lift.goToBottom();
                    }

                    break;


                case "goToBlocks":

//                    drive.goToPosition((int)blockLocation, 0.28);
//
//                    if (drive.atLinearPos()){
//                        drive.stopMotors();
//
//                        state = "turn";
//                    }


                    //////////////////
                    // 45 in
                    drive.goToStrafingPos(180, 0.3, "right");

                    if (drive.atStrafingPos()){
                        drive.stopMotors();
                        drive.resetStrafingPos();

                        state = "atBlocks";
                    }

                    //////////////////////

                    break;


//                case "turn":
//
//                    drive.setDegrees(80);
//
//                    if(drive.getAngle() == 0) {
//                        drive.clockwise();
//                    }
//
//                    if(drive.getDegrees() > 0) {
//
//                        if(drive.getAngle() < drive.getDegrees()){
//                            drive.counterClockwise();
//
//                        } else {
//
//                            drive.stopMotors();
//                            state = "atBlocks";
//                        }
//
//                    } else {
//
//                        if(drive.getAngle() > drive.getDegrees()){
//                            drive.clockwise();
//
//                        } else {
//
//                            drive.stopMotors();
//                            state = "atBlocks";
//                        }
//
//                    }
//
//                    break;


                case "atBlocks":

                    drive.resetEncoders();
                    drive.resetEncoderOmni();
                    traveled = drive.motors[0].getCurrentPosition();

                    state = "lookingForSkystone";

                    break;


                case "lookingForSkystone":
                    if (colorSensor.isSkystoneHue()) {

                        drive.stopMotors();
                        traveled = drive.motors[0].getCurrentPosition();

                        state = "approach";

                    } else {
                        drive.moveForward(0.5);
                        traveled = drive.motors[0].getCurrentPosition();
                    }

                    break;


                case "approach":

                    // 2 in
                    drive.goToStrafingPos(8, 0.3, "right");

                    if (drive.atStrafingPos()){
                        drive.stopMotors();
                        drive.resetStrafingPos();

                        state = "grabBlock";
                    }

                    break;


                case "grabBlock":

                    drive.grab();
                    Thread.sleep(1500);
                    state = "retreat";

                    break;

                case "retreat":

                    //4 in
                    drive.goToStrafingPos(16, 0.3, "left");

                    if (drive.atStrafingPos()){
                    drive.resetStrafingPos();
                    drive.stopMotors();
                        state = "getToZero";
                    }

                    break;


                case "getToZero":
                    drive.goToPosition(0, 0.3);

                    if (drive.atLinearPos()){
                        drive.stopMotors();

                        state = "score";
                    }



                case "score":
                    drive.goToPosition((int)(underBridge + awayBridge), 0.3);

                    if (drive.atLinearPos()){
                        drive.stopMotors();

                        state = "next";
                    }


                case "next":

                    drive.release();
                    Thread.sleep(1500);

                    if (grabbed == 2){
                        drive.goToPosition((int)underBridge, 0.3);
                    }

                    else {
                        state = "return";
                    }

                    break;


                case "return":
                    double Return = drive.ticksToInches(traveled);
                    drive.goToPosition((int)Return + 3, 0.3);

                    if (drive.atLinearPos()){
                        drive.stopMotors();
                        drive.resetStrafingPos();


                        state = "resetStrafe";
                    }

                    break;

                case "resetStrafe":

                    // 16
                    drive.goToStrafingPos(-6, 0.3, "right");

                    if (drive.atStrafingPos()){
                        drive.stopMotors();
                        drive.resetStrafingPos();

                        state = "lookingForSkystone";
                    }

                    break;
            }



            /////////////////////////////////////////////////////////////////////////////////////////

            telemetry.addData( "skystone", colorSensor.isSkystoneSat());
            telemetry.addData( "current position 0", drive.motors[0].getCurrentPosition());
            telemetry.addData("traveled", traveled);
            telemetry.addLine();

            telemetry.addLine();

            telemetry.addData("state", state);
            telemetry.update();
        }
    }
}
