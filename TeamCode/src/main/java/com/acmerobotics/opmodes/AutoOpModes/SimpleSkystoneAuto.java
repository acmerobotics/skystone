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
        ElapsedTime time = new ElapsedTime();

        drive.resetEncoders();
        drive.resetAngle();

        drive.moveForwardPower = 0.3;
        drive.turnPower = 0.35;
        drive.strafePower = 0.3;


        waitForStart();
        time.reset();

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

                    drive.goToPosition(blockLocation, 0.28);

                    if (drive.atLinearPos()){
                        drive.stopMotors();

                        state = "turn";
                    }

                    break;


                case "turn":

                    drive.setDegrees(80);

                    if(drive.getAngle() == 0) {
                        drive.clockwise();
                    }

                    if(drive.getDegrees() > 0) {

                        if(drive.getAngle() < drive.getDegrees()){
                            drive.counterClockwise();

                        } else {

                            drive.stopMotors();
                            state = "atBlocks";
                        }

                    } else {

                        if(drive.getAngle() > drive.getDegrees()){
                            drive.clockwise();

                        } else {

                            drive.stopMotors();
                            state = "atBlocks";
                        }

                    }

                    break;


                case "atBlocks":
                    // might need to strafe closer to blocks

                    drive.resetEncoders();
                    traveled = drive.motors[0].getCurrentPosition();

                    state = "lookingForSkystone";

                    break;


                case "lookingForSkystone":
                    if (colorSensor.isSkystoneHue()) {

                        drive.stopMotors();
                        drive.resetTrackingOmni();
                        traveled = drive.motors[0].getCurrentPosition();

                        state = "approach";

                    } else {
                        drive.moveForward();
                        traveled = drive.motors[0].getCurrentPosition();
                    }

                    break;


                case "approach":

                    if(!timeReset){
                        time.reset();
                        timeReset = true;
                    }

                    if (time.seconds() < forward){
                        drive.strafeRight();
                    }

                    else{
                        drive.stopMotors();
                        timeReset = false;
                        state = "grabBlock";
                    }

                    break;


                case "grabBlock":

                    drive.grab();
                    Thread.sleep(1500);
                    state = "retreat";

                case "retreat":

                    if(!timeReset){
                        time.reset();
                        timeReset = true;
                    }

                    if (time.seconds() < back){
                        drive.strafeRight();
                    }

                    else{
                        drive.stopMotors();
                        timeReset = false;
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
                    drive.goToPosition(underBridge + awayBridge, 0.3);

                    if (drive.atLinearPos()){
                        drive.stopMotors();

                        state = "next";
                    }


                case "next":

                    drive.release();
                    Thread.sleep(1500);

                    if (grabbed == 2){
                        drive.goToPosition(underBridge, 0.3);
                    }

                    else {
                        state = "return";
                    }

                    break;


                case "return":
                    double Return = drive.ticksToInches(traveled);
                    drive.goToPosition(Return + 3, 0.3);

                    if (drive.atLinearPos()){
                        drive.stopMotors();

                        state = "resetStrafe";
                    }

                    break;



                case "resetStrafe":

                    if(!timeReset){
                        time.reset();
                        timeReset = true;
                    }

                    if (time.seconds() < 1){
                        drive.strafeRight();
                    }

                    else{
                        drive.stopMotors();
                        timeReset = false;
                        state = "lookingForSkystone";
                    }

                    break;
            }



            /////////////////////////////////////////////////////////////////////////////////////////

            telemetry.addData( "skystone", colorSensor.isSkystoneSat());
            telemetry.addData( "current position 0", drive.motors[0].getCurrentPosition());
            telemetry.addData("traveled", traveled);
            telemetry.addLine();

            telemetry.addData("omni target", drive.omniTracker.getTargetPosition());
            telemetry.addData("omni current",drive.omniTracker.getCurrentPosition());
            telemetry.addData("motors stopped", drive.motorsStopped);
            telemetry.addLine();

            telemetry.addData("state", state);
            telemetry.update();
        }
    }
}
