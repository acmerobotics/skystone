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

    private double parkTime = 10;
    private boolean timeReset = false;

    private double blockLocation = 26;
    private double underBridge = 24;
    private double awayBridge = 26;

    private int grabbed = 0;

    private String state = "goToBlocks";
    private int path = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        TheColorSensor colorSensor = new TheColorSensor(hardwareMap);
        armEncoder arm = new armEncoder(hardwareMap);
        liftEncoder lift = new liftEncoder(hardwareMap);
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

            arm.runTo(110);

            if (!lift.bottomSet) {

                lift.tightenLiftString();

                lift.goToBottom();
            }

            switch(state){

                /////////////////////// time check ///////////////////////// ( can probably put it in another switch that
                                                                                          // runs parallel to main one)
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


                case "angle adjust 1": // might remove because the error is not worth the time to fix it
                    if (time.seconds() < 1.5) {
                        drive.correctingPower(0, 0, "");
                        drive.correctingPower(0, 1, "");
                        drive.correctingPower(0, 2, "");
                        drive.correctingPower(0, 3, "");
                    }
                    else{
                        state = "atBlocks";
                    }
                    break;


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

                        if (path == 1){
                            state = "approach1";
                        }

                        else {
                            state = "pickPath1";
                        }

                    } else {
                        drive.moveForward(-0.28);
                        traveled = drive.motors[0].getCurrentPosition();
                    }

                    break;


                case "pickPath1":
                    //path 1
                    if (drive.ticksToInches(-traveled) == 0){
                        path = 1;

                        colorSensor.HSV();
                        if (colorSensor.isSkystoneHue()){
                            drive.moveForward(-0.28);
                        }

                        else{
                            drive.stopMotors();
                            state = "lookingForSkystone";
                        }
                    }

                    //path 2
                    if (drive.ticksToInches(-traveled) >= 1 && drive.ticksToInches(-traveled) < 8){
                        path = 2;
                        state = "approach1";
                    }

                    //path 3
                    if (drive.ticksToInches(-traveled) >= 8){
                        path = 3;
                        state = "approach1";
                    }


                case "approach1":

                    // 2 in
                    drive.IgoToStrafingPos(3.5, "right");

                    if (drive.IatStrafingPos()){
                        drive.stopMotors();
                        drive.resetStrafingPos();
                        time.reset();
                        drive.resetEncoders();

                        state = "move3";
                    }
                    break;


                case "move3":
                    drive.goToPosition(-3.7, 0.25);

                    if (drive.atLinearPos()){
                        drive.stopMotors();

                        state = "grabBlock";
                    }
                    break;


                case "grabBlock":

                    Thread.sleep(1000);
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


                case "getToZero":
                    double zero = drive.ticksToInches(-traveled);

                    drive.goToPosition(zero, 0.3);

                    if (drive.atLinearPos()){
                        drive.stopMotors();
                        drive.resetEncoders();


                        state = "score";
                    }
                    break;


                case "score":

                    drive.goToPosition(((underBridge + awayBridge)), 0.3);

                    if (drive.atLinearPos()){
                        drive.stopMotors();
                        grabbed++;

                        state = "next";
                    }
                    break;


                case "next":

                    Thread.sleep(500);
                    drive.release();
                    Thread.sleep(1000);

                    if (grabbed == 2){
                        state = "park";
                    }

                    else {
                        state = "pickPath2";
                    }

                    break;


                case "pickPath2":
                    if (path == 1){
                        drive.goToPosition(underBridge - 10, 0.3); // might need to adjust the subtraction

                        if (drive.atLinearPos()){
                            drive.stopMotors();
                            state = "resetStrafe";
                        }
                    }

                    if (path == 2){
                        double return1 = traveled - 24;

                        drive.goToPosition(return1, 0.28);

                        if (drive.atLinearPos()){
                            drive.stopMotors();
                            state = "resetStrafe";
                        }
                    }

                    if (path == 3){
                        double return2 = traveled - 8;

                        drive.goToPosition(return2, 0.28);

                        if (drive.atLinearPos()){
                            drive.stopMotors();
                            state = "resetStrafe";
                        }
                    }


                case "resetStrafe":

                    drive.IgoToStrafingPos(6, "right");

                    if (drive.IatStrafingPos()){
                        drive.stopMotors();
                        drive.resetStrafingPos();
                        time.reset();

                        if (path == 1){
                            state = "lookingForSkystone";
                        }
                        else {
                            state = "approach 1";
                        }
                    }
                    break;


                case "park":
                    drive.goToPosition((int)underBridge, 0.3);

            }



            ////////////////////////////////////////////////////////////////////////////////////////

            telemetry.addData("traveled", traveled);
            telemetry.addLine();

            telemetry.addLine();

            telemetry.addData("angle error", drive.error);

            telemetry.addData("state", state);
            telemetry.update();
        }
    }
}
