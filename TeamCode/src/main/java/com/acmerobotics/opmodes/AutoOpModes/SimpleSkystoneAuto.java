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

    private double passedBlocks;

    private double parkingLocation;
    private double blockLocation = 22; // inches
    private double forward = 2;
    private double back = -3;
    private double underBridge = -22; // will be negative
    private double awayBridge = -12; // will be negative

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

        drive.moveForwardPower = 0.3;

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
                    drive.goToStrafingPos((int)blockLocation, 0.3, "right");

                    if (drive.atStrafingPos()){
                        drive.stopMotors();

                        state = "atBlocks";
                    }

                    break;


                case "atBlocks":
                    drive.resetEncoders();
                    drive.resetTrackingOmni();
                    traveled = drive.motors[0].getCurrentPosition();

                    state = "lookingForSkystone";

                    break;


                case "lookingForSkystone":
                    if (colorSensor.isSkystoneHue()) {

                        drive.stopMotors();
                        drive.resetTrackingOmni();
                        traveled = drive.motors[0].getCurrentPosition();

                        state = "grabBlock";

                    } else {
                        drive.moveForward();
                        traveled = drive.motors[0].getCurrentPosition();
                    }

                    break;


                case "grabBlock":
                    drive.goToStrafingPos((int)forward, 0.25, "right");

                    drive.grab();

                    drive.goToStrafingPos((int)back, 0.25, "left");

                    grabbed++;

                    state = "score";

                    break;


                case "score":
                    drive.goToPosition(0, 0.3);
                    drive.goToPosition(underBridge + awayBridge, 0.3);

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

                    drive.goToStrafingPos((int)forward, 0.25, "right");

                    state = "lookingForSkystone";

                    break;


                case "grabAny":

            }



            /////////////////////////////////////////////////////////////////////////////////////////

            telemetry.addData( "skystone", colorSensor.isSkystoneSat());
            telemetry.addData( "current position 0", drive.motors[0].getCurrentPosition());
            telemetry.addData("traveled", traveled);
            telemetry.addData("omni",drive.omniTracker.getCurrentPosition());
            telemetry.addLine();

            telemetry.addData("state", state);
            telemetry.update();
        }
    }
}
