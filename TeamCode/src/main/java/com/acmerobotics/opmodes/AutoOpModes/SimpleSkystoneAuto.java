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
    private double blockLocation;
    private double underBridge; // will be negative
    private double awayBridge; // will be negative

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
                case "checkEncoders":
                    if (drive.motors[0].getCurrentPosition() > passedBlocks){ //// need to convert to ticks
                        drive.goToPosition(parkingLocation, 0.3);
                    }

                    else{
                        // change state
                    }

                    break;


                case "checkTime":
                    if (time.seconds() < parkTime){
                        drive.goToPosition(parkingLocation, 0.3);
                    }

                    else{
                        // change state
                    }

                    break;

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


                case "goToBlocks":                     /// requires strafe
                    // strafe to block location

                    if (){ // if target location reached
                        drive.stopMotors();

                        state = "atBlocks";
                    }

                    break;


                case "atBlocks":
                    drive.resetEncoders();
                    traveled = drive.motors[0].getCurrentPosition();

                    state = "lookingForSkystone";

                    break;


                case "lookingForSkystone":
                    if (colorSensor.isSkystoneHue()) {

                        drive.stopMotors();
                        traveled = drive.motors[0].getCurrentPosition();

                        state = "grabBlock";

                    } else {
                        drive.moveForward();
                        traveled = drive.motors[0].getCurrentPosition();
                    }

                    break;


                case "grabBlock": /// requires strafe
                    // strafe to the right x inches to grab block

                    drive.grab();

                    // strafe to the left x to get back to original spot then plus 4 inches to not run into other blocks when returning

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
                    drive.goToPosition(Return, 0.3);

                    // strafe x minus 4 (see grab block)

                    state = "lookingForSkystone";

                    break;


                case "grabAny":

            }



            /////////////////////////////////////////////////////////////////////////////////////////
            if(!stoneDetected) {
                if (colorSensor.isSkystoneHue()) {

                    drive.stopMotors();
                    traveled = drive.motors[0].getCurrentPosition();
                    stoneDetected = true;

                } else {
                    drive.moveForward();
                    traveled = drive.motors[0].getCurrentPosition();
                }
            }

            if (stoneDetected){

                    drive.goToPosition(0, 0.3);

                    if (lift.bottomSet) {
                        drive.goToPosition(-40, 0.3);

                        if (drive.atLinearPos()) {
                            drive.stopMotors();
                        }
                    } else {
                        arm.runTo(80);

                        lift.tightenLiftString();

                        lift.goToBottom();
                    }
            }

            telemetry.addData( "skystone", colorSensor.isSkystoneSat());
            telemetry.addData( "current position 0", drive.motors[0].getCurrentPosition());
            telemetry.addLine();

            telemetry.addData("state", state);
            telemetry.update();
        }
    }
}
