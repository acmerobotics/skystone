package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.TheColorSensor;
import com.acmerobotics.robot.armEncoder;
import com.acmerobotics.robot.liftEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="SkystoneRed")
@Config

public class BlueSkystoneAuto extends LinearOpMode {

    private int traveled = 0;

    private double blockLocation = 26;
    private double underBridge = 22;
    private double awayBridge = 26;

    private int grabbed = 0;

    private String state = "goToBlocks";
    private String state2 = "liftDown";
    private int path = 0;

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
        arm.resetEncoder();

        waitForStart();
        //time.reset();

        while (!isStopRequested()) {

            switch(state2){
                case "liftDown":
                    arm.runTo(80);

                    if (!lift.bottomSet) {

                        lift.tightenLiftString();

                        lift.goToBottom();
                    }
                    break;

                case "liftUp":
                    arm.armMotor.setPower(-0.2);

                    lift.runTo(2000, 0.5);

                    break;
            }

            switch(state){

                case "goToBlocks":
                    state2 = "liftDown";

                    drive.IgoToStrafingPos(28, "right");

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
                        if (!colorSensor.isSkystoneHue()){
                            drive.moveForward(-0.28);
                        }

                        else{
                            drive.stopMotors();
                            state = "lookingForSkystone";
                        }
                    }

                    else{
                        state = "approach1";
                    }

                    break;


                case "approach1":

                    // 2 in
                    drive.IgoToStrafingPos(4.2, "right");

                    if (drive.IatStrafingPos()){
                        drive.stopMotors();
                        drive.resetStrafingPos();
                        time.reset();
                        drive.resetEncoders();

                        state = "move3";
                    }
                    break;


                case "move3":
                    drive.goToPosition(-3.9, 0.25);

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
                    state2 = "liftDown"; // might need time delay so lift has time to move to position

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

                    state = "park";

                    break;

                case "park":
                    drive.goToPosition((int)underBridge, 0.3);

            }


            ///////////////////////////////////////////////////////////////////////////////////////

            telemetry.addData("traveled", traveled);
            telemetry.addLine();

            telemetry.addLine();

            telemetry.addData("angle error", drive.error);

            telemetry.addData("state", state);
            telemetry.update();
        }

    }
}
