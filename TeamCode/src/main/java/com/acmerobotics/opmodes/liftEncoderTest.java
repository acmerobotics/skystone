package com.acmerobotics.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.armEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.robot.liftEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="liftEncoderTest")

@Config
public class liftEncoderTest extends LinearOpMode{

    private boolean isDpadUp = false;
    private boolean isDpadDown = false;
    private boolean isDpadLeft = false;

    private int blocks = 0;

    public static int foundation = 150; // 2 in. from ground
    public static int above = 15; // 1 in. from ground

    @Override
    public void runOpMode(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        liftEncoder lift = new liftEncoder(hardwareMap);

        armEncoder arm = new armEncoder(hardwareMap);

        lift.init();
        arm.init();

        lift.resetEncoder(); // sets 0 position
        arm.resetEncoder();

        while(true) {
            lift.tightenLiftString();
            arm.runTo(50);
            lift.goToBottom();

            if(lift.bottomSet){
                break;
            }
        }

        lift.goToStartHeight(); // raise lift so arm is ready for blocks coming in from intake

        arm.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.armMotor.setPower(0.1);

        while(true) {
            if (!lift.liftMotor.isBusy()) {
                arm.resetEncoder();
                break;
            }
        }

        waitForStart();

        while (!isStopRequested()) {

            lift.setPID();

            //TODO do something to stop the lift from getting to and passing its max height

            if (gamepad2.dpad_up) {
                if (isDpadUp == false) {

                    isDpadUp = true;

                    isDpadDown = false;

                        blocks += 1;
                        //lift.runTo(blocks, lift.liftPower, liftEncoder.Mode.BLOCKS);
                        lift.runToBlocks(blocks, lift.liftPower);
                }
            }

            else{
                isDpadUp = false; // allows runTo to be used after every press
            }



            ///////////////// ^^^^^^^^^^^^^^^^^^^^^^^^^ ////////////////////////


            if (gamepad2.dpad_down) {

                if (isDpadDown == false) {

                    isDpadDown = true;

                    isDpadUp = false;

                    blocks -= 1;
                    //lift.runTo(blocks, lift.liftPower, liftEncoder.Mode.BLOCKS);
                    lift.runToBlocks(blocks, lift.liftPower);
                }
            }

            else{
                isDpadDown = false;
            }


            //TODO why does lift move during init when this code is uncommented?
//            if (gamepad2.dpad_left) {
//
//                if (isDpadLeft == false) {
//
//                    isDpadLeft = true;
//
//                    isDpadUp = false;
//                    isDpadDown = false;
//                }
//            }
//
//
//            ///////// resets lift height ////////
//            if (isDpadLeft = true) {
//
//                blocks = 0;
//                lift.runTo(0, lift.liftPower, liftEncoder.Mode.DIRECT);
//
//                isDpadLeft = false;
//                isDpadUp = false;
//                isDpadDown = false;
//           }


            ///////////// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ //////////////////


            if (gamepad2.a){
                //starting height, arm at rest (at hard stop)

                //hand will grab block

                lift.runTo(liftEncoder.startHeight, lift.liftPower, liftEncoder.Mode.DIRECT);

                arm.runTo(10);

                if (arm.armMotor.isBusy() == false){
                    arm.armMotor.setPower(0);
                }
            }


            if (gamepad2.x){
                // lift goes to bottom, arm moves to a position where it is greater than the foundation (2 in) and 1 in above f
                // the foundation so it can block a block

                // allows robot to go under bridge

                int blockLifted = foundation + above;

                arm.runTo(blockLifted);    // moves 2 in. plus 1 in. above ground
                lift.runTo(0, lift.liftPower, liftEncoder.Mode.DIRECT);
            }


            if (gamepad2.b){
                // arm moves down 1 in. to place block on foundation

                // hand releases block

                int blockPlaced = foundation;

                arm.runTo(blockPlaced);
            }


            if (gamepad2.right_bumper){
                arm.setHand("close");
            }

            if (gamepad2.left_bumper){
                arm.setHand("open");
            }


            dashboardTelemetry.addData("current position ", lift.liftMotor.getCurrentPosition());
            dashboardTelemetry.addData("target position ", lift.liftMotor.getTargetPosition());
            dashboardTelemetry.addData("pid coefficients", lift.liftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            dashboardTelemetry.addData("blocks", blocks);


            dashboardTelemetry.addData("arm current position", arm.armMotor.getCurrentPosition());
            dashboardTelemetry.addData("arm target position", arm.armMotor.getTargetPosition());

            dashboardTelemetry.update();

            ///////////////////////////////////////////////////////////////////////////////
            // Moves lift back to bottom prevents positions from being changed and
            // keeps everything the same as the time before the robot was turned on.

            // also moves arm up so it doesn't break the hand as it is going down

            //////// arm is moved to 90 degrees
            //////// lift.runTo(0, lift.liftPower, liftEncoder.Mode.DIRECT);

        }
    }
}
