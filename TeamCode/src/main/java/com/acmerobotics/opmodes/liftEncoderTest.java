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
    private boolean atZeroBlocks = true;

    @Override
    public void runOpMode(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        liftEncoder lift = new liftEncoder(hardwareMap);

        armEncoder arm = new armEncoder();

        lift.init();
        arm.init(hardwareMap);

        lift.resetEncoder(); // sets 0 position
        arm.resetEncoder();

        lift.goToStartHeight(); // raise lift so arm is ready for blocks coming in from intake

        while (!isStopRequested()) {

            //TODO do something to stop the lift from getting to and passing its max height

            if (gamepad2.dpad_up) {
                if (isDpadUp == false) {

                    isDpadUp = true;

                    isDpadDown = false;
                }
            }


            if (isDpadUp == true) {
                if (atZeroBlocks) {

//                    lift.runTo(blocks, lift.liftPower, liftEncoder.Mode.BLOCKS);
//                    atZeroBlocks = false;
//                } else {

                    blocks -= 50;
                    lift.runTo(blocks, lift.liftPower, liftEncoder.Mode.DIRECT);
                }

                isDpadUp = false; // allows runTo to be used after every press

            }

            ///////////////// ^^^^^^^^^^^^^^^^^^^^^^^^^ ////////////////////////


            if (gamepad2.dpad_down) {

                if (isDpadDown == false) {

                    isDpadDown = true;

                    isDpadUp = false;
                }
            }


            if (isDpadDown == true) {

                blocks -= 1;
                lift.runTo(blocks, lift.liftPower, liftEncoder.Mode.BLOCKS);

                isDpadDown = false;
            }

            ///////////// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ //////////////////


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

            arm.runTo(armEncoder.targetPosition, arm.thePower);


            dashboardTelemetry.addData("current position ", lift.liftMotor.getCurrentPosition());
            dashboardTelemetry.addData("target position ", lift.liftMotor.getTargetPosition());
            dashboardTelemetry.addData("mode", lift.liftMotor.getMode());
            dashboardTelemetry.addData("is busy", lift.liftMotor.isBusy());
            dashboardTelemetry.addData("mode", lift.mode);
            dashboardTelemetry.addData("pid coefficients", lift.liftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            dashboardTelemetry.addData("hall effect sensor", lift.isAtBottom());

            dashboardTelemetry.update();
        }

        // Moves lift back to bottom after stop is pressed, prevents positions from being changed and
        // keeps everything the same as the time before the robot was turned on.

        // NOTE: might have to add break statement to exit while loop and run the following code.

        //if (!isStarted()) {
            lift.runTo(0, lift.liftPower, liftEncoder.Mode.BOTTOM);
        //}
    }
}
