package com.acmerobotics.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.robot.liftEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="liftEncoderTest")

@Config
public class liftEncoderTest extends LinearOpMode{

    public boolean isDpadUp = false;
    public boolean isDpadDown = false;
    public boolean isDpadLeft = false;

    public int blocks = 0;
    public double liftPower = 1;
    public boolean atZeroBlocks = true; // true

    @Override
    public void runOpMode(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        liftEncoder lift = new liftEncoder(hardwareMap);

        lift.init();

        lift.resetEncoder();

        while (!isStopRequested()) {

            //TODO do something to stop the lift from getting to and passing its max height

            if (gamepad2.dpad_up) {
                if (isDpadUp == false) {

                    isDpadUp = true;

                    isDpadDown = false;
                }
            }
            ////////// code is activated when isDpadUp is true ///////////////

            if (isDpadUp == true) {
                if (atZeroBlocks) {

                    lift.runTo(blocks, liftPower);
                    atZeroBlocks = false;
                } else {

                    blocks += 1;
                    lift.runTo(blocks, liftPower);
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

            ////////// code is activated when isDpadDown is true ///////////////

            if (isDpadDown == true) {

                blocks -= 1;
                lift.runTo(blocks, liftPower);

                isDpadDown = false;
            }

            ///////////// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ //////////////////


            //TODO why does lift move during init when this code is uncommented?
//            if (gamepad2.dpad_left) {
//
//                if (isDpadLeft == false) {
//                    isDpadLeft = true;
//                    isDpadUp = false;
//                    isDpadDown = false;
//                }
//            }
//
//
//            ///// code is activated when isDpadLeft is true ///////
//
//            ///////// resets lift height ////////
//            if (isDpadLeft = true) {
//
//                blocks = 0;
//                lift.runTo(blocks, liftPower);
//
//                double resetToBase = -1 * (lift.foundationHeight + lift.extraHeight) / lift.blockHeight; // will make the
//                // target position equal to 0
//                lift.runTo(resetToBase, liftPower);
//
//                isDpadLeft = false;
//                isDpadUp = false;
//                isDpadDown = false;
//           }


            dashboardTelemetry.addData("current position ", lift.liftMotor.getCurrentPosition());
            dashboardTelemetry.addData("target position ", lift.liftMotor.getTargetPosition());
            dashboardTelemetry.addData("pid coefficients", lift.liftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));


            dashboardTelemetry.addData("dpad up", isDpadUp);
            dashboardTelemetry.addData("dpad down", isDpadDown);
            dashboardTelemetry.addData("dpad left", isDpadLeft);
            dashboardTelemetry.update();
        }
    }
}
