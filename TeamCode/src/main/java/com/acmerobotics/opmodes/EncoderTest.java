package com.acmerobotics.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.armEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="EncoderTest")

@Config
public class EncoderTest extends LinearOpMode {


    @Override
    public void runOpMode(){

        armEncoder arm = new armEncoder(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        arm.init();

        arm.resetEncoder();

        arm.leaveReset();

        waitForStart();


        while(!isStopRequested()){

            //arm.encoderRunTo(arm.testAngle);


            //////////////////////////// ARM //////////////////////////

            if (gamepad2.a){
                arm.runTo(arm.grabPosition);
            }

            if (gamepad2.x){
                arm.runTo(arm.underBridge);
            }

            if (gamepad2.y){
                arm.runTo(arm.liftPosition);
            }

            if (gamepad2.b){
                arm.runTo(arm.placePosition);
            }

            ///////////////////////////////////////////////////////////////

            ////////////////////////////// HAND ////////////////////////////

            if (gamepad2.right_bumper){
                arm.setHand("open");
            }

            if (gamepad2.left_bumper){
                arm.setHand("close");      //TODO why won't hand close?
            }

            //////////////////////////////////////////////////////////////////

            dashboardTelemetry.addData("target position", arm.armMotor.getTargetPosition());
            dashboardTelemetry.addData("current position", arm.armMotor.getCurrentPosition());
            dashboardTelemetry.addData("pid", arm.armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

            dashboardTelemetry.update();


        }
    }
}
