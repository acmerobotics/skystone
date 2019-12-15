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

    public int currentPosition = 0;

    @Override
    public void runOpMode(){

        armEncoder arm = new armEncoder();

        FtcDashboard dashboard = FtcDashboard.getInstance();

        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        arm.init(hardwareMap);

        arm.resetEncoder();

        arm.leaveReset();

        waitForStart();


        while(!isStopRequested()){

            // description: test to make sure your setup and main encoder code is correct

            // outcome: arm should move to the encoder position set (45) and it should hold that position

            currentPosition = arm.armMotor.getCurrentPosition();

            arm.runTo(armEncoder.targetPosition, arm.thePower);

            ////////////////////////
            dashboardTelemetry.addData("target position", armEncoder.targetPosition);
            dashboardTelemetry.addData("current position", currentPosition);
            dashboardTelemetry.addData("runTo() target position", arm.positionInRunTo); //if my theory is correct then
                                                                                                //by adding targetPos. and curPos.
                                                                                                 //then you should get pos.InRunTo

            dashboardTelemetry.update();
            ////////////////////////


            // ^^^^can delete when testing is successful and uncomment the code below


            /*

            // TODO get the real ticks per rev value

            // description/ outcome: arm will raise to a 45 degree angle from the init position


            arm.encoderRunTo(arm.testAngle);

            telemetry.addData("angle: ", arm.testAngle);
            telemetry.addData("target position: ", arm.testEncoderPosition);
            telemetry.addLine();

            telemetry.addData("current position: ", arm.armMotor.getCurrentPosition());
            telemetry.addData("power: ", arm.armMotor.getPower());
            telemetry.update();

             */
        }
    }
}
