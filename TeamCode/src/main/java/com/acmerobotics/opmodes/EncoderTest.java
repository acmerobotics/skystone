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

        armEncoder arm = new armEncoder(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        arm.init();

        arm.resetEncoder();

        arm.leaveReset();

        waitForStart();


        while(!isStopRequested()){

            arm.runTo(armEncoder.targetPosition);


            ////////////////////////
            dashboardTelemetry.addData("target position", arm.armMotor.getTargetPosition());
            dashboardTelemetry.addData("current position", arm.armMotor.getCurrentPosition());
            dashboardTelemetry.addData("pid", arm.armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

            dashboardTelemetry.addData("is busy", arm.armMotor.isBusy());

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
