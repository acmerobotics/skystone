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

            //arm.encoderRunTo(arm.testAngle);

            arm.runTo(arm.testEncoderPosition);

            dashboardTelemetry.addData("target position", arm.armMotor.getTargetPosition());
            dashboardTelemetry.addData("current position", arm.armMotor.getCurrentPosition());
            dashboardTelemetry.addData("pid", arm.armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

//            dashboardTelemetry.addData("ticksPerInch", arm.ticksPerInch);
//            dashboardTelemetry.addData("arcLength", arm.arcLength);
//            dashboardTelemetry.addData("arcInchesToTicks", arm.arcInchesToTicks );
//            dashboardTelemetry.addData("toArmGearTicks", arm.toArmGearTicks);
//            dashboardTelemetry.addData("setEncoderTicks", arm.setEncoderTicks);
//            dashboardTelemetry.addData("finalPosition", arm.finalPosition);

            dashboardTelemetry.update();


        }
    }
}
