package com.acmerobotics.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.armEncoder;
import com.acmerobotics.robot.liftEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="EncoderTest")

@Config
public class EncoderTest extends LinearOpMode {

    public static int armTargetPosition = 0;

    @Override
    public void runOpMode(){

        armEncoder arm = new armEncoder(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        arm.init();

        arm.resetEncoder();

        arm.leaveReset();

        waitForStart();

        arm.resetEncoder();

        while(!isStopRequested()){

            //arm.encoderRunTo(arm.testAngle);

            //////////////////////////// ARM //////////////////////////

            arm.runTo(armTargetPosition);  //from 0, 2 in the encoder value is 50

            //arm.moveTo(desiredInches);

            ///////////////////////////////////////////////////////////////



            dashboardTelemetry.addData("target position", arm.armMotor.getTargetPosition());
            dashboardTelemetry.addData("current position", arm.armMotor.getCurrentPosition());
            dashboardTelemetry.addData("pid", arm.armMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));

            dashboardTelemetry.update();


        }
    }
}
