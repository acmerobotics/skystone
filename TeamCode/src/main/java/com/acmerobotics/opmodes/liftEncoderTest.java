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

    public static int targetPosition = 0;
    public static double thePower = 0;

    @Override
    public void runOpMode(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        liftEncoder lift = new liftEncoder(hardwareMap);

        lift.init();

        lift.resetEncoder();

        waitForStart();

        lift.resetEncoder();

        while (!isStopRequested()) {

            lift.setPID();

            lift.runTo(targetPosition, thePower);


            dashboardTelemetry.addData("current position ", lift.liftMotor.getCurrentPosition());
            dashboardTelemetry.addData("target position ", lift.liftMotor.getTargetPosition());
            dashboardTelemetry.addData("pid coefficients", lift.liftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

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
