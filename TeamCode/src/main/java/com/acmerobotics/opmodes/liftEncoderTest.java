package com.acmerobotics.opmodes;

import android.util.Log;

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

    public static int targetPosition = 1980;
    public static double thePower = 0.5;

    public static int initH = 1528;
    public static int block = 1204;

    @Override
    public void runOpMode(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        liftEncoder lift = new liftEncoder(hardwareMap);
        armEncoder arm = new armEncoder(hardwareMap);

        lift.init();

        lift.resetEncoder();

        waitForStart();

        while (!isStopRequested()) {
            lift.PController();

            if (gamepad2.x){
                lift.setPosition(0);
            }

            if (gamepad2.a){
                lift.setPosition(initH);
            }

            if (gamepad2.b){
                lift.setPosition(block);
            }

            //////////////////////////////////////////////


            dashboardTelemetry.addData("current 1", lift.liftMotor1.getCurrentPosition());
            dashboardTelemetry.addData("current 2", lift.liftMotor2.getCurrentPosition());

            dashboardTelemetry.addLine();

            dashboardTelemetry.addData("target", lift.setPoint);

            dashboardTelemetry.addLine();

            dashboardTelemetry.addData("error", lift.error);

            dashboardTelemetry.update();

//            telemetry.addData("current position ", lift.liftMotor1.getCurrentPosition());
//            telemetry.addData("target position ", lift.liftMotor1.getTargetPosition());
//
//            telemetry.addLine();
//
//            telemetry.addData("bottom set", lift.bottomSet);
//            telemetry.addData("arm position", arm.armMotor.getCurrentPosition());
//
//            telemetry.update();
//
//            Log.d("FTC-DBG", "lift1 pos: " + lift.liftMotor1.getCurrentPosition());
//            Log.d("FTC-DGB", "lift1 target" + lift.liftMotor1.getCurrentPosition());
//            Log.d("FTC-DBG", "lift1 power" + lift.liftMotor1.getPower());
//
//            Log.d("FTC-DBG", "lift2 pos: " + lift.liftMotor2.getCurrentPosition());
//            Log.d("FTC-DGB", "lift2 target" + lift.liftMotor2.getCurrentPosition());
//            Log.d("FTC-DBG", "lift2 power" + lift.liftMotor2.getPower());

            ///////////////////////////////////////////////////////////////////////////////
            // Moves lift back to bottom prevents positions from being changed and
            // keeps everything the same as the time before the robot was turned on.

            // also moves arm up so it doesn't break the hand as it is going down

            //////// arm is moved to 90 degrees
            //////// lift.runTo(0, lift.liftPower, liftEncoder.Mode.DIRECT);

        }
    }
}
