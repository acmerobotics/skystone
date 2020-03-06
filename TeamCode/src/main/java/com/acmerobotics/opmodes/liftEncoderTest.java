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

    public static int targetPosition = 1980;
    public static double thePower = 0.5;

    @Override
    public void runOpMode(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        liftEncoder lift = new liftEncoder(hardwareMap);
        armEncoder arm = new armEncoder(hardwareMap);

        lift.init();

        lift.resetEncoder();

        waitForStart();

        lift.resetEncoder();
        arm.resetEncoder();

        while (!isStopRequested()) {

//            lift.setPID();
//
//            lift.runTo(targetPosition, thePower);

            arm.runTo(80);

            lift.tightenLiftString();

            lift.goToBottom();

            telemetry.addData("current position ", lift.liftMotor1.getCurrentPosition());
            telemetry.addData("target position ", lift.liftMotor1.getTargetPosition());

            telemetry.addLine();

            telemetry.addData("bottom set", lift.bottomSet);
            telemetry.addData("arm position", arm.armMotor.getCurrentPosition());

            telemetry.update();

            ///////////////////////////////////////////////////////////////////////////////
            // Moves lift back to bottom prevents positions from being changed and
            // keeps everything the same as the time before the robot was turned on.

            // also moves arm up so it doesn't break the hand as it is going down

            //////// arm is moved to 90 degrees
            //////// lift.runTo(0, lift.liftPower, liftEncoder.Mode.DIRECT);

        }
    }
}
