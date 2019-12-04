package com.acmerobotics.opmodes;

import com.acmerobotics.robot.armEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="EncoderTest")

public class EncoderTest extends LinearOpMode {

    @Override
    public void runOpMode(){

        armEncoder arm = new armEncoder();

        arm.init(hardwareMap);
        telemetry.addLine(arm.initialized);//

        arm.resetEncoder();
        telemetry.addLine(arm.reset);//

        arm.leaveReset();
        telemetry.addLine(arm.leave);//

        telemetry.addLine();
        telemetry.addLine("waiting for start");//

        telemetry.update();

        waitForStart();

        telemetry.clear();//
        telemetry.update();//


        while(!isStopRequested()){

            // description: test to make sure your setup and main encoder code is correct

            // outcome: arm should move to the encoder position set (45) and it should hold that position
            arm.runTo(arm.testPosition1, arm.thePower);

            telemetry.addData("target position: ", arm.testPosition1);
            telemetry.addLine();

            if (arm.armMotor.isBusy()){
                telemetry.addLine(arm.runningTo);
                telemetry.addData("run mode", arm.armMotor.getMode());
            }

            if (arm.armMotor.getCurrentPosition() == arm.testPosition1){
                telemetry.addLine(arm.positionReached);
            }

            telemetry.addData("current position: ", arm.armMotor.getCurrentPosition());
            telemetry.addData("power: ", arm.armMotor.getPower());
            telemetry.addLine();

            telemetry.addData("PID Coefficients", arm.armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

            telemetry.update();


            // ^^^^can delete when testing is successful and uncomment the code below


            /*

            // TODO get the real ticks per rev value or code will NOT work

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
