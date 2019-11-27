package com.acmerobotics.opmodes;

import com.acmerobotics.robot.armEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="EncoderTest")

public class EncoderTest extends LinearOpMode {

    @Override
    public void runOpMode(){

        armEncoder arm = new armEncoder();

        arm.init(hardwareMap);

        arm.resetEncoder();

        arm.leaveReset();

        waitForStart();

        while(!isStopRequested()){

            // description: test to make sure your setup and main encoder code is correct

            // outcome: arm should move to the encoder position set (45) and it should hold that position
            arm.runTo(arm.testPosition1, arm.thePower);

            telemetry.addData("target position: ", arm.testPosition1);
            telemetry.addLine();

            telemetry.addData("current position: ", arm.armMotor.getCurrentPosition());
            telemetry.addData("power: ", arm.armMotor.getPower());
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
