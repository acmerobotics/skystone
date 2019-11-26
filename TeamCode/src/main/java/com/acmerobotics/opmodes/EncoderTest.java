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
            arm.runTo(arm.position1, arm.thePower);

            telemetry.addData("set position: ", arm.position1);
            telemetry.addLine();

            telemetry.addData("current position: ", arm.armMotor.getCurrentPosition());
            telemetry.addData("power: ", arm.armMotor.getPower());
            telemetry.update();
        }
    }
}
