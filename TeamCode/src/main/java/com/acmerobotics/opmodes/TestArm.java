package com.acmerobotics.opmodes;

import com.acmerobotics.robot.PlacingArm;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ArmTeleOp")
public class TestArm extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        PlacingArm arm = new PlacingArm(hardwareMap);

        arm.resetEncoder();

        waitForStart();

        while (!isStopRequested()) {
            telemetry.addData("encoder", Double.toString(arm.checkEncoder()));
            telemetry.update();

            if(gamepad1.y){
                arm.armRelocationPosition();
            }


        }
    }
}