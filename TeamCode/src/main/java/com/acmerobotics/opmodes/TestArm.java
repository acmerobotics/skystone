package com.acmerobotics.opmodes;

import com.acmerobotics.robomatic.util.StickyGamepad;
import com.acmerobotics.robot.PlacingArm;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ArmTeleOp")
public class TestArm extends LinearOpMode {

    public boolean isYPressed = false;
    public boolean isAPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        PlacingArm arm = new PlacingArm(hardwareMap);



        arm.resetEncoder();

        waitForStart();

        while (!isStopRequested()) {


            if (gamepad1.y){
                //arm relocation
                isYPressed = true;

            } else if (isYPressed){
                arm.armRelocationPosition();
                isYPressed = false;

            }

            //telemetry.addData("target pos:" arm.targetPosition);
            telemetry.addData("encoder", Double.toString(arm.checkEncoder()));
            telemetry.update();


        }
    }
}