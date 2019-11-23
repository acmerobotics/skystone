package com.acmerobotics.opmodes;

import com.acmerobotics.robot.Arm;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="ArmTeleOp")
public class TestArm extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm();

        arm.init(hardwareMap);

        arm.resetEncoder();

        waitForStart();

        while (!isStopRequested()) {

            if (gamepad1.y){
                /////////////////move 20 degrees from resting point
                arm.goToPosition(0);
                //target position should be 31.111
            }

            if (gamepad1.a){
                /////////////////move 45 degrees from resting point
                arm.goToPosition(1);
                //target position should be 69.99 or approx. 70
            }

            telemetry.addData("encoder: ", arm.armMotor.getCurrentPosition());
            telemetry.addData("target_position: ", arm.targetPosition);
            telemetry.update();


        }
    }
}