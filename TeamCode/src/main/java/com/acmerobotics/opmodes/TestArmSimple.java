package com.acmerobotics.opmodes;

import com.acmerobotics.robot.ArmSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="TestArmSimple")
public class TestArmSimple extends LinearOpMode{



    @Override
    public void runOpMode() throws InterruptedException {
        ArmSimple arm = new ArmSimple(hardwareMap);

       // arm.init(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {

            if (gamepad2.left_bumper){
                arm.setHand("open");
            }

            if (gamepad2.right_bumper){
                arm.setHand("close");
            }
/*
            if (gamepad2.x){
                double thePower = arm.armMotor.getPower();
                arm.armMotor.setPower(thePower);
            }

            else {
                arm.armMotor.setPower(arm.setMotorPower(gamepad2.left_stick_y));
            }

            */

            telemetry.addData("power", arm.armMotor.getPower());
            telemetry.update();
        }
    }
}
