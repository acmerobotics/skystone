package com.acmerobotics.opmodes;

import com.acmerobotics.robot.Arm;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="ArmTest")
public class TestArm extends LinearOpMode {

    public boolean isYPressed = false;
    public boolean isAPressed = false;

    public double thePower = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm();

        arm.init(hardwareMap);

        //arm.resetEncoder();

        waitForStart();

        while (!isStopRequested()) {

            if (gamepad1.y) {

//                isYPressed = true;
//            }
//
//            else if (isYPressed) {
//                /////////////////move 20 degrees from resting point
                arm.goToPosition(0);
                //target position should be 31.111

                //isYPressed = false;
            }

//            if (gamepad1.a) {
//
//                isAPressed = true;
//            }
//
//            else if (isAPressed) {
//                /////////////////move 45 degrees from resting point
//                arm.goToPosition(0);
//
//                isAPressed = false;
//            }

            if (gamepad2.left_bumper){
                arm.setHand("close");
            }

            if (gamepad2.right_bumper){
                arm.setHand("open");
            }

            if (gamepad2.x){
                thePower = arm.armMotor.getPower();
                arm.armMotor.setPower(thePower);
            }

            else {
                arm.armMotor.setPower(arm.setMotorPower(gamepad2.left_stick_y));
            }




            telemetry.addData("encoder: ", arm.armMotor.getCurrentPosition());
            telemetry.addData("target_position: ", arm.targetPosition);
            telemetry.addData("power", arm.armMotor.getPower());
            telemetry.addData("tolerance", arm.armMotor.getTargetPositionTolerance());
            telemetry.update();


        }
    }
}