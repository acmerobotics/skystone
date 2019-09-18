package com.acmerobotics.practice;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")
public class TeleOp extends LinearOpMode {

    private Servo suction;
    private DcMotor arm;
    private DcMotor elevator;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap);

        arm = hardwareMap.dcMotor.get("arm_motor");
        elevator = hardwareMap.dcMotor.get("elevator");
        suction = hardwareMap.servo.get("suction");


        arm.setPower(0);
        elevator.setPower(0);
        suction.setPosition(0);


        while (!isStopRequested()){
            drive.setPower(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x);



        }
    }
}
