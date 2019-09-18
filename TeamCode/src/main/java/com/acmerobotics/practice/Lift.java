package com.acmerobotics.practice;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@TeleOp(name="Lift")
public class Lift extends LinearOpMode {

    private Servo suction;
    private DcMotor arm;
    private DcMotor elevator;


    @Override
    public void runOpMode() throws InterruptedException {

        arm = hardwareMap.dcMotor.get("arm_motor");
        elevator = hardwareMap.dcMotor.get("elevator");
        suction = hardwareMap.servo.get("suction");


        arm.setPower(0);
        elevator.setPower(0);

        while (!isStopRequested()){
                elevator.setPower(gamepad1.left_stick_y);

                arm.setPower(gamepad1.right_stick_y);

            if (gamepad1.a){
                suction.setPosition(0.6);
            }

            if(gamepad1.b){
                suction.setPosition(0.1);
            }


        }


    }










}

