package com.acmerobotics.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {

    private DcMotorEx leftMotor, rightMotor;
    private Servo leftServo, rightServo;

    private double leftIntakePower, rightIntakePower;

    private double openPos = 0;
    private double closePos = 0;

    public Intake(HardwareMap hardwareMap){
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

    }


    public void internalSetVelocity(double leftIntakePower, double rightIntakePower){
        this.leftIntakePower = leftIntakePower;
        this.rightIntakePower = rightIntakePower;
        leftMotor.setPower(leftIntakePower);
        rightMotor.setPower(rightIntakePower);

    }

    public void openLeft(){
        leftServo.setPosition(openPos);
    }

    public void openRight(){
        rightServo.setPosition(openPos);
    }

    public void closeLeft(){
        leftServo.setPosition(closePos);
    }

    public void closeRight(){
        rightServo.setPosition(closePos);
    }

    public void setIntakePower(double intakePower) {
        internalSetVelocity(intakePower, intakePower);
    }



}
