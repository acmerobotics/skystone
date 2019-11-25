package com.acmerobotics.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {

    private DcMotorEx leftMotor, rightMotor;
    private Servo leftServo, rightServo;

    private double leftIntakePower, rightIntakePower;

    private double leftOpen = 0.25;
    private double leftClose = 0.98;
    private double rightOpen = 0.95;
    private double rightClose = 0.16;
    private double leftOutOfWay = 0;
    private double rightOutOfWay = 1;

    public Intake(HardwareMap hardwareMap){
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        rightMotor.setDirection(DcMotorEx.Direction.REVERSE);

    }


    public void internalSetVelocity(double leftIntakePower, double rightIntakePower){
        this.leftIntakePower = leftIntakePower;
        this.rightIntakePower = rightIntakePower;
        leftMotor.setPower(leftIntakePower);
        rightMotor.setPower(rightIntakePower);

    }

    public void leftOpen(){
        leftServo.setPosition(leftOpen);
    }

    public void rightOpen(){
        rightServo.setPosition(rightOpen);
    }

    public void leftClose(){
        leftServo.setPosition(leftClose);
    }

    public void rightClose(){
        rightServo.setPosition(rightClose);

    }

    public void leftOpenAllWay(){
        leftServo.setPosition(leftOutOfWay);
    }

    public void rightOpenAllWay(){
        rightServo.setPosition(rightOutOfWay);
    }

    public void setIntakePower(double intakePower) {
        internalSetVelocity(intakePower, intakePower);
    }



}
