package com.acmerobotics.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {

    private DcMotorEx intakeMotor;
    public Servo leftServo, rightServo;


    private double leftOpen = 0.607;
    private double leftClose = 0.95;

    private double rightOpen = 0.45;
    private double rightClose = 0.12;

    public double LfullyOpen = 0.35;
    private double RfullyOpen = 0.685;

    public Intake(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");


    }

    public void setIntakePower(double intakePower) {
        intakeMotor.setPower(intakePower);
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

    public void leftFullyOpen(){
        leftServo.setPosition(LfullyOpen);
    }

    public void rightFullyOpen(){
        rightServo.setPosition(RfullyOpen);
    }



}
