package com.acmerobotics.RobomaticTesting;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config

public class roboIntake extends Subsystem {

    private DcMotorEx intakeMotor;
    public Servo leftServo, rightServo;


    private double leftOpen = 0.7;
    private double leftClose = 0.99;

    private double rightOpen = 0.353;
    private double rightClose = 0.001;

    public double LfullyOpen = 0.47;
    private double RfullyOpen = 0.57;

    public roboIntake(Robot robot){
        super("Intake");

        intakeMotor = robot.getMotor("intakeMotor");

        leftServo = robot.getServo("leftServo");
        rightServo = robot.getServo("rightServo");
    }
    @Override
    public void update(Canvas overlay){

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
