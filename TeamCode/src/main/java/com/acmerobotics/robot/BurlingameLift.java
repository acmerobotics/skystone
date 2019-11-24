package com.acmerobotics.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class BurlingameLift {

    private DcMotorEx liftMotor;

    private int initPosition = 0;
    private int bottomPosition = 975;
    private int intakePosition = -680;

    public static double RADIUS = 1;

    public BurlingameLift(HardwareMap hardwareMap){

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

    }

    public void init(){
        liftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void resetEncoder(){
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }


    public double checkEncoder(){
        return liftMotor.getCurrentPosition();
    }


    public void setMotorEncoders(int distance){
        liftMotor.setTargetPosition(distance);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1);
    }

    public void goToBottom(){
        setMotorEncoders(bottomPosition);
    }

    public void goToIntake(){
        setMotorEncoders(intakePosition);
    }

    public void driverControlled(double power){
        liftMotor.setPower(1);
    }





}
