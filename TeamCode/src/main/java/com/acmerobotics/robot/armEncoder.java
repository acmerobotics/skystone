package com.acmerobotics.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class armEncoder {

    public DcMotorEx armMotor;

    private static double TICK_COUNT_PER_REVOLUTION = 280;

    public int position1 = 0;
    public int position2 = 0;

    public double thePower = 0.85;

    public void armEncoder(){
    }

    public void init(HardwareMap hardwareMap){
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void resetEncoder(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void leaveReset(){
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void runTo(int position, double power) {
        armMotor.setTargetPosition(position);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(power);
    }


    public void stopMotors(){
        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}

