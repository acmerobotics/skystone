package com.acmerobotics.robot;

import com.acmerobotics.util.PIDBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PIDBaseSubsystem {

    public DcMotorEx motor1;
    public DcMotorEx motor2;

    public PIDBase motor1PID = new PIDBase();
    public PIDBase motor2PID = new PIDBase();

    public PIDBaseSubsystem(HardwareMap hardwareMap){

        // drive motors
        motor1 = hardwareMap.get(DcMotorEx.class, "m0");
        motor2 = hardwareMap.get(DcMotorEx.class, "m1");

        // each motor has to be added to its own PID Base so they can independently interact with each other
        motor1PID.addMotor(motor1);
        motor2PID.addMotor(motor2);
    }

    public void init(){
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void motor1MoveInches(){
        // turn m1 on rev

        int target = motor1PID.unitsToTicks(560, 4, 12.56); // unit is in inches because diameter is in inches

        motor1PID.runTo(target);
    }

    public void moveBoth(){
        // turn m1 and m2 each 2 rev

        motor1PID.runTo(1200);
        motor2PID.runTo(1200);
    }


}
