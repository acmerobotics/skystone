package com.acmerobotics.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private DcMotorEx armMotor;
    private Servo handServo;
    private Servo rotationServo;

    private static final double TICK_COUNT_PER_REVOLUTION = 280;

    private static final double DIAMETER_OF_MOTOR_GEAR = 1;
    private static final double TICKS_PER_INCH_OF_MOTOR_GEAR = TICK_COUNT_PER_REVOLUTION/ DIAMETER_OF_MOTOR_GEAR * Math.PI;

    private static final double DIAMETER_OF_ARM_GEAR = 2;
    private static final double TICKS_PER_INCH_OF_ARM_GEAR = TICK_COUNT_PER_REVOLUTION/ DIAMETER_OF_ARM_GEAR * Math.PI;


    public Arm(HardwareMap hardwareMap){
        armMotor = hardwareMap.get(DcMotorEx.class, "Arm Motor");
        handServo = hardwareMap.get(Servo.class, "Hand Servo");
        rotationServo = hardwareMap.get(Servo.class, "Hand Servo");

        armMotor.setDirection(DcMotorEx.Direction.FORWARD);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }


    public void goToPosition(){

    }

    public int convertToTicks(double angle){
        double distance = getRadianLen(angle, );

        int numToTicks = (int)((distance) * TICKS_PER_INCH_OF_MOTOR_GEAR);
        return numToTicks;
    }

    public double getRadianLen(double angle, double radius){
        // returns radian length (arm movement curve length)

        double i =  (angle/360) * 2 * Math.PI * radius;
        return i;
    }
}
