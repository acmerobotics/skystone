package com.acmerobotics.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSimple {

    public DcMotorEx armMotor;
    private Servo handServo;
    private Servo rotationServo;

    private double handOpenPos = 0.78;
    private double handClosePos = 0.25;
    private double rotateCenter = 0.53;
    private double grabTheCapStone = 0.25;

    public double stablePower = 0.4;

    public ArmSimple(HardwareMap hardwareMap){
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        handServo = hardwareMap.get(Servo.class, "handServo");
        rotationServo = hardwareMap.get(Servo.class, "rotationServo");


    }


    public void init(){

        armMotor.setDirection(DcMotorEx.Direction.FORWARD);
        armMotor.setTargetPosition(0);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationServo.setPosition(rotateCenter);
        armMotor.setPower(stablePower);


    }


    public void setHand(String position){
        // take in close or open then set servo position accordingly

        if (position.equals("open")){
            //open hand
            handServo.setPosition(handOpenPos);
        }

        if (position.equals("close")){
            //close hand
            handServo.setPosition(handClosePos);
        }
    }

    public void grabTheCapStone(){
        handServo.setPosition(grabTheCapStone);
    }


    public void setMotorPower(double power){
        armMotor.setPower(power * - 0.80);
    }

}