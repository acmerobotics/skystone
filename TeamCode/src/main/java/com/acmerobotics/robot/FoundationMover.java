package com.acmerobotics.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FoundationMover {

    private Servo servo;

    private static double GRABBING = 0.2;
    private static double STORING = 0.6;


    public FoundationMover(HardwareMap hardwareMap){

        servo = hardwareMap.servo.get("foundationServo");
    }

    public void moveToGrab(){
        servo.setPosition(GRABBING);
    }

    public void moveToStore(){
        servo.setPosition(STORING);
    }




}

