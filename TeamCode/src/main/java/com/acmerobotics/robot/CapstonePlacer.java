package com.acmerobotics.robot;


import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CapstonePlacer  {

    private Servo servo;

    public static final double PLACING = 0.75;
    public static final double STORING = 0.14;

    public CapstonePlacer(HardwareMap hardwareMap){

        servo = hardwareMap.servo.get("capstoneServo");

    }

    public void moveToPlace(){
        servo.setPosition(PLACING);
    }

    public void moveToStore(){
        servo.setPosition(STORING);
    }

}
