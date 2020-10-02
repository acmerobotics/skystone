package com.acmerobotics.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class calcRPM {

    public double TICKS_PER_REVOLUTION = 560; // Rev 20:1

    public ElapsedTime time;

    public DcMotorEx motor;

    public double Time = 0.001;
    public double ticks = 0;

    public double lastTicks = 0;
    public double lastTime = 0;


    public calcRPM(HardwareMap hardwareMap){
        time = new ElapsedTime();
        motor = hardwareMap.get(DcMotorEx.class, "motor");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // get accumulating encoder ticks
    public double getTicks() { // unnessary function (using it for concept)
        return motor.getCurrentPosition();
    }

    // keep track ticks per sec
    public double ticksPerSec(){

        if (Time > 1){
            lastTicks = motor.getCurrentPosition();
            lastTime = time.seconds();
        }

        ticks = motor.getCurrentPosition() - lastTicks;
        Time = time.seconds() - lastTime;

        double tickVelocity = ticks / Time;

        return tickVelocity;
    }

    // divide accumulated ticks by total ticks in one rotation to get the rpm of 1 min
    public double getRPM(){
        double ticks = ticksPerSec() * 60;

        double RPM = ticks / TICKS_PER_REVOLUTION;

        return RPM;
    }

}
