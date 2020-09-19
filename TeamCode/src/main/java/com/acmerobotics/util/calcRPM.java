package com.acmerobotics.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class calcRPM {

    public double TICKS_PER_REVOLUTION = 0;

    public ElapsedTime time;

    public DcMotor motor;

    public double accumulatedTicks = 0; // this will let me reset the ticks without actually resetting the tick value which might be used else where

    public calcRPM(HardwareMap hardwareMap){
        time = new ElapsedTime();
        motor = hardwareMap.get(DcMotor.class, "motor");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // get accumulating encoder ticks
    public double getTicks() { // unnessary function (using it for concept)
        return motor.getCurrentPosition();
    }

    // keep track ticks per sec
    public double ticksPerSec(){
        time.reset();

        double totalTicks = 0;

        while (time.seconds() <= 1){
            // subtracting the accumulatedTicks will act as a way to reset the tick count every second
            totalTicks = getTicks() - accumulatedTicks;
        }

        accumulatedTicks += getTicks();

        return totalTicks;
    }

    // divide accumulated ticks by total ticks in one rotation to get the rpm of 1 min
    public double getRPM(){
        double ticks = ticksPerSec() * 60;

        double RPM = ticks / TICKS_PER_REVOLUTION;

        return RPM;
    }

}
