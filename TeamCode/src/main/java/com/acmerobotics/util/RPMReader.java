package com.acmerobotics.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RPMReader {


    public double TICKS_PER_REVOLUTION = 0;

    public ElapsedTime time;

    public DcMotor motor;

    public double Time = 0.001; // set a non 0 value to prevent any initial div by 0
    public double ticks = 0;

    public double lastTicks = 0;
    public double lastTime = 0;


    public RPMReader(HardwareMap hardwareMap, DcMotor motor, double TICKS_PER_REVOLUTION){
        this.TICKS_PER_REVOLUTION = TICKS_PER_REVOLUTION;

        this.motor = motor;

        this.motor = hardwareMap.get(DcMotor.class, "motor");

        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        time = new ElapsedTime();
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

    // multiply by 60 then divide ticks vel by total ticks in one rotation to get the rpm of 1 min
    public double getRPM(){
        double ticks = ticksPerSec() * 60;

        double RPM = ticks / TICKS_PER_REVOLUTION;

        return RPM;
    }

}
