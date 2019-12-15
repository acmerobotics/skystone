package com.acmerobotics.robot;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class liftEncoder {
    public DcMotorEx liftMotor;

    public static double P = 25;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;

    public PIDFCoefficients coefficients = new PIDFCoefficients(P, I, D, F);

    public double blockHeight = 0;
    public double foundationHeight = 2.5;
    public double extraHeight = 0.5; // will get height greater than target so it doesn't run into it


    private int radius = 1;
    private int TICKS_PER_REV = 280;

    public int targetPosition;

    public void liftEncoder(){
    }


    ////////////////////////////// encoder setup and main methods //////////////////////////////////

    public void init(HardwareMap hardwareMap){

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void resetEncoder(){
        // motor's current encoder position is set as the zero position

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void leaveReset(){
        // motor mode is set to RUN_USING_ENCODER to get motor out of STOP_AND_RESET mode
        // motor will just continue to hold a power of 0

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void runTo(double blocks, double power){

        //TODO might need to setPID() to change PID coefficients

        targetPosition = inchesToTicks(blocks);

        liftMotor.setTargetPosition(targetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setPower(power);
    }


    public void setPID(PIDFCoefficients pidfCoefficients){
        //will set the pid coefficients, it is likely that only p will need to be changed for now
        liftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
    }


    ////////// encoder math, inches to encoder ticks ///////////////


    public double blocksToTotalHeight(double blocks){
        double height = (blocks * blockHeight) + foundationHeight + extraHeight;
        return height;
    }


    public int inchesToTicks(double blocks){

        double targetHeight = blocksToTotalHeight(blocks);

        int ticks = (int) ((targetHeight * TICKS_PER_REV) / (Math.PI * radius * 2));

        return ticks;
    }

}
