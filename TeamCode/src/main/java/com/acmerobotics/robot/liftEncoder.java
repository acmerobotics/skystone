package com.acmerobotics.robot;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class liftEncoder {
    public DcMotorEx liftMotor;

    public static double P = 10;
    public static double I = 0.05;
    public static double D = 0;
    public static double F = 0;

    public static PIDFCoefficients coefficients = new PIDFCoefficients(P, I, D, F);

    public double blockHeight = 5;
    public double foundationHeight = 2.5;
    public double extraHeight = 0.5; // will get height greater than target so it doesn't run into it


    private int radius = 1;
    private int TICKS_PER_REV = 280;

    public int targetPosition = 0;

    public liftEncoder(HardwareMap hardwareMap){
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

    }


    ////////////////////////////// encoder setup and main methods //////////////////////////////////

    public void init(){

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftMotor.setTargetPosition(0);
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void resetEncoder(){
        // motor's current encoder position is set as the zero position

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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


    //TODO test and adjust height math (doesn't seem to be correct)

    public double blocksToTotalHeight(double blocks){
        double height = (blocks * blockHeight) + foundationHeight + extraHeight;
        return (height * -1);
    }


    public int inchesToTicks(double blocks){

        double targetHeight = blocksToTotalHeight(blocks);

        int ticks = (int) ((targetHeight * TICKS_PER_REV) / (Math.PI * radius * 2));

        return ticks;
    }

}
