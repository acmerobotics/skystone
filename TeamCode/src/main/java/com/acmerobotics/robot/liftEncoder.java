package com.acmerobotics.robot;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class liftEncoder {
    public DcMotorEx liftMotor;
    private DigitalChannel bottomHallEffect;


    public double blockHeight = 5;
    public double foundationHeight = 2;
    public double extraHeight = 0.5; // will get height greater than target so it doesn't run into it
    public static int startHeight = 1870;
    public static int bottomPosition = 0;

    public  boolean stringTightened = false;
    public boolean bottomSet = false;

    //////////////////////
    public int blockPosition = 0;

    public static int blockEncoderHeight = 1530; //1130


    private int radius = 1;
    private int TICKS_PER_REV = 280;

    public double liftPower = 1;

    public static PIDFCoefficients coefficients = new PIDFCoefficients(10, 0.05, 0, 0, MotorControlAlgorithm.LegacyPID);


    public liftEncoder(HardwareMap hardwareMap){
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        bottomHallEffect = hardwareMap.digitalChannel.get("bottomHallEffect");

        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    ////////////////////////////// encoder setup and main methods //////////////////////////////////

    public void init(){

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftMotor.setTargetPosition(0);
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }


    public void resetEncoder(){
        // motor's current encoder position is set as the zero position

        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void runTo(int position, double power){

                setPID();

                liftMotor.setTargetPosition(position);
                liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                liftMotor.setPower(power);

    }


    public void runToBlocks(int position, double power){

        blockPosition = (blockEncoderHeight * position);// + startHeight;

        liftMotor.setTargetPosition(blockPosition);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(power);
    }


    public void runToIncrement(int position){
        int targetPosition = liftMotor.getCurrentPosition() + position;

        runTo(targetPosition, liftPower);
    }


    public void goToStartHeight(){
        runTo(startHeight, liftPower);
    }

    public void tightenLiftString(){
        int tightPosition = 150;
        if(stringTightened == false) {
            runTo(tightPosition, liftPower);

            if (!liftMotor.isBusy()) {
                stringTightened = true;
            }
        }
    }


    public void goToBottom(){
        boolean isAtBottom = isAtBottom();
        if(bottomSet == false && stringTightened == true) {
            if (!isAtBottom) {
                bottomPosition = liftMotor.getCurrentPosition();
                bottomPosition -= 5;
                runTo(bottomPosition, liftPower);
            } else {
                bottomPosition = 0;
                liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                bottomSet = true;
            }
        }
    }


    ////////// encoder math, inches to encoder ticks ///////////////


    //TODO test and adjust height math (doesn't seem to be correct)

    public double blocksToTotalHeight(int blocks){
        double height = (blocks * blockHeight) + foundationHeight + extraHeight;
        return (height);
    }


    public int inchesToTicks(int blocks){

        double targetHeight = blocksToTotalHeight(blocks);

        int ticks = (int) ((targetHeight * TICKS_PER_REV) / (Math.PI * radius * 2));

        return ticks;
    }


    /////////////////////// other methods //////////////////////////

    public boolean isAtBottom(){
        boolean state = bottomHallEffect.getState();
        boolean inverseState = false;

        if (state == true){
            inverseState = false;
        }
        if (state == false){
            inverseState = true;
        }

        return inverseState; // is at bottom
    }

    /////////////////////////////////////////////////


    public void setPID(){//PIDFCoefficients coefficients){
        liftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, coefficients);
    }

}
