package com.acmerobotics.robot;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class liftEncoder {
    public DcMotorEx liftMotor;
    private DigitalChannel bottomHallEffect;


    public double blockHeight = 5;
    public double foundationHeight = 2;
    public double extraHeight = 0.5; // will get height greater than target so it doesn't run into it
    public static int startHeight = 0; ////////////////TODO get starting height /////////////////////


    private int radius = 1;
    private int TICKS_PER_REV = 280;

    public int targetPosition = 0;
    public double liftPower = 1;

    public enum Mode{
        BLOCKS,
        BOTTOM,
        DIRECT
    }

    public Mode mode;

    public liftEncoder(HardwareMap hardwareMap){
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        bottomHallEffect = hardwareMap.digitalChannel.get("bottomHallEffect");
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


    public void goToStartHeight(){
        runTo(startHeight, liftPower, liftEncoder.Mode.DIRECT);
    }


    public void runTo(int position, double power, Mode mode){
                     // blocks can also be used as a direct encoder position if the mode is set to DIRECT

        setMode(mode);

        switch (mode){
            case BOTTOM:
                targetPosition = 0;

                liftMotor.setTargetPosition(targetPosition);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                liftMotor.setPower(power);

            case BLOCKS:
                int blocks = position;

                targetPosition = inchesToTicks(blocks);

                liftMotor.setTargetPosition(targetPosition);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                liftMotor.setPower(power);

            case DIRECT:
                targetPosition = position;

                liftMotor.setTargetPosition(targetPosition);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                liftMotor.setPower(power);
        }

    }


    ////////// encoder math, inches to encoder ticks ///////////////


    //TODO test and adjust height math (doesn't seem to be correct)

    public double blocksToTotalHeight(int blocks){
        double height = (blocks * blockHeight) + foundationHeight + extraHeight;
        return (height * -1);
    }


    public int inchesToTicks(int blocks){

        double targetHeight = blocksToTotalHeight(blocks);

        int ticks = (int) ((targetHeight * TICKS_PER_REV) / (Math.PI * radius * 2));

        return ticks;
    }


    /////////////////////// other methods //////////////////////////

    private void setMode(Mode mode){
        this.mode = mode;
    }

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

}
